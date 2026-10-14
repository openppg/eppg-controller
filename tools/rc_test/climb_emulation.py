#!/usr/bin/env python3
"""Headless flight-emulation check for the climb power bar (PR #131, issue #57).

Flies deterministic scenarios through the dashboard's paramotor simulator
(tools/hc_emulator/web/sim.js, run headlessly under node by sim_driver.mjs),
feeds the controller's view of each flight to the real estimators
(climb_eff_replay: src/sp140/climb_efficiency.cpp + thermal_headroom.cpp) the
way the UI task does, and checks what the power bar shows against the
simulator's still-air and thermal ground truth.

    python3 tools/rc_test/climb_emulation.py \
        --replay build-screenshot/climb_eff_replay \
        [--emulator build-screenshot/hc_emulator --frames-dir frames/] \
        [--json results.json] [--work-dir work/] [--scenario NAME ...]

Both binaries are targets of the screenshot-test CMake project
(tools/hc_emulator/README.md). Needs Python 3 (stdlib only) and node >= 18.
Exits non-zero if any check fails.

What the bar shows is derived exactly as main.cpp refreshDisplay() does:
climbBarZones() (ported below from src/sp140/climb_efficiency.cpp) on the
estimator result plus the thermal limits when valid, and the visibility rule
of updateClimbEfficiencyDisplay() (src/sp140/lvgl/lvgl_climb_efficiency.cpp).
With --emulator the real LVGL main screen is also rendered at labelled
moments, saved as PNG, and its bar pixels are checked.
"""

import argparse
import bisect
import hashlib
import json
import math
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import zlib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
NAN = float("nan")
PARTS = ["motor", "escMos", "escCap", "escMcu", "battery", "bmsMos", "bmsBalance"]
PART_LABEL = ["motor", "ESC MOSFET", "ESC capacitor", "ESC MCU", "battery cells",
              "BMS MOSFET", "BMS balance"]

# ---------------------------------------------------------------------------
# Tolerances. Each is taken from a claim in the PR / firmware comments or the
# dashboard's own warning thresholds (tools/hc_emulator/web/app.js).
# ---------------------------------------------------------------------------
LEVEL_TOL_KW = 0.35        # PR: level-power error 0.15 kW mean, 0.31 worst (real flights)
LEVEL_SHOW_WITHIN_S = 300  # PR: tick shows after a median 3.2 min; steady sim cruise <= 5 min
LEVEL_SETTLE_S = 300       # error must hold from 5 min after cruise starts
MARGIN_AT_5MIN_KW = 0.60   # PR: brackets ~0.45 kW 5 min after they appear
MARGIN_FINAL_KW = 0.45     # PR: ~0.32 kW later in the flight (floor 0.25)
MARGIN_COVERAGE = 0.80     # PR: true level inside the brackets 91 % of the time
BAND_SLACK_KW = 0.5        # true P* inside the green band, +/- this
BEST_TOL_KW = 2.0          # dashboard flags |best - true best| > 2 kW
HOT_BELOW_C = 7.0          # yellow/red start held: hottest part ends <= this under threshold
HOT_ABOVE_C = 1.0          #   ... and <= this over it (firmware aims 3 C under:
                           #   thresholdMarginC; PR: 5-min forecasts off 3.4 C motor)
HOT_GRACE_S = 30.0         # once a part is past threshold - margin: the limits are
                           #   filtered (limitSmoothingS 20 s, 10 s steps)
HOT_STEADY_S = 60.0        # "too conservative" is judged only while the true limit
HOT_STEADY_KW = 0.3        #   stayed within this over the last HOT_STEADY_S: the display
                           #   filter halves the gap per 10 s step (tau ~14 s), 60 s ~ 4 tau
EARLY_WARNING_S = 30.0     # PR: zones showed 40-190 s before real crossings
GAP_LEVEL_DRIFT_KW = 0.15  # level with baro gaps vs the same flight without them


# ---------------------------------------------------------------------------
# Firmware constants, read from the headers so the port stays in sync.
# ---------------------------------------------------------------------------
def header_float(text, name):
    m = re.search(r"\b" + name + r"\s*=\s*([-0-9.eE]+)f?\s*;", text)
    if not m:
        sys.exit(f"cannot find {name} in firmware headers")
    return float(m.group(1))


def load_firmware_constants():
    ce = (REPO / "inc/sp140/climb_efficiency.h").read_text()
    th = (REPO / "inc/sp140/thermal_headroom.h").read_text()
    mc = (REPO / "inc/sp140/monitor_config.h").read_text()
    thresholds = {}
    for m in re.finditer(r"Thresholds\s+(\w+)\s*=\s*\{([^}]*)\}", mc):
        fields = dict(re.findall(r"\.(\w+)\s*=\s*([-0-9.]+)", m.group(2)))
        thresholds[m.group(1)] = (float(fields["warnHigh"]), float(fields["critHigh"]))
    # Same mapping as makeThermalHeadroom() in src/sp140/main.cpp.
    source = ["motorTempThresholds", "escMosTempThresholds", "escCapTempThresholds",
              "escMcuTempThresholds", "bmsCellTempThresholds", "bmsTempThresholds",
              "bmsTempThresholds"]
    return {
        "q": header_float(ce, "curvatureQ"),
        "max_kw": header_float(ce, "maxPowerKw"),
        "band_fraction": header_float(ce, "bandFraction"),
        "max_margin_kw": header_float(ce, "maxShownMarginKw"),
        "min_climb_weight": header_float(ce, "minClimbWeight"),
        "margin_c": header_float(th, "thresholdMarginC"),
        "horizon_s": header_float(th, "horizonMaxS"),
        "soc_reserve": header_float(th, "socReservePct"),
        "thresholds": {p: list(thresholds[s]) for p, s in zip(PARTS, source)},
    }


# ---------------------------------------------------------------------------
# What the bar shows: climbBarZones() + updateClimbEfficiencyDisplay().
# ---------------------------------------------------------------------------
def yield_shape(p, level, q):
    x = p - level
    return x * (1.0 - q * x) / p


def bar_zones(r, fw):
    """Port of climbBarZones() in src/sp140/climb_efficiency.cpp."""
    z = {"green": False, "yellow": False, "red": False,
         "green_lo": NAN, "green_hi": NAN, "yellow_lo": NAN, "yellow_hi": NAN,
         "red_lo": NAN, "red_hi": NAN}
    max_p = fw["max_kw"]
    warn_in = r["warn_kw"] if r["th_valid"] else NAN   # main.cpp: heat.valid ? .. : NAN
    crit_in = r["crit_kw"] if r["th_valid"] else NAN
    warn = max_p if math.isnan(warn_in) else min(warn_in, max_p)
    crit = max_p if math.isnan(crit_in) else min(max(crit_in, warn), max_p)
    if warn < max_p and crit > warn:
        z.update(yellow=True, yellow_lo=warn, yellow_hi=crit)
    if crit < max_p:
        z.update(red=True, red_lo=crit, red_hi=max_p)
    q = fw["q"]
    if not (r["curve_valid"] and r["level_valid"] and q > 0):
        return z
    level = r["level"]
    peak = min(math.sqrt(level * level + level / q), max_p)
    best = min(peak, warn)
    if best <= level:
        return z
    c = fw["band_fraction"] * yield_shape(best, level, q)
    b = 1.0 - c
    disc = max(0.0, b * b - 4.0 * q * c * level)
    low = level + (b - math.sqrt(disc)) / (2.0 * q)
    heat_limited = warn < peak
    high = best if heat_limited else min(level + (b + math.sqrt(disc)) / (2.0 * q), warn)
    if high > low:
        z.update(green=True, green_lo=low, green_hi=high)
    return z


def displayed(r, z, armed):
    """updateClimbEfficiencyDisplay(): what is visible (bar enabled, the default)."""
    show = bool(armed) and (bool(r["level_valid"]) or z["green"] or z["yellow"] or z["red"])
    return {
        "bar": show,
        "tick": show and bool(r["level_valid"]),
        "brackets": show and bool(r["level_valid"]) and not math.isnan(r["level_margin"]),
        "green": show and z["green"],
        "yellow": show and z["yellow"],
        "red": show and z["red"],
    }


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------
COOL = {  # cold day, well-cooled motor/ESC: nothing reaches a warning at <= 16 kW
    "env": {"airTempC": 5},
    # 0.25 x the fleet heating gains = the lowest gain the firmware's live fit
    # may use (gainMinRel), so the estimator can represent this setup exactly.
    "thermalScale": {"motor": 0.25, "escMos": 0.25, "escCap": 0.25, "escMcu": 0.25},
}
DEFAULT_DAY = {"env": {"airTempC": 25}}  # sim.js defaults: fleet-fitted part models
HOT = {"env": {"airTempC": 36}}


def seg(name, dur, power, armed=True, ground=False, **kw):
    s = {"name": name, "dur": dur, "power": power, "armed": armed, "ground": ground}
    s.update(kw)
    return s


def preflight(idle1=60, idle2=55, runup=(5, 8.0), takeoff=(12, 14.0)):
    return [
        seg("disarmed", 20, 0, armed=False, ground=True),
        seg("ground_idle", idle1, 0.3, ground=True),
        seg("runup", runup[0], runup[1], ground=True),
        seg("ground_idle2", idle2, 0.3, ground=True),
        seg("takeoff_run", takeoff[0], takeoff[1], ground=True),
    ]


CLIMB_GLIDE = [[75, 13], [60, 0], [75, 10], [60, 0], [75, 13], [60, 0], [90, 8], [60, 0]]


def flight_a(gaps=False):
    """Ground idle -> takeoff -> climb/glide cycles -> 12 min level cruise."""
    g = (lambda *x: list(x)) if gaps else (lambda *x: [])
    return preflight() + [
        seg("climb_out", 120, 13, gaps=g([30, 3])),
        # 10 kW climb at 135-210 s: a 60 s dropout mid-climb; 40 s in a glide
        seg("climb_glide", sum(d for d, _ in CLIMB_GLIDE), {"cycle": CLIMB_GLIDE},
            gaps=g([20, 3], [140, 60], [350, 40], [420, 3], [430, 3])),
        seg("cruise_level", 240, {"level": 0.0}, gaps=g([60, 120], [200, 3])),
        seg("cruise_up", 120, {"level": 0.3}),
        seg("cruise_down", 120, {"level": -0.3}, gaps=g([30, 5])),
        seg("cruise_level2", 240, {"level": 0.0}),
    ]


def scenarios(fw):
    sweep = [
        seg("disarmed", 20, 0, armed=False, ground=True),
        seg("ground_idle", 60, 0.3, ground=True),
        seg("takeoff_run", 12, 14.0, ground=True),
        seg("climb_out", 90, 13),
        seg("cruise_level", 300, {"level": 0.0}),
        seg("sweep", 390, "sweep"),          # sim.js sweepPower(): 4..20 kW, 40 s steps
        seg("glide", 240, 0),
        seg("sweep2", 390, "sweep"),
        seg("cruise_level2", 180, {"level": 0.0}),
    ]
    hot = [
        seg("disarmed", 20, 0, armed=False, ground=True),
        seg("ground_idle", 60, 0.3, ground=True),
        seg("takeoff_run", 10, 12.0, ground=True),
        seg("climb_out", 60, 11),
        seg("cruise_level", 360, {"level": 0.0}),
        # a long climb in steps, as a pilot might push on toward a thermal ceiling
        seg("climb_6.5", 150, 6.5),
        seg("climb_7.5", 150, 7.5),
        seg("climb_8.5", 150, 8.5),
        seg("climb_10", 150, 10.0),
        seg("cruise_cool", 300, {"level": 0.0}),
    ]
    ground = [
        seg("disarmed", 20, 0, armed=False, ground=True),
        seg("ground_idle", 60, 0.3, ground=True),
        seg("runup", 5, 8.0, ground=True),
        seg("ground_idle2", 175, 0.3, ground=True),
        seg("runup2", 3, 15.0, ground=True),
        seg("ground_idle3", 177, 0.3, ground=True),
        seg("runup3", 5, 6.0, ground=True),
        seg("ground_idle4", 175, 0.3, ground=True),
        seg("disarmed_end", 20, 0, armed=False, ground=True),
    ]
    ground_runup = [
        seg("disarmed", 20, 0, armed=False, ground=True),
        seg("ground_idle", 60, 0.3, ground=True),
        seg("ground_run", 240, {"cycle": [[20, 4.0], [20, 9.0]]}, ground=True),
        seg("ground_idle2", 120, 0.3, ground=True),
        seg("disarmed_end", 20, 0, armed=False, ground=True),
    ]
    relaunch = [
        seg("disarmed", 20, 0, armed=False, ground=True),
        seg("ground_idle", 60, 0.3, ground=True),
        seg("takeoff_run", 12, 14.0, ground=True),
        seg("climb_out", 90, 13),
        seg("cruise_level", 360, {"level": 0.0}),
        seg("climb_glide", 270, {"cycle": [[75, 13], [60, 0]]}),
        seg("glide_land", 0, 0, until="landed", maxDur=1500),
        seg("landed_idle", 20, 0.3, ground=True),
        seg("disarmed_landed", 90, 0, armed=False, ground=True),
        seg("ground_idle_rearmed", 90, 0.3, ground=True),
        seg("takeoff_run2", 12, 14.0, ground=True),
        seg("climb_out2", 60, 13),
        seg("cruise_level2", 120, {"level": 0.0}),
    ]
    heavy = {"setup": {"massKg": 140, "sinkMs": 1.45}}
    return {
        "a_flight": {"desc": "ground idle + run-up, takeoff, climb/glide cycles, 12 min level "
                             "cruise (cool day)", "seed": 1, **COOL, "segments": flight_a()},
        "a_heavy": {"desc": "as a_flight, 140 kg / 1.45 m/s sink setup (higher level power)",
                    "seed": 2, **COOL, **heavy, "segments": flight_a()},
        "b_sweep": {"desc": "takeoff, 5 min cruise, two 4->20 kW sweeps (sim.js sweepPower)",
                    "seed": 4, **COOL, "segments": sweep},
        "c_hot": {"desc": "36 C day, takeoff, cruise, long stepped climb 6.5->10 kW", "seed": 3,
                  **HOT, "segments": hot},
        # The fleet-fitted sim parts match the firmware priors exactly, so c_hot
        # never exercises the live fit. Here the motor heats 1.4x faster per kW^2.
        "c2_hot_steep": {"desc": "as c_hot with a motor heating 1.4x the fleet prior (live fit "
                                 "has to learn it)", "seed": 8, **HOT,
                         "thermalScale": {"motor": 1.4}, "segments": hot},
        "d_ground": {"desc": "armed on the ground 10 min, idle with 3 short run-ups, never "
                             "airborne (25 C)", "seed": 5, **DEFAULT_DAY, "segments": ground},
        "d2_ground_runup": {"desc": "armed on the ground, 4 min ground run alternating 4/9 kW, "
                                    "never airborne (25 C)", "seed": 6, **DEFAULT_DAY,
                            "segments": ground_runup},
        "e_baro_gaps": {"desc": "a_flight (same seed, same trajectory) with baro dropouts: "
                                "random 0.2-0.6 s, 3-5 s, 40 s glide, 60 s climb, 120 s cruise",
                        "seed": 1, **COOL, "segments": flight_a(gaps=True),
                        "randomGaps": {"meanEveryS": 20, "minS": 0.2, "maxS": 0.6},
                        "gapSeed": 99},
        "f_relaunch": {"desc": "flight, landing, disarm, re-arm on the ground, second takeoff "
                               "(same power-on)", "seed": 7, **COOL, "segments": relaunch},
    }


# Labelled moments for the real-screen snapshots: (scenario, label, how to pick t).
FRAMES = [
    ("a_flight", "pre_takeoff", ("seg_end", "ground_idle2", -10)),
    ("a_flight", "cruise_tick_band", ("seg_end", "cruise_level2", -5)),
    ("b_sweep", "climb_12kw_band", ("seg_offset", "sweep2", 4 * 40 + 30)),
    ("c_hot", "hot_climb_zones", ("seg_offset", "climb_8.5", 60)),
    ("d2_ground_runup", "ground_runup", ("seg_end", "ground_run", -5)),
    ("f_relaunch", "relaunch_ground_rearmed", ("seg_end", "ground_idle_rearmed", -5)),
]


# ---------------------------------------------------------------------------
# Running things
# ---------------------------------------------------------------------------
def run_driver(node, driver, sim_js, spec):
    out = subprocess.run([node, str(driver), str(sim_js)], input=json.dumps(spec).encode(),
                         capture_output=True, check=False)
    if out.returncode != 0:
        raise RuntimeError(f"sim_driver failed: {out.stderr.decode()[-2000:]}")
    return out.stdout.decode()


def fnum(s):
    return NAN if s in ("", "nan") else float(s)


def parse_driver(text):
    samples, truth, meta = [], [], None
    for line in text.splitlines():
        if line.startswith("S,"):
            v = line.split(",")
            samples.append({
                "t": int(v[1]), "alt": fnum(v[2]), "p": float(v[3]), "armed": int(v[4]),
                "soc": float(v[5]), "temps": [float(x) for x in v[6:13]], "volts": float(v[13]),
                "seg": int(v[14]), "ground": int(v[15]), "true_alt": float(v[16]),
            })
        elif line.startswith("T,"):
            v = line.split(",")
            truth.append({
                "t": int(v[1]), "seg": int(v[2]), "true_alt": float(v[3]), "level": fnum(v[4]),
                "best16": fnum(v[5]), "lo16": fnum(v[6]), "hi16": fnum(v[7]),
                "best_all": fnum(v[8]), "warn_kw": fnum(v[9]), "crit_kw": fnum(v[10]),
                "warn_kw_m": fnum(v[11]), "crit_kw_m": fnum(v[12]),
                "warn_part": int(v[13]), "crit_part": int(v[14]), "horizon": fnum(v[15]),
            })
        elif line.startswith("M,"):
            meta = json.loads(line[2:])
    return samples, truth, meta


def replay_text(samples):
    """climb_eff_replay stdin: t_ms altitude_m power_kw armed soc + 7 temps."""
    def f(v, n):
        return "nan" if math.isnan(v) else f"{v:.{n}f}"
    return "\n".join(
        f"{s['t']} {f(s['alt'], 3)} {s['p']:.3f} {s['armed']} {s['soc']:.2f} "
        + " ".join(f"{x:.2f}" for x in s["temps"]) for s in samples) + "\n"


def run_replay(exe, text):
    out = subprocess.run([str(exe), "every=1000"], input=text.encode(), capture_output=True,
                         check=False)
    if out.returncode != 0:
        raise RuntimeError(f"climb_eff_replay failed: {out.stderr.decode()[-2000:]}")
    lines = out.stdout.decode().splitlines()
    hdr = lines[0].split(",")
    rows = []
    for line in lines[1:]:
        r = {h: fnum(v) for h, v in zip(hdr, line.split(","))}
        for k in ("t_ms", "phase", "curve_valid", "level_valid", "th_valid", "warn_part",
                  "crit_part"):
            r[k] = int(r[k])
        rows.append(r)
    return rows, out.stdout


class Run:
    """One scenario: samples + truth from the sim, replay rows + derived display."""

    def __init__(self, name, spec, samples, truth, meta, rows, fw):
        self.name, self.spec, self.meta, self.fw = name, spec, meta, fw
        self.samples, self.truth, self.rows = samples, truth, rows
        self.sample_at = {s["t"]: s for s in samples}
        self.truth_t = [t["t"] for t in truth]
        for r in rows:
            s = self.sample_at[r["t_ms"]]
            r["s"] = s
            r["truth"] = self.truth_now(r["t_ms"])
            r["z"] = bar_zones(r, fw)
            r["d"] = displayed(r, r["z"], s["armed"])
            r["seg"] = meta["segments"][s["seg"]]["name"]
        self.liftoff = meta["liftoffMs"][0] if meta["liftoffMs"] else None

    def truth_now(self, t):
        k = bisect.bisect_right(self.truth_t, t) - 1
        return self.truth[max(k, 0)]

    def truth_window(self, t0, t1):
        a = max(bisect.bisect_right(self.truth_t, t0) - 1, 0)
        b = bisect.bisect_right(self.truth_t, t1)
        return self.truth[a:max(b, a + 1)]

    def seg(self, name):
        return next(s for s in self.meta["segments"] if s["name"] == name)

    def seg_rows(self, *names):
        return [r for r in self.rows if r["seg"] in names]


# ---------------------------------------------------------------------------
# Thermal truth at a given power (sim.js part models, as in sim_driver.mjs).
# ---------------------------------------------------------------------------
def hold_excess(run, sample, power, which, detail=False):
    """Hottest part's peak temperature over the hold minus its threshold (C).

    With detail=True also returns whether that peak is the part's present
    temperature (it is cooling at this power, so the peak does not depend on
    the power held)."""
    fw, meta = run.fw, run.meta
    pack = run.spec.get("packKwh", 4.8)
    if power <= 0.1:
        h = fw["horizon_s"]
    else:
        usable = (sample["soc"] - fw["soc_reserve"]) / 100.0 * pack
        h = 30.0 if usable <= 0 else min(max(usable * 3600.0 / power, 30.0), fw["horizon_s"])
    air = meta["env"]["airTempC"]
    worst, who, now = -1e9, -1, False
    for i, part in enumerate(PARTS):
        m = meta["thermalParts"][part]
        t0 = sample["temps"][i]
        tss = air + m["baseOverAir"] + m["k"] * power * power
        end = tss + (t0 - tss) * math.exp(-h / m["tau"])
        ex = max(t0, end) - fw["thresholds"][part][which]
        if ex > worst:
            worst, who, now = ex, i, t0 >= end
    return (worst, who, now) if detail else (worst, who)


def first_crossing(run, which):
    for s in run.samples:
        for i, part in enumerate(PARTS):
            if s["temps"][i] >= run.fw["thresholds"][part][which]:
                return s["t"], i
    return None, None


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
class Results:
    def __init__(self):
        self.items = []

    def add(self, cid, name, ok, detail):
        self.items.append({"id": cid, "name": name, "status": "pass" if ok else "fail",
                           "detail": detail})


def fmt(v, n=2):
    return "n/a" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{n}f}"


def shown_summary(rows):
    keys = ["tick", "brackets", "green", "yellow", "red"]
    return ", ".join(f"{k}={sum(1 for r in rows if r['d'][k])}" for k in keys)


def check_nothing_before_takeoff(res, runs):
    for name in ("a_flight", "a_heavy", "b_sweep", "c_hot", "e_baro_gaps", "f_relaunch"):
        run = runs.get(name)
        if not run:
            continue
        pre = [r for r in run.rows if run.liftoff is None or r["t_ms"] < run.liftoff]
        armed = [r for r in pre if r["s"]["armed"]]
        shown = [r for r in pre if r["d"]["bar"]]
        first = next((r for r in run.rows if r["d"]["bar"]), None)
        after = (f"; bar first shown {(first['t_ms'] - run.liftoff) / 1000:.0f} s after "
                 f"lift-off ({first['seg']})") if first and run.liftoff else ""
        res.add(f"pre_takeoff_{name}", f"nothing shown before takeoff ({name})", not shown,
                f"{len(pre)} rows before lift-off ({len(armed)} armed: idle, run-up, takeoff run "
                f"on the ground): shown {len(shown)} [{shown_summary(pre)}]{after}")


def check_ground_only(res, runs, name, cid, label):
    run = runs.get(name)
    if not run:
        return
    shown = [r for r in run.rows if r["d"]["bar"]]
    lw = max(r["level_w"] for r in run.rows)
    cw = max(r["climb_w"] for r in run.rows)
    thv = [r for r in run.rows if r["th_valid"]]
    detail = (f"{len(run.rows)} rows, never airborne (max true alt "
              f"{max(s['true_alt'] for s in run.samples):.1f} m): shown {len(shown)} "
              f"[{shown_summary(run.rows)}]; max level_w {lw:.1f}, climb_w {cw:.1f}; "
              f"thermal model valid in {len(thv)} rows")
    if shown:
        r0 = shown[0]
        detail += (f"; first shown at {r0['t_ms'] / 1000:.0f} s ({r0['seg']}): yellow from "
                   f"{fmt(r0['z']['yellow_lo'])} kW, red from {fmt(r0['z']['red_lo'])} kW, "
                   f"limiter {PART_LABEL[r0['warn_part']] if r0['warn_part'] >= 0 else '-'}; "
                   f"last shown {shown[-1]['t_ms'] / 1000:.0f} s ({shown[-1]['seg']})")
    res.add(cid, label, not shown and lw == 0 and cw == 0, detail)


def level_rows(run, cruise_name):
    t_c = run.seg(cruise_name)["startMs"]
    first = next((r for r in run.rows if r["d"]["tick"]), None)
    return t_c, first


def check_level(res, runs):
    for name, cruise in (("a_flight", "cruise_level"), ("a_heavy", "cruise_level"),
                         ("b_sweep", "cruise_level"), ("e_baro_gaps", "cruise_level")):
        run = runs.get(name)
        if not run:
            continue
        t_c, first = level_rows(run, cruise)
        if first is None:
            res.add(f"level_converges_{name}", f"level tick converges to true level ({name})",
                    False, "pink tick never shown")
            continue
        # The convergence clock counts level flight with baro data: a dropout
        # in the cruise (e_baro_gaps) delays it by the dropout's length.
        t_settle, have = None, 0
        for s in run.samples:
            if s["t"] >= t_c and not math.isnan(s["alt"]):
                have += 1
                if have >= LEVEL_SETTLE_S * 30:
                    t_settle = s["t"]
                    break
        late = [r for r in run.rows if t_settle is not None and r["t_ms"] >= t_settle
                and r["s"]["armed"] and r["s"]["true_alt"] > 0.5]
        wall = next((r for r in run.rows if r["t_ms"] >= t_c + LEVEL_SETTLE_S * 1000), None)
        wall_note = ""
        if wall and t_settle and t_settle - wall["t_ms"] > 5000 and wall["level_valid"]:
            wall_note = (f" (baro dropouts in the cruise; at cruise+{LEVEL_SETTLE_S / 60:.0f} min "
                         f"wall clock the error was "
                         f"{wall['level'] - wall['truth']['level']:+.3f} kW, margin "
                         f"{wall['level_margin']:.2f})")
        errs = [r["level"] - r["truth"]["level"] for r in late if r["level_valid"]]
        hidden = [r for r in late if not r["d"]["tick"]]
        truth_l = [r["truth"]["level"] for r in late] or [NAN]
        e5 = errs[0] if errs else NAN
        emax = max((abs(e) for e in errs), default=NAN)
        show_after_cruise = (first["t_ms"] - t_c) / 1000.0
        ok = (show_after_cruise <= LEVEL_SHOW_WITHIN_S and bool(errs) and not hidden
              and emax <= LEVEL_TOL_KW)
        res.add(f"level_converges_{name}", f"level tick converges to true level ({name})", ok,
                f"true level {min(truth_l):.2f}-{max(truth_l):.2f} kW (still air at altitude); "
                f"tick first shown {(first['t_ms'] - run.liftoff) / 60000:.1f} min after "
                f"lift-off = {show_after_cruise / 60:.1f} min into level cruise (limit "
                f"{LEVEL_SHOW_WITHIN_S / 60:.0f} min); error from "
                f"{(t_settle - t_c) / 60000 if t_settle else NAN:.1f} min into the cruise "
                f"({LEVEL_SETTLE_S / 60:.0f} min of level flight with baro data) to the end: "
                f"first {fmt(e5, 3)}, max |err| {fmt(emax, 3)}, final "
                f"{fmt(errs[-1] if errs else NAN, 3)} kW (tol {LEVEL_TOL_KW} kW); "
                f"tick hidden in {len(hidden)} of {len(late)} rows{wall_note}")

        # Brackets
        shown = [r for r in run.rows if r["d"]["brackets"]]
        m0 = shown[0]["level_margin"]
        t0 = shown[0]["t_ms"]
        at5 = next((r for r in shown if r["t_ms"] >= t0 + 300000), None)
        m5 = at5["level_margin"] if at5 else NAN
        mf = shown[-1]["level_margin"]
        span = (shown[-1]["t_ms"] - t0) / 60000.0
        inside = [abs(r["level"] - r["truth"]["level"]) <= r["level_margin"] for r in shown]
        cov = sum(inside) / len(inside)
        px = lambda m: max(2, int(m / 16.0 * 95 + 0.5))  # noqa: E731 lvgl_climb_efficiency.cpp
        ok = (m0 <= run.fw["max_margin_kw"] + 1e-6 and not math.isnan(m5) and m5 < m0
              and m5 <= MARGIN_AT_5MIN_KW and mf <= MARGIN_FINAL_KW and mf < m0
              and cov >= MARGIN_COVERAGE)
        res.add(f"level_brackets_{name}", f"tick brackets shrink and cover the truth ({name})",
                ok, f"margin {fmt(m0, 3)} kW when the tick appears (limit "
                    f"{run.fw['max_margin_kw']}), {fmt(m5, 3)} after 5 min (limit "
                    f"{MARGIN_AT_5MIN_KW}), {fmt(mf, 3)} at the end, {span:.1f} min later "
                    f"(limit {MARGIN_FINAL_KW}); bracket half-width {px(m0)} -> {px(mf)} px; "
                    f"true level inside the brackets {cov * 100:.0f} % of {len(shown)} rows "
                    f"(limit {MARGIN_COVERAGE * 100:.0f} %)")


def check_band(res, runs):
    for name in ("a_flight", "a_heavy", "b_sweep"):
        run = runs.get(name)
        if not run:
            continue
        rows = [r for r in run.rows if r["d"]["green"]]
        if not rows:
            res.add(f"band_true_best_{name}", f"green band contains true best climb ({name})",
                    False, "green band never shown")
            continue
        outside = [r for r in rows if not (r["z"]["green_lo"] - BAND_SLACK_KW
                                           <= r["truth"]["best16"]
                                           <= r["z"]["green_hi"] + BAND_SLACK_KW)]
        best_err = [r["best"] - r["truth"]["best16"] for r in rows]
        bmax = max(abs(e) for e in best_err)
        last = rows[-1]
        t = last["truth"]
        a0, a1 = last["z"]["green_lo"], last["z"]["green_hi"]
        inter = max(0.0, min(a1, t["hi16"]) - max(a0, t["lo16"]))
        union = max(a1, t["hi16"]) - min(a0, t["lo16"])
        lo_err = [r["z"]["green_lo"] - r["truth"]["lo16"] for r in rows]
        climbs_before = rows[0]["climb_w"]
        ok = not outside and bmax <= BEST_TOL_KW
        res.add(f"band_true_best_{name}", f"green band contains true best climb ({name})", ok,
                f"band shown in {len(rows)} rows from {(rows[0]['t_ms'] - run.liftoff) / 60000:.1f} "
                f"min after lift-off (climb_w {climbs_before:.0f}); final band "
                f"{a0:.2f}-{a1:.2f} kW vs true 95 % band {t['lo16']:.2f}-{t['hi16']:.2f} "
                f"(IoU {inter / union:.2f}); true P* {t['best16']:.2f} kW (uncapped "
                f"{t['best_all']:.2f}) vs firmware P* {last['best']:.2f}; P* outside band "
                f"+/-{BAND_SLACK_KW} kW in {len(outside)} rows; max |P* err| {bmax:.2f} kW "
                f"(tol {BEST_TOL_KW}); band low edge err {min(lo_err):+.2f}..{max(lo_err):+.2f} kW")


def check_hot(res, runs, name):
    run = runs.get(name)
    if not run:
        return
    fw = run.fw
    tag = f" ({name})"
    y_rows = [r for r in run.rows if r["d"]["yellow"]]
    r_rows = [r for r in run.rows if r["d"]["red"]]
    t_y_show = y_rows[0]["t_ms"] if y_rows else None
    t_r_show = r_rows[0]["t_ms"] if r_rows else None
    # The pilot's power tick (instantaneous power) inside the zones.
    in_y = next((r for r in run.rows if (r["d"]["yellow"] and r["s"]["p"] >= r["z"]["yellow_lo"])
                 or (r["d"]["red"] and r["s"]["p"] >= r["z"]["red_lo"])), None)
    in_r = next((r for r in run.rows if r["d"]["red"] and r["s"]["p"] >= r["z"]["red_lo"]), None)
    geometry = all(abs(r["z"]["yellow_hi"] - r["z"]["red_lo"]) < 1e-6
                   for r in run.rows if r["d"]["yellow"] and r["d"]["red"])
    ok = (t_y_show is not None and t_r_show is not None and t_y_show <= t_r_show and
          in_y is not None and in_r is not None and in_y["t_ms"] < in_r["t_ms"] and geometry)
    sec = lambda t: f"{t / 1000:.0f} s" if t is not None else "never"  # noqa: E731
    detail = (f"zones first shown: yellow {sec(t_y_show)}, red {sec(t_r_show)} (thermal model "
              f"learned {(t_y_show - run.liftoff) / 1000:.0f} s after lift-off)"
              if t_y_show else "yellow never shown")
    if in_y and in_r:
        detail += (f"; pilot's power tick enters yellow at {sec(in_y['t_ms'])} ({in_y['seg']}, "
                   f"{in_y['s']['p']:.2f} kW >= {in_y['z']['yellow_lo']:.2f}) and red at "
                   f"{sec(in_r['t_ms'])} ({in_r['seg']}, {in_r['s']['p']:.2f} kW >= "
                   f"{in_r['z']['red_lo']:.2f}); yellow ends where red starts: {geometry}")
    res.add(f"hot_zone_order_{name}", f"hot day: yellow before red{tag}", ok, detail)

    # Early warning: the tick was in the zone before the sim's part actually got there.
    tw, pw = first_crossing(run, 0)
    tc, pc = first_crossing(run, 1)
    lead_w = (tw - in_y["t_ms"]) / 1000.0 if tw and in_y else NAN
    lead_c = (tc - in_r["t_ms"]) / 1000.0 if tc and in_r else NAN
    ok = (not math.isnan(lead_w) and not math.isnan(lead_c) and lead_w >= EARLY_WARNING_S
          and lead_c >= EARLY_WARNING_S)
    res.add(f"hot_early_warning_{name}", f"hot day: zones warn before the real crossings{tag}", ok,
            f"sim {PART_LABEL[pw] if pw is not None else '-'} reaches warning "
            f"({fw['thresholds'][PARTS[pw]][0] if pw is not None else '-'} C) at {sec(tw)}, "
            f"{PART_LABEL[pc] if pc is not None else '-'} reaches critical "
            f"({fw['thresholds'][PARTS[pc]][1] if pc is not None else '-'} C) at {sec(tc)}; "
            f"power tick was in yellow-or-red {fmt(lead_w, 0)} s and in red {fmt(lead_c, 0)} s "
            f"earlier (need >= {EARLY_WARNING_S:.0f} s; PR fleet: 40-190 s)")

    # Zone starts vs the power that, held, really reaches the threshold. Two
    # regimes: (A) every part is under (threshold - margin) in the sim - the
    # start must be a power whose hold ends just under the threshold; (B) a
    # part is already past it - any climb is too much, so once the 20 s display
    # filter has caught up (HOT_GRACE_S) the zone must cover every power above
    # level flight.
    for which, cid, label in ((0, "hot_yellow_start", "warning"),
                              (1, "hot_red_start", "critical")):
        truth_key = "warn_kw" if which == 0 else "crit_kw"
        excess, dkw, bad_a, bad_b, missed = [], [], [], [], 0
        n_a = n_b = 0
        cooling_lag = []  # (A) rows where the hottest part cools at the start power
        moving_lag = []   # (A) rows where the true limit is moving (filter lag)
        over_since = None
        for r in run.rows:
            if not r["s"]["armed"]:
                continue
            over = r["truth"][truth_key + "_m"] <= 0.0
            over_since = (over_since or r["t_ms"]) if over else None
            if which == 0:  # where the "would reach warning" colouring starts
                start = r["z"]["yellow_lo"] if r["d"]["yellow"] else \
                    (r["z"]["red_lo"] if r["d"]["red"] else None)
            else:
                start = r["z"]["red_lo"] if r["d"]["red"] else None
            if not over:
                if start is None:
                    missed += r["truth"][truth_key] < fw["max_kw"]
                    continue
                n_a += 1
                ex, who, at_now = hold_excess(run, r["s"], start, which, detail=True)
                dkw.append(start - r["truth"][truth_key])
                if at_now:
                    # The part is cooling at the start power, so the hold's peak
                    # is today's temperature whatever the start is: only the
                    # safety side (not over the threshold) can be judged. Keep
                    # how far the start lags the firmware's own target, as seen
                    # with the sim's true parameters (the 20 s display filter).
                    cooling_lag.append(r["truth"][truth_key + "_m"] - start)
                    if ex > HOT_ABOVE_C:
                        bad_a.append((r, start, ex, who))
                    continue
                # The displayed limits are filtered (limitSmoothingS): while the
                # true limit moves (a part heating or cooling fast), the start
                # trails it. Too-late is judged in every row; too-early only
                # while the true limit is steady.
                recent = run.truth_window(r["t_ms"] - HOT_STEADY_S * 1000, r["t_ms"])
                vals = [x[truth_key + "_m"] for x in recent]
                if max(vals) - min(vals) > HOT_STEADY_KW:
                    moving_lag.append(r["truth"][truth_key + "_m"] - start)
                    if ex > HOT_ABOVE_C:
                        bad_a.append((r, start, ex, who))
                    continue
                excess.append(ex)
                if not -HOT_BELOW_C <= ex <= HOT_ABOVE_C:
                    bad_a.append((r, start, ex, who))
            elif r["t_ms"] - over_since >= HOT_GRACE_S * 1000:
                n_b += 1
                if start is None or start > r["truth"]["level"]:
                    bad_b.append((r, start))
        if not excess:
            res.add(f"{cid}_{name}", f"hot day: {label} zone start held reaches {label}{tag}",
                    False, "zone never shown")
            continue
        srt = sorted(excess)
        detail = (f"(A) {n_a} rows with every part under its {label} threshold - "
                  f"{fw['margin_c']:.0f} C: holding the zone-start power for the climb horizon "
                  f"(heating rows), the sim's hottest part peaks {srt[0]:+.1f} to {srt[-1]:+.1f} "
                  f"C (median "
                  f"{srt[len(srt) // 2]:+.1f}) from its {label} threshold (allowed "
                  f"-{HOT_BELOW_C:.0f}..+{HOT_ABOVE_C:.0f} C; firmware aims "
                  f"-{fw['margin_c']:.0f} C); zone start - true no-margin limit "
                  f"{min(dkw):+.2f}..{max(dkw):+.2f} kW; {len(cooling_lag)} of these rows have "
                  f"the part cooling at the start power (only 'not over' judged; start "
                  f"{fmt(max(cooling_lag, default=NAN))} kW at most below the firmware's own "
                  f"target with true parameters: display filter lag, conservative); "
                  f"{len(moving_lag)} rows with the true limit moving > {HOT_STEADY_KW} kW within "
                  f"{HOT_STEADY_S:.0f} s (only 'not over' judged; start trails the firmware "
                  f"target by {fmt(min(moving_lag, default=NAN))}..{fmt(max(moving_lag, default=NAN))}"
                  f" kW); out of range in {len(bad_a)}; "
                  f"(B) {n_b} rows with a part already past it (after {HOT_GRACE_S:.0f} s): zone "
                  f"covers every power above level flight in {n_b - len(bad_b)}; "
                  f"truth limit < {fw['max_kw']:.0f} kW but no zone (thermal model still "
                  f"learning) in {missed} rows")
        if bad_a:
            r, start, ex, who = bad_a[0]
            detail += (f"; first (A) miss at {r['t_ms'] / 1000:.0f} s ({r['seg']}): start "
                       f"{start:.2f} kW, {PART_LABEL[who]} would peak {ex:+.1f} C")
        if bad_b:
            r, start = bad_b[0]
            detail += (f"; first (B) miss at {r['t_ms'] / 1000:.0f} s ({r['seg']}): start "
                       f"{fmt(start)} kW vs level {r['truth']['level']:.2f} kW")
        res.add(f"{cid}_{name}", f"hot day: {label} zone start held reaches {label}{tag}",
                not bad_a and not bad_b, detail)

    # Green band stops at the heat limit.
    both = [r for r in run.rows if r["d"]["green"] and (r["d"]["yellow"] or r["d"]["red"])]
    overlap = [r for r in both if r["z"]["green_hi"] > (r["z"]["yellow_lo"] if r["d"]["yellow"]
                                                        else r["z"]["red_lo"]) + 1e-6]
    greens = [r for r in run.rows if r["d"]["green"]]
    detail = (f"green shown in {len(greens)} rows, with a thermal zone in {len(both)}; green "
              f"overlaps a zone in {len(overlap)}; learned motor gain at the end "
              f"{run.rows[-1]['gain_motor']:.2f} C/kW^2 (sim truth "
              f"{run.meta['thermalParts']['motor']['k']:.2f})")
    if both:
        r = both[len(both) // 2]
        detail += (f"; e.g. {r['t_ms'] / 1000:.0f} s: green {r['z']['green_lo']:.2f}-"
                   f"{r['z']['green_hi']:.2f}, yellow from {fmt(r['z']['yellow_lo'])}, red from "
                   f"{fmt(r['z']['red_lo'])}, efficiency peak {r['best']:.2f} kW")
    res.add(f"hot_green_capped_{name}", f"hot day: green band ends at the heat limit{tag}",
            bool(both) and not overlap, detail)


def check_cool(res, runs):
    for name in ("a_flight", "a_heavy", "b_sweep", "e_baro_gaps", "f_relaunch"):
        run = runs.get(name)
        if not run:
            continue
        zones = [r for r in run.rows if r["d"]["yellow"] or r["d"]["red"]]
        flying = [r for r in run.rows if r["s"]["armed"]]
        tmin = min(r["truth"]["warn_kw"] for r in flying)
        th_valid = sum(1 for r in run.rows if r["th_valid"])
        fw_min = min((r["warn_kw"] for r in run.rows if r["th_valid"]), default=NAN)
        detail = (f"truth: lowest power that would reach a warning within the horizon "
                  f"{tmin:.2f} kW (>= {run.fw['max_kw']:.0f} = none in the bar); firmware "
                  f"thermal model valid in {th_valid} rows, lowest warn limit {fmt(fw_min)} kW, "
                  f"learned motor gain {run.rows[-1]['gain_motor']:.2f} C/kW^2 (sim "
                  f"{run.meta['thermalParts']['motor']['k']:.2f}, prior 1.00); "
                  f"yellow/red shown in {len(zones)} rows")
        if zones:
            r = zones[0]
            detail += (f"; first at {r['t_ms'] / 1000:.0f} s ({r['seg']}): yellow from "
                       f"{fmt(r['z']['yellow_lo'])}, red from {fmt(r['z']['red_lo'])} kW, "
                       f"limiter {PART_LABEL[r['warn_part']] if r['warn_part'] >= 0 else '-'}")
        ok = not zones and tmin >= run.fw["max_kw"] - 1e-6
        res.add(f"no_zones_cool_{name}", f"no yellow/red on a cool flight ({name})", ok, detail)


def check_gaps(res, runs):
    run = runs.get("e_baro_gaps")
    if not run:
        return
    base = runs.get("a_flight")
    samples = run.samples
    nan_n = sum(1 for s in samples if math.isnan(s["alt"]))
    # Gap intervals from the samples themselves (contiguous NaN runs).
    gaps, start = [], None
    for s in samples:
        if math.isnan(s["alt"]):
            start = s["t"] if start is None else start
            last = s["t"]
        elif start is not None:
            gaps.append((start, last))
            start = None
    long_gaps = [(a, b) for a, b in gaps if b - a >= 1500]
    learned_in_gap, turned_on = [], []
    for a, b in long_gaps:
        # Rows are printed after the sample at their time: the first row inside
        # the dropout holds everything learned before it.
        inside = [r for r in run.rows if a <= r["t_ms"] <= b]
        if len(inside) < 2:
            continue
        ref = inside[0]
        for r in inside[1:]:
            if r["level_w"] > ref["level_w"] + 1e-6 or r["climb_w"] > ref["climb_w"] + 1e-6:
                learned_in_gap.append((a, b, r))
                break
            if r["level_valid"] and not ref["level_valid"]:
                turned_on.append((a, b, r))
                break
    detail = (f"{nan_n} NaN altitude samples in {len(gaps)} dropouts ({len(long_gaps)} >= 1.5 s, "
              f"longest {max(b - a for a, b in gaps) / 1000:.0f} s); learning during a "
              f"dropout: {len(learned_in_gap)}, tick switched on during one: {len(turned_on)}")
    phases = [r["phase"] for r in run.rows if any(a <= r["t_ms"] <= b for a, b in long_gaps)]
    detail += f"; estimator phase inside dropouts: {sorted(set(phases))} (0 = NO_DATA)"
    res.add("baro_gap_no_learning", "baro dropouts teach the estimator nothing",
            not learned_in_gap and not turned_on and nan_n > 0, detail)

    # The level estimate holds through the dropouts and ends where the clean flight does.
    if base:
        last_e = [r for r in run.rows if r["level_valid"]]
        last_a = [r for r in base.rows if r["level_valid"]]
        cruise_gap = next(((a, b) for a, b in long_gaps if b - a >= 100000), None)
        held = NAN
        if cruise_gap:
            inside = [r["level"] for r in run.rows
                      if cruise_gap[0] <= r["t_ms"] <= cruise_gap[1] and r["level_valid"]]
            held = (max(inside) - min(inside)) if inside else NAN
        if last_e and last_a:
            d = last_e[-1]["level"] - last_a[-1]["level"]
            err = last_e[-1]["level"] - last_e[-1]["truth"]["level"]
            first_e = (last_e[0]["t_ms"] - run.liftoff) / 60000.0
            first_a = (last_a[0]["t_ms"] - base.liftoff) / 60000.0
            ok = abs(d) <= GAP_LEVEL_DRIFT_KW and abs(err) <= LEVEL_TOL_KW and \
                (math.isnan(held) or held < 1e-6)
            res.add("baro_gap_level_holds", "level estimate holds through baro dropouts", ok,
                    f"final level {last_e[-1]['level']:.3f} kW with dropouts vs "
                    f"{last_a[-1]['level']:.3f} without ({d:+.3f}, tol {GAP_LEVEL_DRIFT_KW}); "
                    f"vs truth {err:+.3f} (tol {LEVEL_TOL_KW}); tick first shown {first_e:.1f} "
                    f"min after lift-off (clean flight {first_a:.1f}); shown value moved "
                    f"{fmt(held, 3)} kW during the 120 s cruise dropout")
        else:
            res.add("baro_gap_level_holds", "level estimate holds through baro dropouts", False,
                    "no level estimate with or without dropouts")


def check_relaunch(res, runs):
    run = runs.get("f_relaunch")
    if not run or len(run.meta["liftoffMs"]) < 2:
        if run:
            res.add("relaunch_ground", "relaunch: learned values kept across disarm", False,
                    "second takeoff not reached")
        return
    land = run.meta["landingMs"][0]
    rearm = run.meta["armMs"][1]
    lift2 = run.meta["liftoffMs"][1]
    before_land = [r for r in run.rows if r["t_ms"] < land and r["level_valid"]]
    ground2 = [r for r in run.rows if rearm <= r["t_ms"] < lift2]
    kept = bool(before_land) and all(r["level_valid"] for r in ground2) and \
        abs(ground2[0]["level"] - before_land[-1]["level"]) < 0.02
    shown = [r for r in ground2 if r["d"]["bar"]]
    res.add("relaunch_ground", "relaunch: learned values kept across disarm (documented)", kept,
            f"after landing, disarm and re-arm on the ground, {len(shown)} of {len(ground2)} "
            f"rows before the second lift-off show the bar [{shown_summary(ground2)}] using the "
            f"previous flight's level {fmt(ground2[0]['level'], 2)} kW; matches "
            f"test_climb_efficiency DisarmClearsWindowButKeepsWhatWasLearned, but it is a "
            f"tick/band on the ground before takeoff, unlike the per-flight 'nothing before "
            f"takeoff' wording in the PR")


def check_determinism(res, node, driver, sim_js, replay, spec, first_out, first_replay):
    again = run_driver(node, driver, sim_js, spec)
    s2, _, _ = parse_driver(again)
    _, raw2 = run_replay(replay, replay_text(s2))
    h = lambda b: hashlib.sha256(b if isinstance(b, bytes) else b.encode()).hexdigest()[:12]  # noqa
    ok = h(again) == h(first_out) and h(raw2) == h(first_replay)
    res.add("deterministic", "same seed, same flight and same estimator output", ok,
            f"a_flight sim output sha256 {h(first_out)} / {h(again)}, replay output "
            f"{h(first_replay)} / {h(raw2)}")


# ---------------------------------------------------------------------------
# Real screen: hc_emulator snapshots
# ---------------------------------------------------------------------------
W, H = 160, 128
BAR_REGION = (0, 61, 103, 72)  # x0, y0, x1, y1 (exclusive): kBarY 65 +/- the 7 px power tick


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


BAR_COLORS = {  # lvgl_climb_efficiency.cpp setupClimbEfficiencyWidgets()
    1: {"track": (64, 64, 64), "green": (0, 112, 48), "yellow": (210, 170, 0),
        "red": (200, 30, 40), "pink": (255, 80, 190)},
    0: {"track": (200, 200, 200), "green": (0, 170, 0), "yellow": (235, 185, 0),
        "red": (225, 0, 0), "pink": (220, 0, 140)},
}


def emulator_lines(samples, renders_at):
    """hc_emulator commands (the dashboard's sampleLine) with renders at given t_ms."""
    out = []
    prev = 1  # eppgVirtualMillis starts at 1
    todo = sorted(renders_at)
    for s in samples:
        def f(v, n):
            return "nan" if math.isnan(v) else f"{v:.{n}f}"
        t = s["temps"]
        out.append(f"s dt={s['t'] - prev} alt={f(s['alt'], 3)} p={s['p']:.3f} soc={s['soc']:.1f} "
                   f"v={s['volts']:.1f} armed={s['armed']} cruise=0 bt={t[4]:.1f} et={t[1]:.1f} "
                   f"mt={t[0]:.1f} ec={t[2]:.1f} em={t[3]:.1f} bm={t[5]:.1f} bb={t[6]:.1f} "
                   f"bms=1 esc=1")
        prev = s["t"]
        while todo and s["t"] >= todo[0]:
            out.append("render")
            todo.pop(0)
        if not todo:
            break
    return out


def run_emulator(exe, theme, lines):
    head = [f"init theme={theme} metric=1 perf=1", "bar v=1"]
    proc = subprocess.run([str(exe)], input=("\n".join(head + lines) + "\nquit\n").encode(),
                          capture_output=True, check=False)
    data, frames, pos = proc.stdout, [], 0
    while pos < len(data):
        nl = data.index(b"\n", pos)
        header = json.loads(data[pos:nl])
        n = header["frame"]
        frames.append((header, data[nl + 1:nl + 1 + n]))
        pos = nl + 1 + n
    return frames


def frame_rgb(frame):
    px = struct.unpack(f"<{W * H}H", frame)
    return px


def write_png(path, px, scale=3):
    rows = []
    for y in range(H):
        line = bytearray()
        for x in range(W):
            v = px[y * W + x]
            rgb = bytes(((((v >> 11) & 31) * 527 + 23) >> 6, (((v >> 5) & 63) * 259 + 33) >> 6,
                         ((v & 31) * 527 + 23) >> 6))
            line += rgb * scale
        rows.extend([b"\x00" + bytes(line)] * scale)
    raw = b"".join(rows)

    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + \
            struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W * scale, H * scale,
                                                             8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    Path(path).write_bytes(png)


def bar_pixels(px, theme):
    x0, y0, x1, y1 = BAR_REGION
    counts = {}
    for name, rgb in BAR_COLORS[theme].items():
        v = rgb565(*rgb)
        counts[name] = sum(1 for y in range(y0, y1) for x in range(x0, x1) if px[y * W + x] == v)
    return counts


def pick_time(run, how):
    kind, segname, off = how
    s = run.seg(segname)
    t = s["endMs"] + off * 1000 if kind == "seg_end" else s["startMs"] + off * 1000
    return min(max(t, run.samples[0]["t"]), run.samples[-1]["t"])


def do_frames(res, runs, exe, frames_dir):
    jobs = {}
    for scen, label, how in FRAMES:
        if scen in runs:
            jobs.setdefault(scen, []).append((label, pick_time(runs[scen], how)))
    expect = {
        "pre_takeoff": {"none": True},
        "cruise_tick_band": {"need": ["pink", "green", "track"], "none_of": ["yellow", "red"]},
        "climb_12kw_band": {"need": ["pink", "green", "track"], "none_of": ["yellow", "red"]},
        "hot_climb_zones": {"need": ["yellow", "red", "track"]},
    }
    if frames_dir:
        Path(frames_dir).mkdir(parents=True, exist_ok=True)

    def one(scen, theme):
        items = jobs[scen]
        lines = emulator_lines(runs[scen].samples, [t for _, t in items])
        return scen, theme, run_emulator(exe, theme, lines)

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda a: one(*a), [(s, th) for s in jobs for th in (1, 0)]))
    saved, problems, notes = [], [], []
    for scen, theme, frames in results:
        items = sorted(jobs[scen], key=lambda x: x[1])
        if len(frames) != len(items):
            problems.append(f"{scen} theme {theme}: {len(frames)} frames for {len(items)} renders")
            continue
        for (label, t), (header, frame) in zip(items, frames):
            px = frame_rgb(frame)
            tname = "dark" if theme == 1 else "light"
            if frames_dir:
                path = Path(frames_dir) / f"{scen}__{label}__{tname}.png"
                write_png(path, px)
                saved.append(path.name)
            counts = bar_pixels(px, theme)
            exp = expect.get(label)
            if not exp:
                notes.append(f"{label}/{tname}: " + ",".join(f"{k}={v}" for k, v in counts.items()
                                                              if v))
                continue
            if exp.get("none") and any(counts.values()):
                problems.append(f"{label}/{tname}: bar pixels on screen {counts}")
            for k in exp.get("need", []):
                if counts[k] == 0:
                    problems.append(f"{label}/{tname}: no {k} pixels {counts}")
            for k in exp.get("none_of", []):
                if counts[k]:
                    problems.append(f"{label}/{tname}: unexpected {k} pixels {counts}")
            # The emulator's own estimator should agree with the batch replay.
            r = min(runs[scen].rows, key=lambda row: abs(row["t_ms"] - header["t"]))
            if bool(header["levelValid"]) != bool(r["level_valid"]):
                problems.append(f"{label}/{tname}: emulator levelValid {header['levelValid']} vs "
                                f"replay {r['level_valid']}")
    res.add("real_screen_bar", "real LVGL screen draws what the replay predicts",
            not problems and bool(saved or not frames_dir),
            f"{sum(len(v) for v in jobs.values()) * 2} frames rendered (dark + light)"
            + (f", saved to {frames_dir}" if frames_dir else "")
            + (f"; problems: {'; '.join(problems)}" if problems else
               "; pre-takeoff frames have no bar pixels, band frames show green + pink tick, "
               "hot frame shows yellow + red")
            + (f"; evidence frames: {'; '.join(notes)}" if notes else ""))
    return saved


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--replay", required=True, type=Path, help="climb_eff_replay binary")
    ap.add_argument("--emulator", type=Path, help="hc_emulator binary (optional: PNG frames)")
    ap.add_argument("--frames-dir", type=Path, help="where to save PNG snapshots")
    ap.add_argument("--json", type=Path, help="write check results here")
    ap.add_argument("--node", default="node")
    ap.add_argument("--sim", type=Path, default=REPO / "tools/hc_emulator/web/sim.js")
    ap.add_argument("--driver", type=Path, default=HERE / "sim_driver.mjs")
    ap.add_argument("--work-dir", type=Path, help="keep per-scenario inputs/outputs here")
    ap.add_argument("--scenario", action="append", help="run only these (repeatable)")
    args = ap.parse_args()

    t_start = time.time()
    fw = load_firmware_constants()
    specs = scenarios(fw)
    names = args.scenario or list(specs)
    work = args.work_dir or Path(tempfile.mkdtemp(prefix="climb_emu_"))
    work.mkdir(parents=True, exist_ok=True)

    def run_one(name):
        spec = dict(specs[name])
        spec["thresholds"] = fw["thresholds"]
        spec["marginC"] = fw["margin_c"]
        spec["horizonMaxS"] = fw["horizon_s"]
        spec["socReservePct"] = fw["soc_reserve"]
        spec["maxBarKw"] = fw["max_kw"]
        out = run_driver(args.node, args.driver, args.sim, spec)
        samples, truth, meta = parse_driver(out)
        text = replay_text(samples)
        rows, raw = run_replay(args.replay, text)
        if args.work_dir:
            (work / f"{name}.sim.txt").write_text(out)
            (work / f"{name}.replay_in.txt").write_text(text)
            (work / f"{name}.replay_out.csv").write_bytes(raw)
        return name, spec, out, raw, Run(name, spec, samples, truth, meta, rows, fw)

    runs, raw_out = {}, {}
    with ThreadPoolExecutor(max_workers=4) as pool:
        for name, spec, out, raw, run in pool.map(run_one, names):
            runs[name] = run
            raw_out[name] = (spec, out, raw)
            print(f"  {name:16s} {len(run.samples) / 30 / 60:5.1f} min simulated, "
                  f"{len(run.rows)} estimator rows - {spec['desc']}")

    res = Results()
    if "a_flight" in raw_out:
        spec, out, raw = raw_out["a_flight"]
        check_determinism(res, args.node, args.driver, args.sim, args.replay, spec, out, raw)
    check_nothing_before_takeoff(res, runs)
    check_ground_only(res, runs, "d_ground", "ground_only_idle",
                      "ground only, idle + short run-ups: nothing shown")
    check_ground_only(res, runs, "d2_ground_runup", "ground_only_runup",
                      "ground only, 4 min ground run: nothing shown")
    check_level(res, runs)
    check_band(res, runs)
    check_hot(res, runs, "c_hot")
    check_hot(res, runs, "c2_hot_steep")
    check_cool(res, runs)
    check_gaps(res, runs)
    check_relaunch(res, runs)
    if args.emulator:
        do_frames(res, runs, args.emulator, args.frames_dir)

    if not args.work_dir:
        shutil.rmtree(work, ignore_errors=True)
    fails = [c for c in res.items if c["status"] != "pass"]
    for c in res.items:
        print(f"[{c['status'].upper():4s}] {c['id']}: {c['name']}\n         {c['detail']}")
    print(f"\n{len(res.items) - len(fails)}/{len(res.items)} checks passed "
          f"in {time.time() - t_start:.1f} s")
    if args.json:
        args.json.write_text(json.dumps(res.items, indent=1) + "\n")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
