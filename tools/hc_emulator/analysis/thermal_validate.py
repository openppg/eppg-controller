#!/usr/bin/env python3
"""Validate the firmware thermal-headroom model on real flights.

Each flight is replayed through ThermalHeadroom (climb_eff_replay) with the
motor/ESC priors (k, tau) set to the median of the OTHER flights' fits
(leave-one-out; the firmware then learns k and T_base live), then:

1. Forecast skill - from every 30 s, the live T_base and k plus the model
   predict each part's temperature 1, 3 and 5 min ahead using the power
   actually flown; compared with what was measured.
2. Early warning - for every flight where a part crossed its warning
   threshold, how long before the crossing the pilot's power was already
   above the live warning limit (i.e. in the yellow/red zone).
3. What the live limits look like across the fleet.

    python thermal_validate.py --dir thermal/ --fits thermal_fit.json
"""

import argparse
import csv
import glob
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import climb_baseline as cb  # noqa: E402
from fleet_thermal import COMPONENTS  # noqa: E402

PARTS = ["motor", "esc_mos", "esc_cap", "esc_mcu", "battery", "bms_mos", "bms_balance"]
KEYS = {"motor": "motor", "esc_mos": "mos", "esc_cap": "cap", "esc_mcu": "mcu", "battery": "batt",
        "bms_mos": "bmsmos", "bms_balance": "bmsbal"}
BASE_COL = {"motor": "base_motor", "esc_mos": "base_mos", "esc_cap": "base_cap", "esc_mcu": "base_mcu"}


def load(path):
    rows = list(csv.DictReader(open(path)))

    def col(c):
        return np.array([float(r[c]) if r.get(c) not in (None, "") else np.nan for r in rows])
    d = {c: col(c) for c in ["t", "p", "alt", "soc", "state"]}
    d["t"] = d["t"] - d["t"][0]
    for name, (c, _, _) in COMPONENTS.items():
        d[name] = col(c)
    return d


def replay(exe, d, params):
    lines = []
    for i in range(len(d["t"])):
        f = lambda v: "nan" if not np.isfinite(v) else f"{v:.2f}"  # noqa: E731
        temps = " ".join(f(d[p][i]) for p in PARTS)
        lines.append(f"{int(d['t'][i] * 1000) + 1000} {f(d['alt'][i])} {f(max(d['p'][i], 0) if np.isfinite(d['p'][i]) else np.nan)} 1 "
                     f"{f(d['soc'][i])} {temps}")
    args = [exe, "every=10000"] + [f"{k}={v}" for k, v in params.items()]
    out = subprocess.run(args, input="\n".join(lines).encode(), capture_output=True, check=True).stdout.decode()
    rows = out.strip().splitlines()
    hdr = rows[0].split(",")
    return [{h: (float(v) if v != "" else np.nan) for h, v in zip(hdr, r.split(","))} for r in rows[1:]]


def forecast(T0, base, k, tau, p):
    """Step the first-order model 1 s at a time through the power trace."""
    a = np.exp(-1.0 / tau)
    T = T0
    for pk in p:
        T = a * T + (1 - a) * (base + k * pk * pk)
    return T


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", required=True)
    ap.add_argument("--fits", required=True)
    ap.add_argument("--exe", default=cb.DEFAULT_REPLAY + (".exe" if os.name == "nt" else ""))
    args = ap.parse_args()
    fits = json.load(open(args.fits))["per_flight"]

    horizons = [60, 180, 300]
    errs = {p: {h: [] for h in horizons} for p in BASE_COL}
    warnings = []
    limits = []
    firsts = []
    for path in sorted(glob.glob(os.path.join(args.dir, "*.csv"))):
        name = os.path.basename(path)[:19]
        d = load(path)
        if np.nanmax(np.nan_to_num(d["p"])) < 5 or len(d["t"]) < 300:
            continue
        p = np.nan_to_num(np.clip(d["p"], 0, None))
        params = {}
        loo = {}
        for part in BASE_COL:  # battery: the firmware's pack-type model
            others = [f for f in fits.get(part, []) if f["flight"] != name]
            if not others:
                continue
            k = float(np.median([f["k"] for f in others]))
            tau = float(np.median([f["tau"] for f in others]))
            loo[part] = (k, tau)
            params[f"k_{KEYS[part]}"] = round(k, 4)
            params[f"tau_{KEYS[part]}"] = round(tau, 1)
        rows = replay(args.exe, d, params)
        rt = np.array([r["t_ms"] for r in rows]) / 1000.0 - 1.0
        warn = np.array([r["warn_kw"] for r in rows])
        shown = np.array([r["th_valid"] for r in rows]) > 0
        if shown.any():
            firsts.append(rt[shown][0] / 60)
        crit = np.array([r["crit_kw"] for r in rows])
        who = np.array([r["warn_part"] for r in rows])

        # 1. forecast skill
        for part, bc in BASE_COL.items():
            if part not in loo:
                continue
            _, tau = loo[part]
            T = d[part]
            base = np.array([r[bc] for r in rows])
            gain = np.array([r["gain_" + bc[5:]] for r in rows])  # live k
            for j in range(0, len(rt), 3):
                t0 = int(rt[j])
                if not np.isfinite(base[j]) or t0 >= len(T) or not np.isfinite(T[t0]) or rt[j] < 120:
                    continue
                for h in horizons:
                    if t0 + h < len(T) and np.isfinite(T[t0 + h]):
                        pred = forecast(T[t0], base[j], gain[j], tau, p[t0:t0 + h])
                        errs[part][h].append(pred - T[t0 + h])

        # 2. early warning
        for part in PARTS:
            thr = COMPONENTS[part][1]
            T = d[part]
            over = np.where(np.nan_to_num(T, nan=-99) >= thr)[0]
            if len(over) == 0:
                continue
            tc = over[0]
            lead = None
            for j in range(len(rt)):
                if rt[j] > tc:
                    break
                i0 = int(rt[j])
                if i0 < len(p) and p[i0] > warn[j] and lead is None:
                    lead = tc - rt[j]
                elif i0 < len(p) and p[i0] <= warn[j]:
                    lead = None  # must stay above the limit up to the crossing
            warnings.append((name, part, float(T[tc]), float(tc / 60), lead))

        # 3. live limits while airborne
        air = [j for j in range(len(rt)) if int(rt[j]) < len(d["alt"]) and np.nan_to_num(d["alt"][int(rt[j])]) > 15]
        if air:
            limits.append((name, float(np.median(warn[air])), float(np.min(warn[air])),
                           float(np.median(crit[air])), int(np.bincount(who[air].astype(int) + 1).argmax()) - 1))

    print("1. Forecast error, predicted - measured (C), leave-one-out k/tau, live baseline:")
    for part in BASE_COL:
        cells = []
        for h in horizons:
            e = np.array(errs[part][h])
            if len(e):
                cells.append(f"{h // 60} min: bias {np.mean(e):+.1f}, MAE {np.mean(np.abs(e)):.1f}, P90 {np.percentile(np.abs(e), 90):.1f}")
        print(f"   {part:8} " + " | ".join(cells))
    print("\n2. Flights that crossed a warning threshold:")
    for name, part, temp, tmin, lead in warnings:
        lead_s = f"power was above the live warn limit for {lead:.0f} s before" if lead is not None else "NOT flagged before crossing"
        print(f"   {name} {part:8} reached {temp:.0f} C at {tmin:.1f} min - {lead_s}")
    if firsts:
        print(f"\n4. Zones first appear {np.median(firsts):.1f} min into the flight (median; "
              f"range {np.min(firsts):.1f}-{np.max(firsts):.1f}, {len(firsts)} flights)")
    print("\n3. Live limits while airborne (median / minimum of the warn limit, median crit; limiting part):")
    names = ["none"] + PARTS
    for name, wmed, wmin, cmed, who in limits:
        print(f"   {name} warn {wmed:5.1f} kW (min {wmin:5.1f})  crit {cmed:5.1f} kW  limited by {names[who + 1]}")


if __name__ == "__main__":
    main()
