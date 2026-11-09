#!/usr/bin/env python3
"""Ground truth for climb efficiency from real flight logs, and a score for
the firmware estimator against it.

The firmware only has the hand controller's barometer and pack power. Offline
we can do much better, so each flight gets a "golden" answer from everything
that was logged:

  * vertical speed fused from two independent barometers (controller + phone,
    median of their rates) and cross-checked with GPS altitude,
  * straight flight only (GPS ground-track turn rate), airborne only,
  * steady-power windows only (the pendulum has settled),
  * level-flight power = median power over steady, straight, level windows,
  * the climb curve w(P) = robust quadratic fit over steady windows, and from
    it the best climb power (max w/P) and the band within 95 % of it.

Then the controller-only data (altitude + power at the firmware UI rate) is
replayed through the real estimator (climb_eff_replay, built from
src/sp140/climb_efficiency.cpp) and scored against that truth.

Inputs: OpenPPG app CSV exports / flight archives (full rate) and the Flight
Data dashboard's analysis/flights_1hz.json cache.

    python climb_baseline.py --json flights_1hz.json --csv-dir logs/ --out report/
"""

import argparse
import csv
import datetime as dt
import glob
import gzip
import io
import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_REPLAY = os.path.join(REPO, "build-screenshot", "climb_eff_replay")

# --- what counts as a real, clean flight -----------------------------------
MIN_DURATION_S = 5 * 60        # whole log
MIN_AIRBORNE_S = 4 * 60        # > 15 m above launch and > 4 m/s over ground
MIN_MAX_ALT_M = 30
MIN_PEAK_POWER_KW = 3.0        # the motor was actually used
AIRBORNE_ALT_M = 15
AIRBORNE_GS_MS = 4

# --- truth extraction ------------------------------------------------------
RATE_HALF_WINDOW_S = 3         # +/-3 s linear fit for baro rates
GPS_RATE_HALF_WINDOW_S = 6
STRAIGHT_TURN_DEG_S = 5
STRAIGHT_HOLD_S = 8
WINDOW_S = 10                  # truth windows, same length as the firmware's
SETTLE_S = 3
STEP_S = 5
POWER_CV_MAX = 0.10
BARO_AGREE_MS = 0.8            # max |controller rate - phone rate| per sample
LEVEL_BAND_MS = 0.2
MIN_POWER_KW = 1.5
BAND_FRACTION = 0.95


@dataclass
class Flight:
    name: str
    t: np.ndarray                  # 1 Hz grid, seconds from start
    p: np.ndarray                  # kW
    alt: np.ndarray                # controller relative altitude, m
    cbaro_alt: np.ndarray          # controller pressure altitude, m
    pbaro_alt: np.ndarray          # phone pressure altitude, m
    galt: np.ndarray               # GPS altitude, m
    gs: np.ndarray                 # GPS ground speed, m/s
    lat: np.ndarray
    lon: np.ndarray
    raw_t: np.ndarray = None       # controller-rate samples for the replay
    raw_alt: np.ndarray = None
    raw_p: np.ndarray = None
    raw_armed: np.ndarray = None
    meta: dict = field(default_factory=dict)
    source_rate_hz: float = 1.0


def pressure_altitude(hpa):
    hpa = np.asarray(hpa, float)
    with np.errstate(invalid="ignore", divide="ignore"):
        return 44330.77 * (1.0 - np.power(hpa / 1013.25, 0.190263))


def fill_small_gaps(x, max_gap=3):
    """Linear-interpolate NaN runs up to max_gap samples; leave longer ones."""
    x = np.asarray(x, float).copy()
    n = len(x)
    good = ~np.isnan(x)
    if good.sum() < 2:
        return x
    idx = np.arange(n)
    interp = np.interp(idx, idx[good], x[good])
    i = 0
    while i < n:
        if not good[i]:
            j = i
            while j < n and not good[j]:
                j += 1
            if i > 0 and j < n and (j - i) <= max_gap:
                x[i:j] = interp[i:j]
            i = j
        else:
            i += 1
    return x


def rolling_slope(y, half):
    """Centred least-squares slope per sample on a 1 Hz grid (NaN aware)."""
    n = len(y)
    out = np.full(n, np.nan)
    k = np.arange(-half, half + 1, dtype=float)
    for i in range(half, n - half):
        seg = y[i - half:i + half + 1]
        if np.isnan(seg).any():
            continue
        out[i] = np.dot(k, seg - seg.mean()) / np.dot(k, k)
    return out


def rolling(fn, x, win):
    n = len(x)
    out = np.full(n, np.nan)
    for i in range(win - 1, n):
        seg = x[i - win + 1:i + 1]
        if not np.isnan(seg).any():
            out[i] = fn(seg)
    return out


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def to_grid(t, cols):
    """Median per whole second on a regular grid (the dashboard's denoiser)."""
    t = np.asarray(t, float)
    sec = np.floor(t - t[0]).astype(int)
    n = sec[-1] + 1
    out = {}
    order = np.argsort(sec, kind="stable")
    sec_sorted = sec[order]
    bounds = np.searchsorted(sec_sorted, np.arange(n + 1))
    for name, values in cols.items():
        v = np.asarray(values, float)[order]
        g = np.full(n, np.nan)
        for s in range(n):
            a, b = bounds[s], bounds[s + 1]
            if b > a:
                seg = v[a:b]
                seg = seg[~np.isnan(seg)]
                if len(seg):
                    g[s] = np.median(seg)
        out[name] = fill_small_gaps(g)
    return np.arange(n, dtype=float), out


def load_json_1hz(path):
    data = json.load(open(path))
    cols = data["cols"]
    ix = {c: i for i, c in enumerate(cols)}
    flights = []
    for fid, rows in data["flights"].items():
        a = np.array([[np.nan if v is None else v for v in r] for r in rows], float)
        t = a[:, ix["sec"]]
        grid_t, g = to_grid(t, {
            "p": a[:, ix["p"]], "alt": a[:, ix["alt"]], "cbaro": a[:, ix["cbaro"]],
            "pbaro": a[:, ix["pbaro"]], "galt": a[:, ix["galt"]], "gs": a[:, ix["gs"]],
            "lat": a[:, ix["lat"]], "lon": a[:, ix["lon"]],
        })
        f = Flight(
            name=f"{fid[:8]}", t=grid_t, p=g["p"], alt=g["alt"],
            cbaro_alt=pressure_altitude(g["cbaro"]), pbaro_alt=pressure_altitude(g["pbaro"]),
            galt=g["galt"], gs=g["gs"], lat=g["lat"], lon=g["lon"],
            meta={"session": fid, "start": dt.datetime.fromtimestamp(t[0]).isoformat()},
            source_rate_hz=1.0,
        )
        flights.append(f)
    return flights


def open_text(path):
    raw = open(path, "rb").read()
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    return raw.decode("utf-8", errors="replace")


# Column aliases: app CSV export names first, then archive/DB-style names.
ALIASES = {
    "t": ["Timestamp", "timestamp", "ts", "t"],
    "p": ["BMS_Power(kW)", "bms_power"],
    "alt": ["Controller_Altitude(m)", "altitude", "controller_altitude"],
    "cbaro": ["Controller_BaroPressure(hPa)", "controller_baro_pressure", "baro_pressure"],
    "pbaro": ["Phone_BarometerPressure(hPa)", "barometer_pressure", "phone_barometer_pressure"],
    "galt": ["Phone_GPS_Altitude(m)", "gps_altitude"],
    "gs": ["Phone_GPS_Speed(m/s)", "gps_speed"],
    "lat": ["Phone_GPS_Latitude", "latitude"],
    "lon": ["Phone_GPS_Longitude", "longitude"],
    "state": ["Controller_DeviceState", "device_state"],
    "ev": ["ESC_Voltage(V)", "esc_voltage"],
    "ei": ["ESC_DCCurrent(A)", "esc_current", "esc_dc_current"],
    "escp": ["esc_power"],
}


def parse_time(s):
    try:
        return float(s)
    except ValueError:
        pass
    s = s.strip().replace("Z", "+00:00")
    return dt.datetime.fromisoformat(s).timestamp()


def load_csv(path):
    text = open_text(path)
    lines = text.splitlines()
    header_i = next((i for i, l in enumerate(lines)
                     if l.split(",")[0].strip() in ALIASES["t"]), None)
    if header_i is None:
        raise ValueError("no timestamp header")
    meta = {}
    for l in lines[:header_i]:
        if "," in l:
            k, v = l.split(",", 1)
            meta[k.strip()] = v.strip()
    reader = csv.reader(io.StringIO("\n".join(lines[header_i:])))
    header = next(reader)
    col = {}
    for key, names in ALIASES.items():
        for n in names:
            if n in header:
                col[key] = header.index(n)
                break
    rows = [r for r in reader if len(r) >= len(header) // 2]

    def get(key):
        if key not in col:
            return np.full(len(rows), np.nan)
        i = col[key]
        out = np.empty(len(rows))
        for k, r in enumerate(rows):
            try:
                out[k] = float(r[i]) if r[i] not in ("", "null", "NaN") else np.nan
            except (ValueError, IndexError):
                out[k] = np.nan
        return out

    t = np.array([parse_time(r[col["t"]]) for r in rows])
    order = np.argsort(t, kind="stable")
    t = t[order]
    p = get("p")[order]
    esc_p = get("escp")[order]
    if np.isnan(esc_p).all():
        esc_p = (get("ev") * get("ei") / 1000.0)[order]
    p = np.where(np.isnan(p), esc_p, p)
    alt = get("alt")[order]
    state = get("state")[order]
    cols = {"p": p, "alt": alt, "cbaro": get("cbaro")[order], "pbaro": get("pbaro")[order],
            "galt": get("galt")[order], "gs": get("gs")[order],
            "lat": get("lat")[order], "lon": get("lon")[order]}
    grid_t, g = to_grid(t, cols)
    rate = len(t) / max(1.0, t[-1] - t[0])
    ok = ~np.isnan(alt) & ~np.isnan(p)
    f = Flight(
        name=os.path.splitext(os.path.basename(path))[0][:40], t=grid_t, p=g["p"], alt=g["alt"],
        cbaro_alt=pressure_altitude(g["cbaro"]), pbaro_alt=pressure_altitude(g["pbaro"]),
        galt=g["galt"], gs=g["gs"], lat=g["lat"], lon=g["lon"],
        raw_t=t[ok] - t[0], raw_alt=alt[ok], raw_p=p[ok],
        raw_armed=(np.where(np.isnan(state), 1, state)[ok] > 0),
        meta=meta, source_rate_hz=rate,
    )
    return f


# ---------------------------------------------------------------------------
# Real-flight screening
# ---------------------------------------------------------------------------
def screen(f):
    dur = f.t[-1] - f.t[0]
    airborne = (f.alt > AIRBORNE_ALT_M) & (f.gs > AIRBORNE_GS_MS)
    checks = {
        "duration_min": dur / 60,
        "airborne_min": np.nansum(airborne) / 60,
        "max_alt_m": np.nanmax(f.alt) if np.isfinite(f.alt).any() else np.nan,
        "peak_power_kw": np.nanmax(f.p) if np.isfinite(f.p).any() else np.nan,
        "gps": np.isfinite(f.lat).mean(),
        "phone_baro": np.isfinite(f.pbaro_alt).mean(),
    }
    reasons = []
    if dur < MIN_DURATION_S:
        reasons.append("shorter than 5 min")
    if checks["airborne_min"] * 60 < MIN_AIRBORNE_S:
        reasons.append("under 4 min airborne")
    if not checks["max_alt_m"] >= MIN_MAX_ALT_M:
        reasons.append("never above 30 m")
    if not checks["peak_power_kw"] >= MIN_PEAK_POWER_KW:
        reasons.append("motor barely used")
    if checks["gps"] < 0.5:
        reasons.append("no GPS")
    return checks, reasons


# ---------------------------------------------------------------------------
# Truth
# ---------------------------------------------------------------------------
def heading_rate(f):
    lat = np.radians(f.lat)
    lon = np.radians(f.lon)
    n = len(lat)
    ve = np.full(n, np.nan)
    vn = np.full(n, np.nan)
    b = 2  # 4 s baseline
    r = 6371000.0
    ve[b:n - b] = (lon[2 * b:] - lon[:n - 2 * b]) * r * np.cos(lat[b:n - b]) / (2 * b)
    vn[b:n - b] = (lat[2 * b:] - lat[:n - 2 * b]) * r / (2 * b)
    track = np.degrees(np.arctan2(ve, vn))
    good = ~np.isnan(track)
    unwrapped = np.full(n, np.nan)
    if good.sum() > 2:
        unwrapped[good] = np.degrees(np.unwrap(np.radians(track[good])))
    return rolling_slope(unwrapped, 3)


def fused_climb(f):
    wc = rolling_slope(f.cbaro_alt, RATE_HALF_WINDOW_S)
    if np.isnan(wc).all():  # no raw controller pressure: use its altitude
        wc = rolling_slope(f.alt, RATE_HALF_WINDOW_S)
    wp = rolling_slope(f.pbaro_alt, RATE_HALF_WINDOW_S)
    wg = rolling_slope(f.galt, GPS_RATE_HALF_WINDOW_S)
    both = ~np.isnan(wc) & ~np.isnan(wp)
    w = np.where(both, 0.5 * (wc + wp), np.where(np.isnan(wc), wp, wc))
    agree = np.where(both, np.abs(wc - wp) <= BARO_AGREE_MS, True)
    return w, wc, wp, wg, agree


def huber_quadratic(p, w, weights=None):
    if weights is None:
        weights = np.ones_like(p)
    sw = np.sqrt(weights)

    def resid(c):
        return sw * (c[0] + c[1] * p + c[2] * p * p - w)

    c0 = np.polyfit(p, w, 2)[::-1] if len(p) > 3 else np.array([-1.0, 0.3, 0.0])
    res = least_squares(resid, c0, loss="huber", f_scale=0.3)
    return res.x


def curve_optimum(c, lo, hi, fraction=BAND_FRACTION):
    grid = np.linspace(lo, hi, 400)
    y = (c[0] + c[1] * grid + c[2] * grid * grid) / grid
    k = int(np.argmax(y))
    ok = y >= fraction * y[k]
    a = k
    while a > 0 and ok[a - 1]:
        a -= 1
    b = k
    while b < len(grid) - 1 and ok[b + 1]:
        b += 1
    return {
        "best": grid[k], "best_yield": y[k], "band_lo": grid[a], "band_hi": grid[b],
        "at_edge": k in (0, len(grid) - 1),
    }


def truth(f):
    w, wc, wp, wg, agree = fused_climb(f)
    turn = heading_rate(f)
    straight_now = np.abs(turn) < STRAIGHT_TURN_DEG_S
    straight = rolling(lambda s: float(s.min()), straight_now.astype(float),
                       STRAIGHT_HOLD_S) > 0.5
    airborne = (f.alt > AIRBORNE_ALT_M) & (f.gs > AIRBORNE_GS_MS)
    cv = rolling(lambda s: s.std() / s.mean() if s.mean() > 0.05 else 0.0,
                 f.p, WINDOW_S + SETTLE_S)
    pstd = rolling(np.std, f.p, WINDOW_S + SETTLE_S)
    windows = []
    for end in range(WINDOW_S + SETTLE_S, len(f.t), STEP_S):
        sl = slice(end - WINDOW_S + 1, end + 1)
        if not (airborne[sl].all() and straight[sl].all() and agree[sl].all()):
            continue
        if np.isnan(w[sl]).any() or np.isnan(f.p[sl]).any():
            continue
        steady = cv[end] <= POWER_CV_MAX or pstd[end] <= 0.1
        if not steady:
            continue
        windows.append((f.t[end], float(np.mean(f.p[sl])), float(np.mean(w[sl])),
                        float(np.nanmean(wg[sl]))))
    W = np.array(windows) if windows else np.zeros((0, 4))
    out = {"windows": W, "n_windows": len(W),
           "baro_rms_diff": float(np.nanstd((wc - wp)[airborne])) if airborne.any() else np.nan,
           "gps_vs_baro_bias": float(np.nanmedian((wg - w)[airborne])) if airborne.any() else np.nan}
    if len(W) == 0:
        return out
    P, Wc = W[:, 1], W[:, 2]
    level = (np.abs(Wc) < LEVEL_BAND_MS) & (P >= MIN_POWER_KW)
    out["level_n"] = int(level.sum())
    out["level_direct"] = float(np.median(P[level])) if level.sum() >= 6 else np.nan
    near = (np.abs(Wc) < 1.5) & (P >= MIN_POWER_KW)
    if near.sum() >= 10 and np.ptp(P[near]) > 0.5:
        slope, icpt = np.polyfit(P[near], Wc[near], 1)
        out["level_fit"] = float(-icpt / slope) if slope > 0.05 else np.nan
        out["slope_near_level"] = float(slope)
    powered = P >= MIN_POWER_KW
    out["p_lo"] = float(np.percentile(P[powered], 5)) if powered.sum() else np.nan
    out["p_hi"] = float(np.percentile(P[powered], 95)) if powered.sum() else np.nan
    glide = P < 0.3
    out["glide_n"] = int(glide.sum())
    out["glide_sink"] = float(-np.median(Wc[glide])) if glide.sum() >= 3 else np.nan
    if powered.sum() >= 12 and out["p_hi"] - out["p_lo"] > 2.0:
        c = huber_quadratic(P, Wc)
        out["curve"] = c.tolist()
        out.update(curve_optimum(c, out["p_lo"], out["p_hi"]))
    # Golden level power: direct median when there is plenty of level flight
    out["level"] = out["level_direct"] if out.get("level_n", 0) >= 12 else out.get("level_fit", np.nan)
    return out


def pooled_truth(flights_truth):
    W = np.vstack([t["windows"] for t in flights_truth if t["n_windows"]])
    P, Wc = W[:, 1], W[:, 2]
    c = huber_quadratic(P, Wc)
    powered = P >= MIN_POWER_KW
    lo, hi = np.percentile(P[powered], 2), np.percentile(P[powered], 98)
    res = {"curve": c.tolist(), "p_lo": float(lo), "p_hi": float(hi), "n": len(W)}
    res.update(curve_optimum(c, lo, hi))
    res["level_root"] = float(np.roots([c[2], c[1], c[0]]).real[
        np.argmin(np.abs(np.roots([c[2], c[1], c[0]]).real - 3.5))])
    return res


# ---------------------------------------------------------------------------
# Firmware replay + scoring
# ---------------------------------------------------------------------------
def replay_input(f, noise=None, seed=1):
    """Controller-rate (30 Hz) samples: altitude + power as the UI task sees them."""
    if f.raw_t is not None and f.source_rate_hz >= 5:
        t_src, a_src, p_src = f.raw_t, f.raw_alt, f.raw_p
        armed_src = f.raw_armed
    else:
        ok = ~np.isnan(f.alt) & ~np.isnan(f.p)
        t_src, a_src, p_src = f.t[ok], f.alt[ok], f.p[ok]
        armed_src = np.ones(ok.sum(), bool)
    tt = np.arange(t_src[0], t_src[-1], 1 / 30.0)
    # Never interpolate across a logging gap: the firmware would see a gap
    # (and restart its window), not a slow straight-line change.
    nxt = np.clip(np.searchsorted(t_src, tt, side="right"), 1, len(t_src) - 1)
    keep = (t_src[nxt] - t_src[nxt - 1]) <= 3.0
    tt = tt[keep]
    alt = np.interp(tt, t_src, a_src)
    p = np.interp(tt, t_src, p_src)
    armed = np.interp(tt, t_src, armed_src.astype(float)) > 0.5
    if noise:
        # The 1 Hz cache is median-filtered; put back the controller's arm
        # movement (OU, 2 s) and baro noise so the estimator sees a hand.
        rng = np.random.default_rng(seed)
        dt_ = 1 / 30.0
        hand = np.zeros(len(tt))
        for i in range(1, len(tt)):
            hand[i] = hand[i - 1] * (1 - dt_ / 2.0) + noise["hand"] * math.sqrt(dt_) * rng.standard_normal()
        alt = alt + hand + noise["baro"] * rng.standard_normal(len(tt))
    ms = np.round((tt - tt[0]) * 1000).astype(np.int64) + 1000
    return "\n".join(f"{m} {a:.3f} {q:.3f} {int(b)}" for m, a, q, b in zip(ms, alt, p, armed))


def run_replay(exe, text, params):
    args = [exe, "every=5000"] + [f"{k}={v}" for k, v in params.items()]
    out = subprocess.run(args, input=text.encode(), capture_output=True, check=True).stdout.decode()
    lines = out.strip().splitlines()
    hdr = lines[0].split(",")
    rows = []
    for l in lines[1:]:
        vals = l.split(",")
        rows.append({h: (float(v) if v not in ("",) else np.nan) for h, v in zip(hdr, vals)})
    return rows


def score(rows, tr):
    t = np.array([r["t_ms"] for r in rows]) / 1000.0
    t = t - t[0]
    lvl = np.array([r["level"] if r["level_valid"] else np.nan for r in rows])
    best = np.array([r["best"] if r["curve_valid"] else np.nan for r in rows])
    lo = np.array([r["band_lo"] if r["curve_valid"] else np.nan for r in rows])
    hi = np.array([r["band_hi"] if r["curve_valid"] else np.nan for r in rows])
    s = {}
    first_level = np.argmax(~np.isnan(lvl)) if (~np.isnan(lvl)).any() else None
    s["level_first_min"] = t[first_level] / 60 if first_level is not None else np.nan
    if np.isfinite(tr.get("level", np.nan)):
        err = lvl - tr["level"]
        valid = ~np.isnan(err)
        s["level_err_final"] = float(err[valid][-1]) if valid.any() else np.nan
        s["level_mae"] = float(np.mean(np.abs(err[valid]))) if valid.any() else np.nan
    first_band = np.argmax(~np.isnan(best)) if (~np.isnan(best)).any() else None
    s["band_first_min"] = t[first_band] / 60 if first_band is not None else np.nan
    if "best" in tr:
        valid = ~np.isnan(best)
        if valid.any():
            s["best_err_final"] = float(best[valid][-1] - tr["best"])
            s["best_mae"] = float(np.mean(np.abs(best[valid] - tr["best"])))
            a0, a1 = lo[valid][-1], hi[valid][-1]
            b0, b1 = tr["band_lo"], tr["band_hi"]
            inter = max(0.0, min(a1, b1) - max(a0, b0))
            union = max(a1, b1) - min(a0, b0)
            s["band_iou_final"] = inter / union if union > 0 else np.nan
    s["final"] = {"level": float(lvl[~np.isnan(lvl)][-1]) if (~np.isnan(lvl)).any() else None,
                  "best": float(best[~np.isnan(best)][-1]) if (~np.isnan(best)).any() else None,
                  "band": [float(lo[~np.isnan(lo)][-1]), float(hi[~np.isnan(hi)][-1])]
                  if (~np.isnan(lo)).any() else None}
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--json", action="append", default=[], help="dashboard flights_1hz.json")
    ap.add_argument("--csv-dir", action="append", default=[], help="folder of app CSV / .csv.gz logs")
    ap.add_argument("--exe", default=DEFAULT_REPLAY + (".exe" if os.name == "nt" else ""))
    ap.add_argument("--noise", action="store_true", help="add controller hand/baro noise to 1 Hz data")
    ap.add_argument("--param", action="append", default=[], help="estimator override key=value")
    ap.add_argument("--out", default="climb_baseline_out")
    args = ap.parse_args()

    flights = []
    for j in args.json:
        flights += load_json_1hz(j)
    for d in args.csv_dir:
        for path in sorted(glob.glob(os.path.join(d, "*.csv")) + glob.glob(os.path.join(d, "*.csv.gz"))):
            try:
                flights.append(load_csv(path))
            except Exception as exc:  # noqa: BLE001 - report and keep going
                print(f"skip {os.path.basename(path)}: {exc}", file=sys.stderr)

    os.makedirs(args.out, exist_ok=True)
    params = dict(kv.split("=", 1) for kv in args.param)
    noise = {"hand": 0.3, "baro": 0.15} if args.noise else None
    report = []
    kept = []
    for f in flights:
        checks, reasons = screen(f)
        entry = {"name": f.name, "rate_hz": round(f.source_rate_hz, 1), **{k: round(v, 2) for k, v in checks.items()}}
        if reasons:
            entry["rejected"] = reasons
            report.append(entry)
            continue
        tr = truth(f)
        entry["truth"] = {k: (round(v, 3) if isinstance(v, float) else v)
                          for k, v in tr.items() if k != "windows"}
        rows = run_replay(args.exe, replay_input(f, noise), params)
        entry["firmware"] = score(rows, tr)
        report.append(entry)
        kept.append((f, tr))

    if kept:
        pooled = pooled_truth([tr for _, tr in kept])
    else:
        pooled = None
    json.dump({"flights": report, "pooled": pooled, "params": params, "noise": noise},
              open(os.path.join(args.out, "report.json"), "w"), indent=1, default=float)
    print(json.dumps({"flights": report, "pooled": pooled}, indent=1, default=float))


if __name__ == "__main__":
    main()
