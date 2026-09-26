#!/usr/bin/env python3
"""Fit the fleet curvature q and tune the firmware level-power estimator on
real flights.

    python fleet_tune.py --csv-dir fleet/ --out tune/

1. Ground truth per flight (climb_baseline.truth): level-flight power from
   steady, straight, level windows with two-baro vertical speed.
2. Shared-q fit: w = s_g (P - L_g) (1 - q (P - L_g)) over every steady window,
   one (L_g, s_g) per setup (wing + prop) and ONE q for the fleet, bootstrapped
   by flight.
3. Replays each flight's controller altitude + power through the real
   estimator (climb_eff_replay) for a grid of level settings and scores the
   pink tick against the truth: time-averaged error while shown, final error,
   and how long it takes to appear.
"""

import argparse
import collections
import glob
import itertools
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import climb_baseline as cb  # noqa: E402

RELIABLE_LEVEL_N = 30   # truth windows needed to trust a flight's level power


def setup_key(meta):
    wing = f"{meta.get('paraglider_manufacturer')} {meta.get('paraglider_wing_model')} {meta.get('wing_size')}"
    return f"{wing} / {meta.get('propeller_blade_count')}b"


def shared_q_fit(groups):
    """groups: {setup: [(P, W) arrays per flight]} -> q and per-setup (L, s)."""
    keys = list(groups)
    data = []
    for gi, k in enumerate(keys):
        for P, W in groups[k]:
            m = P >= 1.5
            data.append((gi, P[m], W[m]))

    def resid(theta):
        q = theta[0]
        out = []
        for gi, P, W in data:
            L, s = theta[1 + 2 * gi], theta[2 + 2 * gi]
            x = P - L
            out.append(s * x * (1 - q * x) - W)
        return np.concatenate(out)

    theta0 = [0.03] + [v for _ in keys for v in (3.5, 0.33)]
    res = least_squares(resid, theta0, loss="huber", f_scale=0.3)
    per = {k: {"L": res.x[1 + 2 * i], "s": res.x[2 + 2 * i]} for i, k in enumerate(keys)}
    return res.x[0], per


def score_level(rows, truth_level):
    t = np.array([r["t_ms"] for r in rows]) / 60000.0
    t -= t[0]
    lvl = np.array([r["level"] if r["level_valid"] else np.nan for r in rows])
    ok = ~np.isnan(lvl)
    if not ok.any():
        return {"first_min": np.nan, "mae": np.nan, "final": np.nan}
    err = lvl[ok] - truth_level
    return {"first_min": float(t[ok][0]), "mae": float(np.mean(np.abs(err))),
            "final": float(err[-1])}


def run_one(args):
    exe, text, params, truth_level = args
    rows = cb.run_replay(exe, text, params)
    return score_level(rows, truth_level)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv-dir", required=True)
    ap.add_argument("--exe", default=cb.DEFAULT_REPLAY + (".exe" if os.name == "nt" else ""))
    ap.add_argument("--out", default="fleet_tune_out")
    ap.add_argument("--boot", type=int, default=200)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    flights = []
    for path in sorted(glob.glob(os.path.join(args.csv_dir, "*.csv"))):
        f = cb.load_csv(path)
        checks, reasons = cb.screen(f)
        if reasons:
            print(f"skip {f.name}: {reasons}")
            continue
        tr = cb.truth(f)
        flights.append((f, tr))
    print(f"{len(flights)} flights")

    # ---- shared curvature -------------------------------------------------
    groups = collections.defaultdict(list)
    for f, tr in flights:
        if tr["n_windows"]:
            W = tr["windows"]
            groups[setup_key(f.meta)].append((W[:, 1], W[:, 2], f.name))
    usable = {k: [(P, W) for P, W, _ in v] for k, v in groups.items()
              if sum(len(P) for P, _, _ in v) >= 60}
    q, per = shared_q_fit(usable)
    rng = np.random.default_rng(1)
    qs = []
    flat = [(k, i) for k, v in usable.items() for i in range(len(v))]
    for _ in range(args.boot):
        pick = [flat[j] for j in rng.integers(0, len(flat), len(flat))]
        g = collections.defaultdict(list)
        for k, i in pick:
            g[k].append(usable[k][i])
        try:
            qb, _ = shared_q_fit(g)
            qs.append(qb)
        except Exception:  # noqa: BLE001
            pass
    q_lo, q_med, q_hi = np.percentile(qs, [10, 50, 90])
    print(f"shared q = {q:.4f}  (bootstrap by flight: median {q_med:.4f}, 80% [{q_lo:.4f}, {q_hi:.4f}])")
    for k, v in per.items():
        pstar = np.sqrt(v["L"] ** 2 + v["L"] / q)
        print(f"  {k:40} L={v['L']:.2f} s={v['s']:.3f}  P*={pstar:.1f} kW")

    # ---- level-power tuning ------------------------------------------------
    scored = [(f, tr) for f, tr in flights
              if np.isfinite(tr.get("level", np.nan)) and tr.get("level_n", 0) >= RELIABLE_LEVEL_N]
    print(f"\nlevel tuning on {len(scored)} flights with >= {RELIABLE_LEVEL_N} truth level windows")
    inputs = [(f, tr, cb.replay_input(f)) for f, tr in scored]
    grid = {
        "levelband": [0.3, 0.5, 0.8],
        "levelhalflife": [60, 120, 300, 900],
        "minlevelw": [10, 20, 40],
        "priorslope": [0.30, 0.34, 0.40],
    }
    combos = [dict(zip(grid, v)) for v in itertools.product(*grid.values())]
    jobs = [(args.exe, text, {**c, "q": round(q, 4)}, tr["level"]) for c in combos for _, tr, text in inputs]
    with ProcessPoolExecutor() as pool:
        results = list(pool.map(run_one, jobs, chunksize=4))
    table = []
    n = len(inputs)
    for ci, c in enumerate(combos):
        rs = results[ci * n:(ci + 1) * n]
        mae = np.nanmean([r["mae"] for r in rs])
        final = np.nanmean([abs(r["final"]) for r in rs])
        first = np.nanmedian([r["first_min"] for r in rs])
        worst = np.nanmax([abs(r["final"]) for r in rs])
        shown = np.mean([np.isfinite(r["mae"]) for r in rs])
        table.append({**c, "mae": mae, "final_abs": final, "worst_final": worst,
                      "first_min_median": first, "shown_frac": shown})
    table.sort(key=lambda r: r["mae"] + 0.02 * r["first_min_median"])
    print("best level settings (mean |error| while shown, kW):")
    for r in table[:8]:
        print("  " + "  ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in r.items()))
    default = next(r for r in table if r["levelband"] == 0.5 and r["levelhalflife"] == 120
                   and r["minlevelw"] == 20 and abs(r["priorslope"] - 0.34) < 1e-6)
    print("current defaults:\n  " + "  ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                                          for k, v in default.items()))

    json.dump({"q": q, "q_boot": [q_lo, q_med, q_hi], "setups": per, "level_grid": table},
              open(os.path.join(args.out, "tune.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
