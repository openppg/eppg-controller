#!/usr/bin/env python3
"""Calibrate the level-power certainty margin on real flights.

For each setting, every flight is replayed through the firmware estimator
and, while the pink tick is shown, we check whether the ground-truth level
power lies inside tick +/- margin. Target: ~90 % coverage with the tightest
margin.

    python margin_calibrate.py --csv-dir fleet/ [--out margin.json]
"""

import argparse
import glob
import itertools
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import climb_baseline as cb  # noqa: E402

RELIABLE_LEVEL_N = 30


def replay(job):
    exe, text, params, truth_level = job
    rows = cb.run_replay(exe, text, params)
    t = np.array([r["t_ms"] for r in rows]) / 60000.0
    t -= t[0]
    shown = np.array([bool(r["level_valid"]) for r in rows])
    lvl = np.array([r["level"] for r in rows])
    mar = np.array([r["level_margin"] for r in rows])
    return t, shown, lvl, mar, truth_level


def summarize(results):
    inside, margins, firsts, errs = [], [], [], []
    at = {5: [], 10: [], 20: []}
    for t, shown, lvl, mar, truth in results:
        if not shown.any():
            continue
        firsts.append(t[shown][0])
        e = np.abs(lvl[shown] - truth)
        inside += list(e <= mar[shown])
        margins += list(mar[shown])
        errs += list(e)
        for m in at:
            k = np.searchsorted(t, t[shown][0] + m)
            if k < len(t) and shown[k]:
                at[m].append(mar[k])
    return {
        "coverage": float(np.mean(inside)),
        "margin_median": float(np.median(margins)),
        "err_median": float(np.median(errs)),
        "first_min_median": float(np.median(firsts)),
        "shown_flights": len(firsts),
        **{f"margin_after_{m}min": float(np.median(v)) if v else None for m, v in at.items()},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv-dir", required=True)
    ap.add_argument("--exe", default=cb.DEFAULT_REPLAY + (".exe" if os.name == "nt" else ""))
    ap.add_argument("--out", default="margin.json")
    args = ap.parse_args()

    flights = []
    for path in sorted(glob.glob(os.path.join(args.csv_dir, "*.csv"))):
        f = cb.load_csv(path)
        if cb.screen(f)[1]:
            continue
        tr = cb.truth(f)
        if np.isfinite(tr.get("level", np.nan)) and tr.get("level_n", 0) >= RELIABLE_LEVEL_N:
            flights.append((cb.replay_input(f), tr["level"]))
    print(f"calibrating on {len(flights)} flights with reliable level truth")

    grid = {"margink": [4, 6, 8, 10], "marginfloor": [0.20, 0.25, 0.30, 0.35]}
    combos = [dict(zip(grid, v)) for v in itertools.product(*grid.values())]
    jobs = [(args.exe, text, c, truth) for c in combos for text, truth in flights]
    with ProcessPoolExecutor() as pool:
        res = list(pool.map(replay, jobs))
    n = len(flights)
    table = []
    for i, c in enumerate(combos):
        s = summarize(res[i * n:(i + 1) * n])
        table.append({**c, **s})
        print("  " + "  ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in table[-1].items()))
    json.dump(table, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
