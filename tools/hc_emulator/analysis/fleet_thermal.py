#!/usr/bin/env python3
"""Fit first-order thermal models to real flights.

Per component (motor, ESC MOSFET/capacitor/MCU, battery), per flight:

    dT/dt = (T_ss(P) - T) / tau,   T_ss(P) = T_amb + k * P^2

fitted output-error style (the simulated temperature trace is matched to
the measured one; exact discretisation, so a whole flight is one filter
call). k is the steady-state rise per kW^2, tau the time constant, T_amb the
effective ambient for that flight.

    python fleet_thermal.py --dir thermal/ [--out thermal_fit.json]

Input CSVs: 1 Hz rows with t, p (kW), esc_mos, esc_cap, esc_mcu, motor,
bms_high, bms_mos, bms_bal, baro_temp.
"""

import argparse
import csv
import glob
import json
import os

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

COMPONENTS = {
    # name: (column, warn, crit)  - thresholds from inc/sp140/monitor_config.h
    "motor": ("motor", 105, 115),
    "esc_mos": ("esc_mos", 90, 110),
    "esc_cap": ("esc_cap", 85, 100),
    "esc_mcu": ("esc_mcu", 80, 95),
    "battery": ("bms_high", 50, 56),
    "bms_mos": ("bms_mos", 50, 60),
    "bms_balance": ("bms_bal", 50, 60),
}


def load(path):
    rows = list(csv.DictReader(open(path)))

    def col(c):
        return np.array([float(r[c]) if r.get(c) not in (None, "") else np.nan for r in rows])

    t = col("t")
    t = t - t[0]
    data = {"t": t, "p": col("p"), "baro": col("baro_temp")}
    for name, (c, _, _) in COMPONENTS.items():
        data[name] = col(c)
    return data


def fill(x):
    x = x.copy()
    ok = ~np.isnan(x)
    if ok.sum() < 2:
        return None
    idx = np.arange(len(x))
    return np.interp(idx, idx[ok], x[ok])


def simulate(params, p2, t0):
    k, tau, tamb = params
    alpha = np.exp(-1.0 / tau)
    u = tamb + k * p2
    y, _ = lfilter([1 - alpha], [1, -alpha], u, zi=[alpha * t0])
    return y


def fit(T, P):
    p2 = np.clip(P, 0, None) ** 2
    t0 = T[0]

    def resid(params):
        return simulate(params, p2, t0) - T

    best = None
    for tau0 in (60.0, 200.0, 600.0):
        tamb0 = float(np.clip(np.percentile(T, 5), -19.0, 79.0))
        res = least_squares(resid, [0.3, tau0, tamb0],
                            bounds=([0.0, 5.0, -20.0], [20.0, 5000.0, 80.0]))
        if best is None or res.cost < best.cost:
            best = res
    rmse = float(np.sqrt(np.mean(best.fun ** 2)))
    return best.x, rmse


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", required=True)
    ap.add_argument("--out", default="thermal_fit.json")
    args = ap.parse_args()

    results = {name: [] for name in COMPONENTS}
    for path in sorted(glob.glob(os.path.join(args.dir, "*.csv"))):
        d = load(path)
        P = fill(d["p"])
        if P is None or np.nanmax(P) < 5 or len(P) < 300:
            continue
        baro = np.nanmedian(d["baro"])
        for name in COMPONENTS:
            T = fill(d[name])
            if T is None or np.nanstd(T) < 2:
                continue
            (k, tau, tamb), rmse = fit(T, P)
            results[name].append({"flight": os.path.basename(path)[:19], "k": k, "tau": tau,
                                  "tamb": tamb, "baro": baro, "rmse": rmse,
                                  "tmax": float(np.nanmax(T)), "pmax": float(np.nanmax(P))})

    summary = {}
    for name, rs in results.items():
        if not rs:
            continue
        k = np.array([r["k"] for r in rs])
        tau = np.array([r["tau"] for r in rs])
        rmse = np.array([r["rmse"] for r in rs])
        tamb = np.array([r["tamb"] for r in rs])
        baro = np.array([r["baro"] for r in rs])
        warn, crit = COMPONENTS[name][1:]
        summary[name] = {"n": len(rs), "k_median": float(np.median(k)),
                         "k_p10_p90": [float(np.percentile(k, 10)), float(np.percentile(k, 90))],
                         "tau_median_s": float(np.median(tau)),
                         "tau_p10_p90": [float(np.percentile(tau, 10)), float(np.percentile(tau, 90))],
                         "rmse_median_c": float(np.median(rmse)),
                         "tamb_minus_baro_median": float(np.median(tamb - baro)),
                         "warn": warn, "crit": crit}
        s = summary[name]
        print(f"{name:8} n={s['n']:2d}  k={s['k_median']:.3f} C/kW^2 [{s['k_p10_p90'][0]:.3f},{s['k_p10_p90'][1]:.3f}]"
              f"  tau={s['tau_median_s']:.0f}s [{s['tau_p10_p90'][0]:.0f},{s['tau_p10_p90'][1]:.0f}]"
              f"  fit rmse={s['rmse_median_c']:.1f}C  T_amb-baro={s['tamb_minus_baro_median']:+.1f}C")
        # Sustainable power at steady state from a 30 C ambient
        for amb in (20, 30, 35):
            pw = np.sqrt(max(warn - amb, 0) / s["k_median"])
            pc = np.sqrt(max(crit - amb, 0) / s["k_median"])
            print(f"          ambient {amb} C: holds under warn up to {pw:4.1f} kW, under crit up to {pc:4.1f} kW (steady state)")
    json.dump({"per_flight": results, "summary": summary}, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
