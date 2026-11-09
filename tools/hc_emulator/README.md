# Hand-controller emulator

Runs the real SP140 main screen, built from `src/sp140/lvgl` with the same
`lv_conf.h` and fonts, natively on your computer. A browser dashboard feeds
it telemetry. Use it to work on the display without flashing hardware, and to
evaluate algorithms such as climb efficiency (issue #57) against simulated or
recorded flights.

```
browser dashboard (web/)  ──HTTP──  server.py  ──stdin/stdout──  hc_emulator
  flight sim / log replay                                         real LVGL UI +
  charts vs ground truth                                          ClimbEfficiencyEstimator
```

The emulator runs on a virtual clock. Every `millis()` the firmware sees comes
from the simulation, so flights can run at up to 30× speed and replay
identically.

## Build

`hc_emulator` is a target in the screenshot-test CMake project, so it shares
the LVGL build with the screenshot tests:

```bash
cmake -S test/test_screenshots -B build-screenshot -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-screenshot --target hc_emulator
```

On Windows, use the MSYS2 UCRT64 toolchain (`C:\msys64\ucrt64\bin` on `PATH`)
and a checkout path without spaces (for example the `C:\eppg-ctrl` junction).

## Run

```bash
python tools/hc_emulator/server.py --logs path/to/flight-csvs
```

Then open http://127.0.0.1:8140/. `--logs` is optional and can be repeated.
Every `*.csv` in those folders appears in the replay list, and you can also
pick a file from disk in the page. The server needs only the Python standard
library. Stop the server before rebuilding: Windows locks the running binary.

## Dashboard

- **Screen.** The real 160×128 framebuffer, scaled 3×. You can switch the theme,
  the units, the performance mode and the climb-efficiency display option.
- **Simulator.** A quasi-steady paramotor model: wing trim speed and sink,
  actuator-disk prop thrust, and motor losses that grow with load. It adds
  thermals and sink, gusts, arm movement at the controller, baro noise through
  the BMP390 IIR filter the firmware sets, and 10 Hz BMS power with ripple.
  The scenarios are a manual throttle, a 4 → 20 kW power sweep, a random pilot,
  and climb/glide cycles. Wing trim and brake change the power curve so you can
  watch the display respond.
- **Replay.** Plays CSVs exported by the OpenPPG app, the same format the
  Flight Data dashboard uses. It uses `Controller_Altitude(m)`,
  `BMS_Power(kW)` (falling back to ESC V×I), `Controller_DeviceState`, SOC and
  temperatures. Logs recorded below 30 Hz are interpolated up to the
  firmware's UI rate. Gaps longer than 3 s are passed through as gaps.
- **Charts.** They compare the firmware estimate with the simulator's
  still-air truth: power with the sweet-spot band, climb rate, climb yield and
  best-climb power over time, plus the learned climb-vs-power and
  yield-vs-power curves.
- **Stability panel.** Built from the text the firmware readout actually shows:
  changes per minute, step size, spread, and error against the truth.
- **Estimator tuning.** Sets `ClimbEfficiencyConfig` in the running firmware.

## Tuning against real flights (`analysis/`)

`climb_eff_replay` (same CMake project) runs only the estimator,
`src/sp140/climb_efficiency.cpp`, over a flight log. The scripts in
`analysis/` use it to score the estimator against real flights:

- `climb_baseline.py` screens logs for real flights, builds a ground truth for
  each, and replays the controller-only data through the firmware estimator.
  A real flight is over 5 minutes long, spends at least 4 minutes airborne,
  climbs above 30 m, uses the motor, and has GPS. The ground truth uses both
  barometers plus GPS, and only straight, steady-power flight. From that it
  derives the level-flight power and the climb curve.
- `fleet_tune.py` fits the curvature `q` shared across setups, and grid-tunes
  the level-power settings.

Inputs are OpenPPG app CSV exports or flight archives. A folder of
per-flight CSVs pulled at 10 Hz from `telemetry_points` also works; use the
columns `t` (epoch s), `bms_power`, `altitude`, `controller_baro_pressure`,
`barometer_pressure`, `gps_altitude`, `gps_speed`, `latitude`, `longitude`
and `device_state`. Keep flight data out of this repository.

## What is not emulated

The emulator does not cover alerts or the alert carousel, BLE, buttons,
vibration, the splash screen, or the other tasks in `main.cpp`. `hc_emulator`
mirrors `refreshDisplay()`: it feeds the estimator, then calls
`updateLvglMainScreen()` and `updateClimbEfficiencyDisplay()`. The line
protocol is documented at the top of `hc_emulator.cpp`.
