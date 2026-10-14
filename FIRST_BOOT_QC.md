# First-Boot Hardware QC + Throttle Calibration

Design doc for a per-unit hardware self-test and throttle calibration that runs on the
first flash/boot of each controller. Target: **post-8.0** (built on branch `first-boot-qc`).

## Goals

1. **Catch bad hardware before it ships.** Every unit verifies its own peripherals on
   first boot and shows a per-component checklist on screen.
2. **Calibrate each throttle to its own pot.** Capture this unit's real ADC endpoints so
   every controller gets the full throttle range and a consistent feel — instead of the
   fixed `0..4095` assumption that silently under/over-ranges individual units.
3. **Production traceability.** Emit a structured per-unit QC record over USB serial so a
   bench rig can log results across a whole run and flag outliers.
4. **Be invisible after it passes.** Once a unit passes, a flag in NVS makes it boot
   straight to normal operation. Re-runnable on demand for field service / returns.

Non-goal: replacing bench/HIL testing. This is automated first-line QC + calibration.

## Why per-unit calibration matters (variability sources)

The throttle is read as a 12-bit ADC value (`0..4095`). Today the code maps a fixed
`POT_MIN_VALUE=0 .. POT_MAX_VALUE=4095` to `ESC_MIN_PWM..ESC_MAX_PWM`. Real units vary:

| Source | Effect on raw ADC | Consequence with fixed 0..4095 |
|---|---|---|
| Hall/resistive zero offset, 3V3 rail | released ≈ 80–250, not 0 | small; absorbed by the 5% deadband — **unless** offset drifts above engagement → phantom throttle / arming blocked |
| Mechanical end-stop + sensor span | full press ≈ 3850–4095, not always 4095 | unit **never reaches ESC_MAX_PWM** → loses top-end power |
| Spring/lever slop, plastic tolerance | return-to-rest wanders | inconsistent deadband feel unit-to-unit |
| Temperature / supply drift, aging | endpoints move over time | calibration must keep a safety margin, and be re-runnable |

Per-unit calibration captures this unit's `raw_min` (released) and `raw_max` (full press)
and maps **`[raw_min', raw_max'] → [ESC_MIN_PWM, ESC_MAX_PWM]`**, where the primed values
include deadband margins. Result: full range on every unit, consistent feel, reliable idle.

## Trigger & gating

- **First boot:** if NVS key `qc_passed` is absent or `false`, enter the QC flow
  automatically. (Mirrors the existing first-boot detection in `refreshDeviceData()`.)
- **Manual re-run:** hold the button at boot → force QC/recalibration (field service,
  returns, pot drift after years).
- **After pass:** write `qc_passed=true` + `qc_fw=<version>`; subsequent boots skip QC.
- **FW bump policy (optional):** if `qc_fw` major < current major, re-run the *automatic*
  checks but keep existing calibration.

## Automatic checks (no operator — mostly aggregates existing signals)

Collected ~2–3 s after boot, reusing flags/state the firmware already maintains:

| Check | Signal that already exists | Pass criteria |
|---|---|---|
| Display | reached render path | implicit (you see the screen) |
| Barometer (I2C) | `bmpPresent` + reading | present, pressure in 800–1100 hPa |
| CPU temp | `getCachedCpuTemperature()` | reading in -20..90 °C |
| ESC / CAN (TWAI) | `escTwaiInitialized` + `escTelemetryData.escState` | driver up + `CONNECTED` + telemetry seen |
| BMS / CAN | `bmsCanInitialized` + `bmsTelemetryData.bmsState` | up + `CONNECTED` + pack voltage sane |
| NVS / settings | `preferences.begin()` + read-back | write+read round-trips |
| Throttle ADC | `readThrottleRaw()` | reads, and idle within expected band |

## Interactive checks (operator-confirmed — no electrical readback)

Output-only / input devices need a human in the loop:

- **Throttle calibration** (the important one) — full-range sweep, see below.
- **Button** — "press the button" → detect press.
- **Buzzer** — play a tone → operator confirms audible.
- **Vibration** — pulse → operator confirms felt.
- **NeoPixel** — cycle R/G/B → operator confirms colors.

## Throttle calibration procedure

Guided on-screen, with live raw value shown:

1. **"Release throttle fully"** → sample until stable (variance < ε over ~500 ms) →
   capture `raw_min` (median of the window).
2. **"Squeeze throttle fully"** → sample until stable → capture `raw_max`.
3. **"Release again"** → confirm it returns within tolerance of `raw_min` (hysteresis /
   stuck-lever check).
4. **Sanity-check** (reject → FAIL, do not save, fall back to defaults):
   - `span = raw_max − raw_min ≥ MIN_SPAN` (e.g. 2000) — else bad pot/wiring.
   - `raw_min ≤ MAX_IDLE` (e.g. 800) — else miswired/stuck-high.
   - `raw_max ≥ MIN_FULL` (e.g. 3200) — else never reaches full.
5. **Save** `pot_min=raw_min`, `pot_max=raw_max`, `pot_calibrated=true` to NVS.

### Mapping change (the safety-critical part — lands last, see phasing)

Centralized in `throttle.cpp` (`potRawToPwm`, `potRawToModePwm`) and the engagement/cruise
helpers. Replace fixed endpoints with calibrated effective endpoints:

```
bottom_db  = max(FLOOR_DB, BOTTOM_PCT * span)   // keep a small deadband (drift/slop)
top_margin = TOP_PCT * span                     // ensure full press hits ESC_MAX_PWM
eff_min    = pot_min + bottom_db
eff_max    = pot_max - top_margin
pwm        = map(constrain(raw, eff_min, eff_max), eff_min, eff_max, ESC_MIN_PWM, mode_max)
engagement = eff_min + ENGAGE_PCT * (eff_max - eff_min)   // was 5% of 4095
```

Suggested starting values (tune on hardware): `BOTTOM_PCT≈3%`, `TOP_PCT≈2%`,
`FLOOR_DB≈50 counts`, `ENGAGE_PCT≈5%`. You keep "a slight deadband" via `bottom_db`.

### Safety analysis

- **Uncalibrated = today's behavior.** If `pot_calibrated` is false/absent or values fail
  sanitize, fall back to `0..4095` + existing 5% deadband. No regression for existing units.
- **Validate on every load**, not just at capture — extend `sanitizeDeviceData()` so a
  corrupted `pot_min/max` can never produce a non-idle command at rest.
- **Idle always maps to ESC_MIN_PWM**; output always `constrain`ed to `[ESC_MIN_PWM, mode_max]`.
- **Arming gate** (`throttleSafe`) must use the calibrated zero so "throttle released" is
  honored; a bad calibration that read idle as engaged would *block* arming (fail-safe).
- **Re-cal is deliberate only** (button-hold at boot) — never automatic mid-use.
- Cache `pot_min/max` into `throttle.cpp` statics at init/disarm to avoid per-tick reads of
  `deviceData` from the 50 Hz loop (sidesteps the known settings-concurrency concern).

## On-screen UX

Dedicated LVGL QC screen: vertical list, one row per component = label + live value +
status icon (spinner → green ✓ / red ✗) updating as each check completes. Throttle row
expands into the release/squeeze sub-flow with live raw + captured min/max. Final banner:
**QC PASSED ✓** (green) or **FAILED ✗** listing failed checks. (Could later get
screenshot-test coverage via the existing emulator harness.)

## Production QC record (traceability)

On completion, emit one structured JSON line over USB serial for a bench rig to log:
`{ fw, esc_hw_id, esc_serial, bms_id, pot_min, pot_max, span, baro_hpa, cpu_c,
pack_v, checks:{...}, result }`. Across a run this surfaces outliers (e.g. a batch of pots
with low span) — directly the "variability between controllers" visibility you want.

## NVS schema additions (per-key, same pattern as existing settings)

| Key | Type | Default | Meaning |
|---|---|---|---|
| `qc_passed` | uchar | 0 | overall QC pass flag (gates auto-run) |
| `qc_fw` | ushort | 0 | firmware version that last passed QC |
| `pot_calibrated` | uchar | 0 | throttle calibration valid |
| `pot_min` | ushort | 0 | calibrated raw min (released) |
| `pot_max` | ushort | 4095 | calibrated raw max (full press) |

Add `pot_min`/`pot_max`/`pot_calibrated`/`qc_passed` to `STR_DEVICE_DATA_140_V1`; load in
`refreshDeviceData()`, persist in `writeDeviceData()`, validate in `sanitizeDeviceData()`.

## Phased implementation (risk increases down the list)

1. **Scaffold + gating + automatic POST + serial report.** NVS keys, `qc_passed` gate,
   aggregate init flags + liveness, print report. No throttle change. *Low risk.*
2. **On-screen QC checklist UI** (LVGL).
3. **Throttle calibration capture + storage** — capture/store/log endpoints, but **do not
   yet change the live mapping**. *Still safe.*
4. **Switch throttle mapping to calibrated endpoints** behind sanitize+fallback. *The
   safety-critical change — most review + HIL testing; lands last.*
5. **Interactive output checks** (buzzer/vibe/LED) + production serial record + BLE surfacing.

## Open decisions (need your call)

- Deadband split: keep a fixed floor + percentage as above, or pure percentage?
- Save endpoints only, or also a measured center/curve (if any unit is non-linear)?
- QC screen: auto-pass output checks after the cue, or require an explicit button confirm per device?
- Re-QC on every major FW bump, or only on demand?
- Store calibration in `deviceData` (simplest) or a separate factory namespace (cleaner separation of factory vs user data)?
