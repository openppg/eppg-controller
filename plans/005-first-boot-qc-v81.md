# Plan 005: FIRST_BOOT_QC — factory self-test + throttle calibration capture (v8.1)

> **Executor instructions**: The firmware implementation for this plan is
> COMPLETE on branch `first-boot-qc` (commits: design doc, factory_settings +
> qc_logic + tests, POST engine, LVGL screen, calibration capture, interactive
> checks, BLE record, run_qc command). What remains is the **HIL bench
> checklist** below, which requires physical hardware, and the PR into
> `unified-everything-app` before the v8.1 tag. If anything in "STOP
> conditions" occurs, stop and report. When done, update this plan's status.
>
> **Drift check (run first)**: `git log --oneline unified-everything-app..first-boot-qc`
> should show the QC commit series. If `src/sp140/throttle.cpp` mapping
> functions (`potRawToPwm`/`potRawToModePwm`) show ANY diff on this branch,
> treat it as a STOP condition — v8.1 is capture-only.

## Status

- **Priority**: P1
- **Effort**: L (firmware done; HIL verification outstanding)
- **Risk**: LOW on the installed fleet (migration guard + capture-only); MEDIUM on factory-line workflow until HIL-verified
- **Depends on**: none (v8.2 mapping switch depends on THIS plan's calibration data)
- **Category**: feature (factory QC / production traceability)
- **Planned at**: branch `first-boot-qc` off `unified-everything-app` (88deeaa), 2026-07-06
- **Design doc**: [FIRST_BOOT_QC.md](../FIRST_BOOT_QC.md)

## Why this matters

Real units vary: released throttle reads ~80–250 counts (not 0) and full press
~3850–4095 (not always 4095). With the fixed `0..4095` mapping, small-span
units silently lose top-end power and high-idle units risk phantom throttle /
blocked arming — both show up as real support tickets (throttle-cable faults,
arming problems). There is also no automated factory QC: PCB/IC defects on new
boards (I2C/SPI/CAN paths) are caught by hand or by the customer.

This plan ships, in v8.1: automatic POST (bus-communication checks), guided
per-unit throttle calibration **capture** (endpoints stored in the factory NVS
namespace, live mapping untouched), operator-confirmed output checks, a JSON
QC record per unit over USB serial, the same record over BLE for silent fleet
sync (app → Supabase), and a serial `run_qc` re-entry command.

**The v8.2 mapping switch is data-gated on this release**: the pot_min/max
records collected here tell us the real span distribution before any live
throttle behavior changes.

## Decisions locked (Zach, 2026-07-04)

1. Screen-first UX; the throttle pot is the confirmation input; the button is
   used only for its own test + absent-device skip-confirms.
2. QC entry: **fresh NVS, prior failed/aborted attempt (`qc_attempted`), or
   serial `run_qc`** — no button-hold entry. Fleet units (user NVS, never
   attempted) are silently back-filled and never see QC.
3. Storage: `"openppg-factory"` NVS namespace (survives user factory reset;
   never abbreviate "oppg"). Keys include `qc_attempted` so fail/power-cut
   retries instead of looking like a legacy fleet unit after defaults write.
4. Existing fleet units (NVS data from ≤v8.0, never attempted QC) are
   back-filled `qc_passed=1`, never auto-calibrated, and never see the QC flow.
5. POST is framed as bus tests (I2C baro / SPI MCP2515 / CAN ESC active
   request-response / CAN BMS passive listen). No operator LED check (LED is
   inside the case; TFT is the feedback surface).
6. ESC/BMS absent on the bench → skip-confirm pattern (button press records
   "skip", not "fail"); QC is passable on a bare controller.

## Current state (implemented on `first-boot-qc`)

| Piece | File(s) |
|---|---|
| Pure logic (gate, cal gates, stability window, v8.2 mapping scaffold, JSON) | `inc/sp140/qc_logic.h`, `src/sp140/qc_logic.cpp` |
| Factory NVS namespace | `inc/sp140/factory_settings.h`, `src/sp140/factory_settings.cpp` |
| QC flow (POST, calibration, interactive, record) | `inc/sp140/first_boot_qc.h`, `src/sp140/first_boot_qc.cpp` |
| LVGL screen (checklist / prompt / banner) | `inc/sp140/lvgl/lvgl_qc_screen.h`, `src/sp140/lvgl/lvgl_qc_screen.cpp` |
| Boot hooks (context capture pre-refreshDeviceData; Phase 4.5 gate) | `src/sp140/main.cpp` |
| BLE record characteristic (paged fetch) | `inc/sp140/ble/ble_ids.h` (QC_RECORD_UUID), `src/sp140/ble/config_service.cpp` |
| `run_qc` serial command | `src/sp140/device_settings.cpp` |
| Direct vibe drive for pre-task cues | `src/sp140/vibration_pwm.cpp` (`vibeDirectSet`) |
| Tuning constants | `inc/sp140/shared-config.h` (`QC_*`) |
| Native tests (gate + cal + pot-confirm levels + JSON) | `test/test_qc/test_qc.cpp` |
| Screenshot tests (4 states + references) | `test/test_screenshots/` |
| Bench logger | `tools/qc_bench_logger.py` |

## HIL bench checklist (outstanding — requires hardware)

Run with `python3 tools/qc_bench_logger.py --port <port>` attached:

1. **Fresh unit**: erase NVS (`pio run -t erase` then flash, or `esptool
   erase_region` of the NVS partition) → boot → full QC flow appears; complete
   all steps → banner PASSED → JSON captured → reboot → normal boot, no QC.
2. **Existing-fleet simulation**: flash a unit that already has v8.0 settings
   (or seed `openppg` namespace) with empty `openppg-factory` → boot → **no QC
   UI ever appears**, serial shows "existing unit detected - back-filling",
   normal boot proceeds.
3. **Serial re-entry**: `{"command":"run_qc"}` over USB → reboots into QC.
4. **Bare controller**: no ESC/BMS attached → skip-confirm prompts appear;
   button press records `"skip"`; final result PASSED with skips listed.
5. **Attached-but-dead device**: ESC cabled but unpowered → operator does NOT
   confirm skip → check records `"fail"`, unit FAILED → **reboot → QC UI
   appears again** (must not legacy-backfill).
6. **Bad pot simulation**: restrict lever travel (or inject via bench pot) so
   span < 2000 → cal FAILS, defaults kept, `pot_calibrated` stays 0 → reboot
   re-enters QC.
7. **Calibration sanity**: on a good unit compare captured pot_min/max against
   a bench meter; re-run `run_qc` and confirm repeatability within epsilon.
8. **BLE record**: from the app (or a BLE tool), write offset 0 to
   QC_RECORD_UUID and read chunks → reassembled JSON matches the serial line.
9. **User factory reset**: perform a settings reset → `openppg-factory`
   namespace (qc_passed, pot_min/max) survives.
10. **Watchdog**: let QC idle at a prompt ≥60 s → no WDT reset (step timeout
    fires instead).

## Verification (build/test — already green at implementation time)

```
~/.platformio/penv/bin/pio test -e native-test          # 29 QC tests + existing suites
./test/test_screenshots/build_and_run.sh                # 32 pass (2 pre-existing splash failures on base)
~/.platformio/penv/bin/pio run -e OpenPPG-CESP32S3-CAN-SP140   # clean build
cpplint (CI flags) over changed files                   # clean
```

## STOP conditions

- Any diff required in `potRawToPwm` / `potRawToModePwm` / arming logic →
  STOP (that is v8.2, data-gated; v8.1 must be zero-live-behavior-change).
- HIL item 2 shows ANY QC UI on an existing-fleet unit → STOP; the migration
  guard is the fleet-safety contract.
- QC loop trips the task WDT despite `vTaskDelay` yields → STOP (WDT strategy
  decision needed).
- `openppg-factory` namespace fails to open on real hardware partition layout
  → STOP.
- Known pre-existing (NOT this plan's scope, do not chase here): splash
  screenshot failures on macOS; pio `native-screenshot` env linker error on
  macOS (CI uses the CMake harness, which is green).

## Follow-ups (tracked, not in this plan)

- v8.2: switch live throttle mapping to calibrated endpoints behind
  `sanitizeDeviceData`-style validation + fallback (scaffold + tests already
  in `qc_logic`), gated on factory calibration data from this release.
- App-side: silent QC_RECORD_UUID read on connect + Supabase upload
  (openppg_android / site repos).
- Factory: adopt `tools/qc_bench_logger.py` on the bench; batch outlier
  review of pot spans.
