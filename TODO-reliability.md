# Reliability & Crash Prevention TODO

Identified via comprehensive 15-agent code audit (2026-03-30).

## CRITICAL

### C2: Fix state machine race conditions
- `currentState` is read without mutex in `handleThrottle()`, `refreshDisplay()`, and `bleNotifyTask()`
- Multiple volatile reads per function can see different states (torn reads)
- **Fix:** Snapshot `currentState` once per function call into a local variable; or acquire `stateMutex` for reads
- **Files:** `main.cpp:1149,414-419,380`

### C3: Add throttle task to hardware watchdog
- Only `watchdogTask` is registered with `esp_task_wdt` (line 316)
- If throttle task hangs, ESC runs uncontrolled for 300ms then resets — no WDT trigger
- **Fix:** Call `esp_task_wdt_add(throttleTaskHandle)` in watchdog setup; call `esp_task_wdt_reset()` in throttle loop
- **Files:** `main.cpp:313-325,340-349`

### C4: Add stuck-throttle detection (ADC validation)
- `readThrottleRaw()` returns raw ADC with zero validation
- No detection of stuck-at-4095 (full throttle), stuck-at-0, or frozen values
- **Fix:** Track last ADC value and time; flag if near max (>4000) for >500ms; expose `isThrottleStuck()` for monitors
- **Files:** `throttle.cpp:48-51`, `throttle.h`

### C7: Fix OTA firmware corruption risk
- Intermediate OTA data packets have no per-packet CRC (only final sector CRC)
- `receivedBytes` vs `imageTotalLen` size mismatch only checked at CMD_END, after data is already written to flash
- **Fix:** Check `receivedBytes + sectorBufferLen > imageTotalLen` before each `esp_ota_write()`; add max packet length guard (>520 bytes)
- **Files:** `ble/ota_service.cpp:272-280,231`

### C8: Make mutex/queue creation failures fatal
- `lvglMutex`, `stateMutex`, all queues — creation failures print to serial but boot continues
- Subsequent `xSemaphoreTake(NULL, ...)` causes undefined behavior / crash
- **Fix:** Track `allOk` flag; halt boot with `while(true) vTaskDelay(1000)` if any creation fails
- **Files:** `main.cpp:592-634`

## HIGH

### H1: Implement CAN bus recovery
- TWAI alerts configured for `BUS_ERROR`, `ERR_PASS`, `RX_QUEUE_FULL` but never monitored after init
- No `twai_initiate_recovery()` on bus-off state; ESC commands silently fail
- **Fix:** Add `twai_get_status_info()` check in `readESCTelemetry()`; call `twai_initiate_recovery()` on `TWAI_STATE_BUS_OFF`
- **Files:** `esc.cpp:185,203-259`

### H6: Add NULL checks for LVGL display/timer/driver creation
- `lv_display_create()` return not checked — NULL dereference crashes flush callback
- `lv_timer_create()` for critical border flash not checked — state inconsistency if NULL
- `tft_driver` and `px_map` not validated in `lvgl_flush_cb()` — hard fault risk
- **Fix:** Add NULL checks after each creation; return early / reset state on failure
- **Files:** `lvgl_core.cpp:38,50-53,70`, `lvgl_updates.cpp:212,244`

## MEDIUM

### M1: Increase monitoring task frequency from 10Hz to 20Hz
- `monitoringTask` runs at 10Hz (`vTaskDelay(pdMS_TO_TICKS(100))`)
- Safety-critical conditions (ESC over-current, over-temp) detected with up to 100ms latency
- **Fix:** Change delay from 100ms to 50ms
- **Files:** `main.cpp:490`

### M7: Release LVGL mutex between splash and main screen setup
- `lvglMutex` held for ~3s during splash screen + main screen setup
- Any task needing the mutex (display flush) blocked for the entire duration
- **Fix:** Release mutex after `displayLvglSplash()`; re-acquire before `setupMainScreen()`
- **Files:** `main.cpp:771-796`

### M10: Add NaN/infinity guard to vertical speed calculation
- If altitude buffer contains NaN readings, `(NaN - X) / timeDiff = NaN`
- `constrain(NaN, ...)` returns NaN — propagates to display and BLE telemetry
- **Fix:** Add `isfinite()` checks on `newest.altitude` and `oldest.altitude` before calculation; return 0.0f on invalid
- **Files:** `altimeter.cpp:36-56`

### M11: Tighten cell voltage differential warning threshold
- Current: warn at 0.2V, critical at 0.4V
- Typical BMS specs alert on 0.05-0.10V imbalance
- **Fix:** Change to warn 0.1V, critical 0.3V
- **Files:** `monitor_config.h:21`

### M13: Add vibration feedback for critical alerts
- Critical alerts get visual feedback (red border flash) but NO vibration
- Warning alerts get `VIBE_DOUBLE_PULSE` but only when no criticals present
- Pilot may miss critical alerts during high workload
- **Fix:** Add `VIBE_TRIPLE_PULSE` (or stronger pattern) when `deltaCriticals > 0` in `handleAlertVibration()`
- **Files:** `alert_display.cpp:203-213`
