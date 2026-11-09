# OTA screen coverage

The screenshot runner also builds and runs `ota_service_tests`:

```sh
./test/test_screenshots/build_and_run.sh --local
```

The service tests compile the production BLE OTA callbacks with isolated host
fakes for flash, BLE, time, NVS, and RTOS primitives. They cover accepted/rejected
starts, preparation, successful writes, CRC retries, duplicate sectors/starts,
disconnect, timeout, incomplete images, verification/boot-selection failures,
restart, and the one-time boot result. Command ACKs retain their 20-byte format
and CRC; sector ACKs retain their six-byte format. The tests do not model actual
FreeRTOS scheduling, radio timing, or flash-cache stalls.

The renderer tests use the production LVGL screen and compare 24 reference
images (`test/test_screenshots/reference/ota_*.bmp`) across both themes. They also
check text, screen bounds, retry transitions, cleanup, and progress calculation.
Existing main-screen and splash references are unchanged.

## Display behavior

- Preparing, receiving, checking, and restarting keep the update screen active.
- Progress uses successfully written bytes and paints at most every 250 ms.
- Phase changes paint immediately. Before preparation, verification, and restart,
  the OTA task allows up to 250 ms for a rendered-frame acknowledgment; missing
  display acknowledgment cannot indefinitely stall the app protocol.
- Errors remain for 10 seconds; an app retry immediately starts a new screen.
- After reboot, a persisted target-slot receipt produces a 3.5-second completion
  screen with the running version, or a failure if the previous slot was booted.
  Receipt storage failure does not fail an otherwise successful installation.
- Button gestures are consumed while the update/result screen is visible. A
  button held across the transition must be released before a new gesture.
- The existing app UUIDs, commands, packet formats, and ACK values are unchanged.

Before release, exercise updates from Android and iOS on the controller: check
transfer responsiveness, disconnect/retry, the automatic reboot/result screen,
and button handling during and after an update. Host screenshots and mocked
flash tests cannot establish hardware behavior. Firmware already on the device
must contain this UI for it to appear during a transfer.
