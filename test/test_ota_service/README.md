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
images (`test/test_screenshots/reference/ota_*.bmp`) across both themes. On 8.1
they are rendered with the hinted Montserrat set (8.1 disables LVGL's built-in
fonts); `test/test_screenshots/review/` holds 2x contact sheets of them. They also
check text, screen bounds, retry transitions, cleanup, and progress calculation.
Existing main-screen and splash references are unchanged.

## Display behavior

- Preparing, receiving, checking, and restarting keep the update screen active.
- Progress uses successfully written bytes and paints at most every 250 ms.
- Phase changes paint on the next UI tick. The BLE callbacks only publish a
  status copy; nothing in the OTA path waits for the display (8.1 port).
- Errors remain for 10 seconds; an app retry immediately starts a new screen.
- A START refused because the controller is armed publishes nothing: the
  flight display stays up. The screen is never shown while armed.
- 8.1 reboots from `checkOtaTimeout()` about 1 s after the END ACK; the
  target-slot receipt is written to NVS there, just before the reboot, never
  from the BLE write callback. After reboot it produces a 3.5-second completion
  screen with the running version, or "Previous version kept" only when the
  bootloader marked the target slot invalid (rollback). A receipt for a slot
  that is neither running nor invalid is cleared without a screen. Receipt
  storage failure does not fail an otherwise successful installation.
- Button gestures are consumed while the update/result screen is visible and
  the controller is disarmed. A button held across the transition must be
  released before a new gesture.
- The existing app UUIDs, commands, packet formats, and ACK values are unchanged,
  including 8.1's START refusal codes (2 armed, 3 too large, 4 flash).

The serial console logs `OTA status: <phase> (<error>)` when the service
publishes a phase change and `OTA screen: <phase> (<error>) <n>%` /
`OTA screen: closed` when the UI task actually enters or leaves the screen.

Before release, exercise updates from Android and iOS on the controller: check
transfer responsiveness, disconnect/retry, the automatic reboot/result screen,
and button handling during and after an update. Host screenshots and mocked
flash tests cannot establish hardware behavior. Firmware already on the device
must contain this UI for it to appear during a transfer.
