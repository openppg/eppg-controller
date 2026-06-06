# Release Notes

## v8.0 — build 1190 (2026-06-05)

### Fixed

- **Controller reboots when arming with a phone connected over Bluetooth.**
  Completing the arm sequence while a BLE central (the companion app) was
  paired and connected caused the controller to panic and reboot instead of
  arming. Disconnected/unpaired, arming worked normally.

  **Root cause:** on every arm/disarm, `deviceStateUpdateTask` pushes the new
  device state to the app through a BLE characteristic notification. That task
  was created with only a **2 KB stack**, but NimBLE's `notify()` descends
  through the full BLE host stack (GATT → ATT → L2CAP → HCI), which needs more
  than 2 KB. The result was a FreeRTOS stack-overflow trap
  (`vApplicationStackOverflowHook`) → reboot. The notify only runs while a
  central is connected, which is why the crash appeared only after pairing.

  **Fix:** raised the `deviceStateUpdateTask` stack from 2048 to **8192 bytes**,
  matching the sibling `bleStateUpdateTask` that performs the same kind of
  notification. Added comments at both the stack allocation and the `notify()`
  call so the requirement isn't lost. (`src/sp140/main.cpp`)

  **Why it surfaced now:** the undersized stack had been latent since late
  January, but only became reachable after device-state notifications were
  re-enabled (2026-05-06) and the BLE pairing feature merged (PR #110), which
  together made a connected central — and therefore the notify path — the normal
  case during arming. The earlier NimBLE migration (replacing ESP32 BLE) is what
  made a 2 KB stack insufficient in the first place; the same class of overflow
  was already fixed for another task in `BLE: fix Tmr Svc stack overflow`.

### Known issues

- **Spurious "arm sequence started" logged right after arming.** Because the arm
  hold completes while the button is still pressed and the press timer is reset
  at completion, the subsequent button *release* is mis-read as a fresh
  quick-click and starts a new arm sequence. It self-cancels after the 1.5 s
  sequence timeout, so it's harmless log-noise in normal use, but the
  button-handling logic in `buttonHandlerTask` should be tightened so the
  post-arm release can't open a new sequence. Tracked as a separate fix.
