# Hand-controller hardware check (Layer 1)

`hw_check.py` checks a hand controller plugged into your computer over USB,
and optionally over your computer's Bluetooth, using only what the firmware
already reports. It needs no fixture and changes nothing permanently: it puts
back every setting it touches, and it never arms the controller or sends throttle.

It is Layer 1 of the release-bench plan. Layer 2 (a bench fixture plus a CAN
adapter) and Layer 3 (camera, real phone) cover what a cable can't. See
[Coverage](#coverage) below.

## Setup

```bash
pip install -r tools/hw_check/requirements.txt   # pyserial; bleak for --ble; esptool for --esptool
```

Python 3.9 or newer, on Windows, macOS or Linux.

## Run it against a unit

Plug the controller in over USB with the motor disconnected and no prop, then run:

```bash
python tools/hw_check/hw_check.py                     # USB serial checks, port auto-detected
python tools/hw_check/hw_check.py --port COM7         # or name the port
python tools/hw_check/hw_check.py --ble --expect-esc --expect-bms --json-out results.json
```

Every check prints one line: `PASS`, `FAIL`, `WARN` or `SKIP`. The exit
code is 1 if anything failed. `--json-out` writes the same results as JSON so
the release checklist can record the evidence.

| Option | Effect |
|---|---|
| `--expect-version 8.1` | Firmware the unit must report. Defaults to `inc/version.h` in this checkout. |
| `--no-reboot` | Skip the reboot. Settings persistence, reset reason and boot log are then not checked. |
| `--ble` | Add the Bluetooth checks. The controller must already be bonded to this computer (see below). |
| `--ble-address` | Pick a controller when several are bonded. The default is the first `OpenPPG SP140` found. |
| `--expect-esc`, `--expect-bms` | Fail instead of skip when the ESC or BMS isn't reporting telemetry. |
| `--run-qc` | Send `run_qc`. The unit reboots into factory QC and the check waits up to 5 min while the operator follows the screen. Needs firmware with first-boot QC. |
| `--esptool` | At the end, read the flash partition table with esptool. This resets the unit. |
| `--firmware build/firmware.bin` | With `--esptool`, also check the image fits an OTA slot. |
| `-v` | Echo all serial traffic. |

### Bonding for `--ble`

The controller only accepts encrypted reads and writes, and only accepts a new
bond during the 60-second pairing window after power-up. Power-cycle the
controller, then pair "OpenPPG SP140 …" in your computer's Bluetooth settings
within that minute. After that, `--ble` reconnects on its own. The Fast-Link
telemetry packet is 198 bytes, so the link needs an MTU of at least 201. Every
current desktop Bluetooth stack negotiates that.

## Dry run (no hardware)

```bash
python tools/hw_check/hw_check.py --dry-run
python tools/hw_check/hw_check.py --dry-run --fault settings-not-persisted --fault no-bms --expect-bms
python -m unittest discover -s tools/hw_check -v
```

`--dry-run` runs every check, including the BLE, QC and esptool ones, against
`fake_controller.py`. It's a simulated controller that speaks the firmware's
USB serial protocol on a local socket, so the real pyserial path runs, and it
has the firmware's BLE layout and Fast-Link packets. `--fault` injects a
misbehaviour so you can see each check fail. The list is `FAULTS` in
`fake_controller.py`.

`tools/hc_emulator` isn't used here because it only emulates the LVGL screen.
It has no USB serial, BLE or settings storage to check against.

The unit tests do two things:

- **End-to-end dry runs.** They prove that a healthy unit passes every check
  and that each fault fails the check it should.
- **Firmware drift checks.** They parse the firmware sources and fail if the
  tool no longer matches them. The sources are `send_device_data` and
  `parse_serial_command_line` in `device_settings.cpp`, the diagnostics JSON,
  the boot log strings in `main.cpp`, the UUIDs in `ble_ids.h`, the
  `BLE_FastLink_Telemetry` struct, the partition CSV, and the QC record format
  when `qc_logic.cpp` is present.

CI runs both in the **Hardware Check Tool** job.

## What each check does

| Check | How | Fails when |
|---|---|---|
| `serial.link` | every serial request | the unit stops answering, or answers with a damaged reply (bytes lost, keys missing). A damaged reply is a firmware fault and isn't retried around. |
| `fw.version` | serial `sync` | arch isn't ESP32S3, or the version isn't `--expect-version` |
| `diag.crash_history` | serial `diag_sync` | a retained boot record shows PANIC, a watchdog, BROWNOUT or a core dump |
| `settings.sanitize` | writes out-of-range rotation, sea pressure, mode and theme | the unit stores them instead of falling back to defaults |
| `settings.write` | writes a non-default value for all 6 settings | the `sync` echo differs |
| `settings.persist` | serial `reboot`, then `sync` | a setting didn't survive the reboot (NVS) |
| `diag.reset_reason` | `diag_sync` after the reboot | the reboot isn't recorded as SOFTWARE / usb_command_reboot |
| `boot.banner`, `boot.complete`, `boot.alerts`, `sensors.baro_init` | boot log after the reboot | the banner version differs, the barometer failed to init, or boot never completed (WARN). Monitor alerts are WARN. |
| `ota.boot_partition` | boot log `OTA: Running partition …` | the running and boot partitions differ (a pending or failed rollback) |
| `settings.restore` | writes the original settings back | the unit's settings weren't restored. This always runs. |
| `qc.run` / `qc.boot_record` | `--run-qc`, or a QC record in the boot log | the QC record isn't `PASSED`, or any check isn't pass/skip |
| `ble.fw_version` | FW version characteristic (major, minor, build) | it differs from what serial reports |
| `ble.settings_write` | writes the theme over BLE, reads it over serial, restores it | the BLE write didn't reach the controller's settings |
| `telemetry.stream` | 3 s of Fast-Link notifications | no packets, the wrong protocol version, or more than 10% dropped (WARN) |
| `state.disarmed` | Fast-Link `device_state` | the unit isn't DISARMED |
| `sensors.baro`, `sensors.cpu_temp`, `sensors.throttle_pot` | Fast-Link | pressure is outside 300–1100 hPa, CPU temperature is outside 0–85 °C, or the pot is out of range |
| `telemetry.esc` | Fast-Link ESC fields | ESC error or self-check bits are set, or the motor is turning while disarmed |
| `telemetry.bms` | Fast-Link BMS fields | the SOC, fail level, or a cell is outside 2.5–4.3 V, or the cells don't add up to the pack voltage |
| `ota.ble_service` | GATT discovery | the OTA service or any of its 4 characteristics is missing |
| `qc.stored_record` | QC record characteristic (paged read) | no record, or the record isn't `PASSED`. SKIP on firmware without first-boot QC. |
| `ota.partition_table` | `--esptool` reads the table at 0x8000 | there aren't 2 OTA app slots and otadata, or `--firmware` doesn't fit a slot |

## Coverage

This maps the hardware checklist areas from the bench plan to what covers each one.

**Covered by this tool**

- **Firmware identity:** the version over serial and BLE, the BLE build number, the git hash in the boot banner.
- **Settings:** write, sanitize, persist across a reboot, restore, and a BLE write landing in NVS.
- **Diagnostics:** the reset reason of a planned reboot, and retained crash, watchdog, brownout and core-dump history. Run it after a soak to check the soak.
- **First-boot QC:** the stored QC record over BLE, and a full QC run with `--run-qc` while an operator follows the prompts.
- **Sensor and power presence:** barometer, CPU temperature, throttle pot ADC, ESC telemetry and fault bits, and BMS telemetry with a cell sanity check.
- **OTA readiness:** BLE OTA service present, unit disarmed, running partition equal to the boot partition, the partition table, and the image fitting a slot.

**Layer 1, not built yet (still USB and the computer's Bluetooth only)**

- **BLE OTA transfers:** a bonded update to the newer build, refusal from a never-paired adapter, a link drop mid-transfer and at the end, the stall timeout, and the rollback image. This needs a Python client for the Espressif OTA protocol in `ota_service.cpp`.
- **USB flashing paths:** 7.x to the release candidate over USB, installing 8.0 and then updating over BLE, and an erase-flash to make a fresh unit for QC (esptool).
- **Settings across a firmware upgrade:** set non-default values on 8.0, upgrade, and check them. The settings checks here plus the flashing above make up that check.
- **Idle soak loop:** repeat `diag_sync` and telemetry over hours, and report heap and reset history.

**Needs Layer 2 (bench fixture, USB-CAN adapter, BMS on a relay)**

- All cruise and throttle checks, with exact µs read off the CAN bus.
- Arming and disarming through the real button path, and update-while-armed refusal.
- The interactive parts of QC: squeezes, skips, a dead ESC, a bad calibration, and a power cut mid-QC.
- Ten cold boots, ESC and BMS unplug and recovery, the ESC-settings interlock, and a full soak with everything connected.

**Needs Layer 3 (optional)**

- **Display:** theme, cruise icon and climb bar, through a webcam compared with the screenshot references.
- **Real phone:** one real update through the OpenPPG app on an Android phone over adb.

**Stays manual:** ground runs with the prop, test flights, the climb bar in
flight, and the release steps.
