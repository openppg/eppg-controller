# EPPG Controller

[![Build](https://github.com/openppg/eppg-controller/actions/workflows/config.yml/badge.svg)](https://github.com/openppg/eppg-controller/actions/workflows/config.yml)

Rust firmware for the ESP32-S3 OpenPPG SP140 Throttle Controller. Cargo builds
the application on ESP-IDF 5.5.3 and FreeRTOS, with no Arduino SDK dependency.
Controller logic, tasks, services, chip drivers, diagnostics and screen composition
are Rust. ESP-IDF, LVGL, Bosch compensation and libcanard remain C dependencies;
a small native peripheral boundary is shared with the preceding ESP-IDF branch.

### Key Features

- CAN bus communication with ESC and BMS
- LVGL-based LCD display interface
- Bluetooth Low Energy (BLE) connectivity via NimBLE
- Barometric altitude sensing (BMP3XX)
- Multiple flight modes (manual, cruise)
- Real-time sensor monitoring with hysteresis-based alerting
- Web-based configuration via [config.openppg.com](https://config.openppg.com) (no firmware flash needed)
- Optional encrypted ESP-NOW controller-to-controller control and mirrored telemetry; see [WIRELESS_CONTROL.md](WIRELESS_CONTROL.md)

The migration is split into stacked branches: `codex/esp-idf-no-arduino` removes
Arduino while retaining the C++ application; `codex/rust-port` builds on that
branch and makes Rust the production application. The C++ sources remain as
compatibility references and are excluded from the Rust firmware image.

### Reasons for this migration

- The ESC transport has one Rust owner; BLE and radio workers send bounded typed requests.
- Shared telemetry and settings use synchronized snapshots instead of unsynchronized application globals.
- Firmware and the native emulator execute the same controller, monitor, ESC and UI code.
- Fixed-width byte encoders make BLE, OTA and wireless compatibility independent of compiler struct layout.
- Removing Arduino allows direct use of the pinned ESP-IDF SDK and its drivers.
- Cargo locking, host tests, clippy and screenshot fixtures make changes easier to reproduce and review.

These are architectural and correctness benefits. Faster execution, smaller images
or lower power consumption are not assumed. The [migration assessment](advisor-plans/001-rust-port.md)
records the repository-specific rationale and original feature inventory.

X4 code has been migrated to a separate repo - https://github.com/openppg/x4-controller

It may not be stable and is not recommended for flying.
See stable releases [here](https://github.com/openppg/eppg-controller/releases)
<details>
<summary>Older Version Information</summary>

For non-CAN bus controllers please use [version/6 branch](https://github.com/openppg/eppg-controller/tree/version/6).

Version 6.1 was initially released only for the reworked RP2040 module based controller and being ported to the original RP2040 PCB which is currently in testing. The version shipped with that hardware can be found on the [rp2040-module-release branch](https://github.com/openppg/eppg-controller/tree/rp2040-module-release)

Version 6.0 introduced [FreeRTOS](https://www.freertos.org/index.html) and is currently only working with RP2040 processors. For M0/SAMD21 processors please use [version/5 branch](https://github.com/openppg/eppg-controller/tree/version/5).

For batch 3 (non-telemetry) controllers please see the [batch-3 branch](https://github.com/openppg/eppg-controller/tree/batch-3).

For batch 2 (Arduino nano based) controllers please see the [batch-2 branch](https://github.com/openppg/eppg-controller/tree/batch-2).
</details>

## Build and flash firmware

Follow the pinned toolchain installation in [rust/README.md](rust/README.md):
ESP-IDF 5.5.3, host Rust 1.100.0, espup 0.19.0, Xtensa Rust 1.97.0.0 and
ldproxy 0.3.5. Activate the IDF and espup export environments, then run:

```sh
bash rust/tools/build.sh
```

Artifacts are written to `rust/target/xtensa-esp32s3-espidf/release/images/`:

- `firmware.bin`: app-only image for the existing BLE OTA clients.
- `firmware.elf` and `firmware.map`: matching crash/debug evidence.
- `recovery.bin`: bootloader, partition table, OTA selection and app0 for recovery.
- `release-check.json`: image size, discovery marker and linked dependency checks.

The exporter verifies the installed 8 MB partition geometry and excludes Arduino
and the legacy C++ application. Existing NVS keys/types, BLE UUIDs and wire formats
are retained. A contiguous full recovery write at offset 0 erases settings, BLE bonds, radio
configuration and OTA selection in the covered flash range; normal app
OTA uses the inactive slot and preserves settings. ESC uploads retain the full
256 KiB limit using bounded buffers and the reserved scratch partition. Unknown
pre-existing scratch-partition data is rejected without erasure.

Production startup is [rust/apps/firmware-sp140/src/main.rs](rust/apps/firmware-sp140/src/main.rs).
See [Rust architecture, compatibility and build details](rust/README.md) and
[retained dependency provenance](components/README.md).

## Testing

Native tests require host Rust 1.100.0, a C compiler and system libclang. Use the
system libclang for host tests; unset `LIBCLANG_PATH` if it was set by espup.

```sh
cd rust
unset LIBCLANG_PATH
cargo +1.100.0 test --locked --workspace --exclude firmware-sp140
cargo +1.100.0 clippy --locked --workspace --exclude firmware-sp140 --all-targets -- -D warnings
cargo +1.100.0 build --locked -p controller-sim
cd ..
./simulator/run.sh --rust --test-only
```

These exercise production control/state transitions, monitoring, explicit wire
codecs, DroneCAN peers, maintenance sessions, NOR-staged maximum-size ESC images,
phone OTA packet sequences, and all 34 existing screen fixtures. The native C++
regressions and pinned protocol oracles remain separately available under `tools/`
and run in CI as compatibility references.

## Interactive emulator

```sh
./simulator/run.sh --rust --test-only
cd simulator/ui
npm ci
npm start
```

Open `http://127.0.0.1:4174/`. The lab drives the production Rust controller,
serializes ESC/BMS telemetry onto real CAN frames, and renders the actual Rust
LVGL screen as a live 160×128 framebuffer. It supports button/throttle input,
wire faults and scenario replay. [Emulator commands and fidelity](simulator/README.md)
describe the modeled devices.

For execution of the cross-compiled ESP32-S3 instructions and FreeRTOS tasks:

```sh
bash rust/tools/build.sh --qemu
python3 simulator/tools/qemu_smoke.py \
  --image rust/target/qemu/xtensa-esp32s3-espidf/release/images/qemu-flash.bin \
  --qemu /path/to/qemu-system-xtensa --timeout 30 --control-proof \
  --expect 'Rust controller boot complete' --expect 'pwm_01us=17700' \
  --report rust/target/qemu/smoke.json
```

Use Espressif QEMU's `esp32s3` machine. Its build uses DIO flash and UART0 for the
emulator, with register-level MCP2515, ST7735 and BMP3xx models and a separate
DroneCAN ESC peer. Physical builds use QIO and USB Serial/JTAG. QEMU explicitly
skips unsupported BLE/Wi-Fi RF. Phone pairing/bond migration, electrical timing,
real sensors and physical update interruption still require board testing.

## Config tool

The open source web based config tool for updating certain settings over USB (without needing to flash firmware) can be found at [config.openppg.com](https://config.openppg.com).

## Help improve these docs

Pull requests are welcome for these instructions and code changes.
