# Rust ESP32-S3 firmware

This workspace is the production firmware on this branch. All controller state, tasks, sensor monitoring, BLE/ESP-NOW services, settings, diagnostics, ESC maintenance, feedback, and UI orchestration are Rust. ESP-IDF supplies the RTOS/drivers/radio stack; LVGL 9.6, Bosch BMP3 compensation and libcanard remain C. `platform-rust` links only low-level peripheral calls from the native branch. The C++ application is retained as a reference and is excluded from the Rust image.

## Toolchain

Use ESP-IDF **v5.5.3**, host Rust **1.100.0**, espup **0.19.0**, Xtensa Rust **1.97.0.0**, and ldproxy **0.3.5**. Install IDF and its ESP32-S3 tools through Espressif's `install.sh esp32s3`, then activate its `export.sh` in the same shell as the Rust environment:

```sh
rustup toolchain install 1.100.0
cargo +1.100.0 install espup --version 0.19.0 --locked
espup install --toolchain-version 1.97.0.0 --targets esp32s3 --std
cargo +1.100.0 install ldproxy --version 0.3.5 --locked
. "$HOME/export-esp.sh"
. "$IDF_PATH/export.sh"
bash rust/tools/build.sh
```

Run the build from any directory. `IDF_PATH` must identify the pinned SDK. `EPPG_COMPONENTS` is an optional external components directory for development against the first branch; a normal checkout uses its own vendored components.

`rust/tools/build.sh` builds the complete release image and exports `rust/target/xtensa-esp32s3-espidf/release/images/`. `firmware.bin` is app-only for the existing OTA slots. `recovery.bin` includes bootloader, unchanged partition table, initial OTA selection, and app0; flashing this contiguous image at offset 0 erases settings, BLE bonds, radio configuration and OTA selection in the covered flash range. Use the app-only image for normal updates. The exporter checks the firmware discovery marker, slot size, installed partition geometry, and linked-object exclusions, and writes `release-check.json` and a link map. App-only updates retain the existing NVS and radio namespaces.

## Native verification and emulator

```sh
cd rust
cargo +1.100.0 test --locked --workspace --exclude firmware-sp140
cargo +1.100.0 clippy --locked --workspace --exclude firmware-sp140 --all-targets -- -D warnings
cargo +1.100.0 fmt --all -- --check
cd ..
./simulator/run.sh --rust --test-only
cd simulator/ui && npm ci && npm start
```

On macOS, unset `LIBCLANG_PATH` inherited from `export-esp.sh` before native tests so bindgen uses the system host libclang.

The native emulator consumes the same Rust controller, monitors, UI and ESC maintenance engines. Raw CAN traverses the pinned C libcanard transport and synthetic ESC; the maximum-size image test stages through the actual portable C staging engine on a NOR flash model. See [controller-sim](apps/controller-sim/README.md) for CLI/scenarios.

```sh
bash rust/tools/build.sh --qemu
python3 simulator/tools/qemu_smoke.py --help
```

The QEMU image executes the actual Xtensa-compiled Rust firmware and production tasks. Its shared device model supplies ADC/button/SPI/I2C/PWM/TWAI peers. QEMU cannot emulate the ESP32 radio, so the QEMU profile explicitly skips physical BLE/WiFi initialization; host protocol tests exercise radio framing and mobile OTA packet sequences. Physical builds initialize both radios normally.

## Runtime performance and memory

The renderer owns one four-byte-aligned 64-row RGB565 buffer. LVGL widgets use the
ESP-IDF allocator instead of reserving a separate 48 KiB arena. Numeric formatting
uses bounded stack buffers; stable telemetry/alert updates have an allocation-budget
regression test. UI snapshots copy only display fields, avoiding maintenance-buffer
allocations while the control mutex is held. The single sensor owner publishes at
25 Hz, while the control task retains its 100 Hz period.

LVGL and libcanard are compiled without PIC on ESP-IDF to match its static firmware
link model. Host libraries keep their normal PIC settings. Rust release optimization
remains size-oriented: measured `-O2`/`-O3` experiments were slower on this board.
Failed LVGL allocations abort explicitly. CI runs the C rendering boundaries under
AddressSanitizer as well as the screen goldens and firmware/emulator checks.

Measured board results, raw counters, build settings and limits are recorded in the
[device runtime benchmark](../docs/device-runtime-performance-2026-11-13.json).

## Compatibility and limits

The BLE UUIDs, signed 32-bit time wire value, FastLink v4 offsets, existing NVS types, USB JSON keys, OTA sector framing, wireless sessions/timeouts and ESC maintenance services are preserved. Full 256 KiB ESC uploads use a bounded scratch buffer and the existing unused reserved flash partition. An ownership marker protects unknown pre-existing partition data: uploads fail without erasing data if it is neither virgin nor owned by this staging format.

The Rust ANT decoder intentionally fixes an inherited identity-frame indexing defect: all three identity fragments are accepted without an out-of-bounds access. Controller stalls stop new remote packets, preserving the receiver's 150 ms failsafe. C++ conversion of NaN temperatures to integers was undefined; raw temperatures stay separate for monitor/UI decisions and wire conversion is deterministic in Rust. Hardware RF pairing/bond persistence, motor output timing, physical OTA power-loss behavior and real sensor compensation still require board validation; native/QEMU tests do not substitute for physical acceptance.
