# Plan 001: Port the ESP32-S3 controller application to Rust and remove Arduino

> This is a researched migration plan, not an implemented or hardware-validated port.
> Execute the milestones in order. A compiling image does not satisfy a milestone's behavioral gates.
> Preserve the existing dirty working tree; do not reset it or replace it with HEAD.

## Status and baseline

- Planned at: `35eba22`, 2026-11-12, including the uncommitted source present that day.
- Category: migration. Priority: P1. Effort: L. Change risk: HIGH.
- Scope: all active ESP32-S3 firmware functionality, including the uncommitted wireless work and host simulator integration.
- Source size: approximately 12,272 lines of application `.cpp` across `src/sp140`, its BLE directory, and its LVGL directory, before headers, dependencies, assets, and tests.
- Baseline: `advisor-plans/001-baseline.sha256`. Check it before implementation with `shasum -a 256 -c advisor-plans/001-baseline.sha256`. Expected: every entry OK; investigate any changed/missing entry before using the cited excerpts as authoritative.
- Working tree already contained substantial modifications/untracked files. A HEAD-only diff is insufficient to reconstruct this baseline.
- No builds, installations, hardware operations, or firmware modifications were performed for this assessment. PlatformIO, CMake, and clang++ were on PATH; Cargo/rustup were not. This worktree has no `.pio` dependency cache. A read-only dependency cache in the original checkout supplied additional driver/protocol evidence.

The separate `plans/005-first-boot-qc-v81.md` describes work on another branch. Its named factory/QC implementation files are absent here. Do not claim those features are implemented in this baseline. Reconcile that branch before finalizing release scope if it is intended to ship with this port.

Load-bearing current-state excerpts for drift checks:

```ini
; platformio.ini, active ESP32-S3 environment
framework = arduino, espidf
board_build.partitions = default_8MB.csv
```

```cpp
// inc/sp140/structs.h:224
static_assert(sizeof(BLE_FastLink_Telemetry) == 230,
              "BLE_FastLink_Telemetry V4 size mismatch - update if struct changes");
```

```cpp
// src/sp140/main.cpp:310
TickType_t lastWake = xTaskGetTickCount();
const TickType_t throttleTicks = pdMS_TO_TICKS(20);  // 50 Hz
```

## Decision

Use a **Cargo-built Rust application on ESP-IDF and FreeRTOS**, targeting `xtensa-esp32s3-espidf` with the Espressif Xtensa Rust toolchain.

Remove Arduino startup, Arduino APIs, ArduinoJson, Preferences, Adafruit Arduino drivers, NimBLE-Arduino, CircularBuffer, and the Arduino-dependent C++ CAN wrappers from the production application. Implement controller logic, tasks, service handlers, protocol adapters, device drivers, and screen construction/update logic in Rust.

Retain ESP-IDF's C implementation of FreeRTOS, Wi-Fi/BLE controller support, flash/NVS, OTA, and hardware services. Retain LVGL 9.5.0 as a C dependency called from Rust. Initially retain the exact pinned libcanard C transport and generated DSDL C codecs behind a narrow adapter; replace `CanardAdapter`/`SineEsc` C++ orchestration in Rust. This delivers an Arduino-free, Rust-authored application, **not an exclusively Rust binary with no C dependencies**.

If exclusively Rust dependencies are a hard requirement, this recommendation does not meet it. That requires a separate `esp-hal`/runtime/radio/GUI/DroneCAN evaluation and a larger migration. Merely selecting `no_std` does not guarantee that the radio stack or all linked code is Rust.

### Why this stack is the best fit here

The firmware already uses FreeRTOS tasks, ESP-IDF TWAI, raw NVS writes, OTA partitions, and ESP-IDF watchdog/crash APIs. Keeping these facilities reduces the amount of behavior that must change while moving the application language. The [esp-rs ESP-IDF workspace](https://github.com/esp-rs/esp-idf) provides raw bindings, driver wrappers, and service wrappers; its maintainers identify those wrappers as community-maintained. That is a maintenance tradeoff, not an official Espressif Rust support guarantee.

The alternative `esp-hal` approach is viable for the S3, but changes scheduling, driver integration, radio hosting, persistence, and update integration together. Espressif's [current ancillary-crate overview](https://docs.espressif.com/projects/rust/book/introduction/ancillary-crates.html) documents radio/runtime components and their stability separately from the HAL. For this repo, choose that route only for a specific measured requirement that the IDF route cannot satisfy.

### Specific reasons to do the port

| Benefit | Concrete connection to this repository | What must be done to realize it |
| --- | --- | --- |
| Make CAN ownership enforceable | `main.cpp:1333` and `esc_config_relay.h` depend on the throttle task being the sole owner of the ESC protocol instance; BLE handlers must not call it concurrently. | Move the transport into one Rust task and expose typed, bounded requests. Ownership then prevents accidental direct access elsewhere. |
| Reduce cross-core memory hazards | `main.cpp:525` explicitly avoids torn reads of live telemetry; `telemetry_hub.cpp` supplies mutex-protected snapshots. Many other shared objects are global/volatile. | Replace application globals with owned state, safe channels, snapshots, and atomics for appropriate scalars. Rust rejects unsynchronized safe sharing; do not bypass it with blanket `unsafe impl Sync`. |
| Make OTA/session lifetimes easier to reason about | `ble/ota_service.cpp:18` documents the abort/write race that its lock now prevents. Config and ESC flashing also have mutable sessions and buffers. | One owner for each session, typed states, RAII cleanup, explicit abort/finish paths. This helps prevent regressions of already-fixed problems; it is not a claim that the current lock is missing. |
| Test the production control logic on a laptop | `simulator/CMakeLists.txt` includes production `throttle.cpp`, but `simulator/controller_sim.cpp` separately models much of `main.cpp` and monitoring. | Use the same hardware-independent Rust controller/monitor crates in firmware and simulator. This removes a source of model drift and makes randomized event/fault tests practical. |
| Make units and numeric boundaries explicit | ADC counts, PWM microseconds, tenths of a microsecond on CAN, milliseconds, seconds, and RTOS ticks are mixed across `throttle.cpp`, `esc.cpp`, and `main.cpp`. `main.cpp:720` records a historical seconds-versus-milliseconds watchdog mistake. | Use small types such as `AdcCounts`, `PwmMicros`, `Millis`, and `Duration`, with checked conversions. Rust alone does not distinguish units without these types. |
| Define protocol compatibility independently of compiler layout | FastLink uses a packed 230-byte layout; BLE time uses native `time_t`; throttle read/write/notify use different widths or endianness. | Explicit byte encoders, fixed-width fields, and golden vectors. This improves reviewability and portability; a naive Rust struct cast would lose the benefit. |
| Remove the Arduino version constraint | `platformio.ini` combines Arduino and IDF; the installed PlatformIO framework is IDF 4.4.7. Replacing Arduino opens a direct path to a selected supported IDF release. | Validate a compatible Rust/IDF/NimBLE version set and pin it. Newer SDK capability and fixes are benefits only after migration testing, not automatic improvements. |
| Simplify peripheral integration | Display and MCP2515 share SPI via manual CS control and a mutex; barometer, LEDs, PWM and ADC use several Arduino wrappers. | Use explicit IDF device handles and Rust drivers with defined ownership/configuration. Keep chip-select arbitration and per-device settings correct. |
| Improve dependency/build reproducibility and feedback | Builds use PlatformIO manifests and Python hooks; host protocol/display harnesses depend on `.pio` downloads. | Pin `Cargo.lock`, the Xtensa toolchain, IDF revision, C components and reference libraries; use host tests, rustfmt and clippy in CI. Cargo does not itself freeze external C downloads. |
| Lower the ongoing cost of control changes | A change to cruise, telemetry or alerts currently spans globals, callbacks, task coordination and simulation models. | Separate pure logic, hardware adapters, and presentation. Typed errors and exhaustive enum matches make future changes easier to check. This benefit comes from the architecture as well as Rust. |

The strongest Rust-specific benefits are ownership, safe cross-core sharing, typed sessions/units, and shared native-testable logic. The strongest Arduino-removal benefits are direct SDK access, fewer Arduino-specific adapters, and independence from Arduino's SDK release cycle. Removing Arduino while keeping C++ could deliver that second group too.

Do **not** justify the migration with promised higher speed, smaller flash, lower RAM/power, or inherently deterministic timing. Those require measurement. Bounds/ownership checks do not validate a throttle rule, CAN payload, radio timeout, ADC calibration, or physical device. IDF, LVGL, retained C codecs and FFI remain outside Rust's safe-code guarantees.

## Complete feature and replacement inventory

Paths in this table are relative to the repo. Source behavior takes precedence over stale README/comments.

| Area and current sources | Rust destination / dependency | Compatibility requirements |
| --- | --- | --- |
| Startup, tasks, state, button: `src/sp140/main.cpp`, `device_state.h`, `mode.cpp` | `controller-core` plus `firmware-sp140` boot/tasks | Phased initialization, idle motor at boot, click-then-hold arm/disarm, cruise, boot/performance mode, 10-second pairing gesture, 500 ms rearm cooldown, update/relay arming interlocks. |
| Throttle: `throttle.cpp`, `throttle.h`, `shared-config.h` | Pure Rust fixed-size filter and state; IDF ADC adapter | Raw 12-bit GPIO7 ADC, inherited attenuation characteristics, integer mapping and averaging, 8 samples, chill/sport ramps and caps, cruise capture/cap/override and reset semantics. No calibration change. |
| ESC: `esc.cpp`, `esc.h`, pinned SINE-ESC-CAN | IDF TWAI adapter + Rust ESC owner + pinned C libcanard/codecs initially | TWAI RX3/TX2 at 1 Mbps, DroneCAN v0, normal node 0x01 / ESC 0x20, transfer IDs/CRC/reassembly, 0.1 microsecond command units, telemetry/error mapping, runtime wrap, LED patterns, motor beeps, bus-off recovery. Cyphal/UAVCAN v1 is not a replacement. |
| BMS: `bms.cpp`, pinned ANT-BMS-CAN | Rust ANT parser and MCP2515 `SpiDevice` driver | CS5, 250 kbps, current driver assumes 16 MHz oscillator, SPI mode 0 at 10 MHz. Preserve Type A/B cell byte-order detection, 24 cells, disconnected probes/NaN policy, flags, identity, cycle/energy counters, coherent startup latch and ESC battery fallback. |
| Altitude: `altimeter.cpp` | Rust BMP3xx driver verified against fitted sensor, or Bosch C compensation behind Rust I2C callbacks | I2C SDA44/SCL41; active GPIO42/40 setup, retries/delays, 25 Hz ODR, 2x temperature/4x pressure oversampling, IIR15; pressure units, ground zero, last-good behavior, 50-sample vario. GPIO9 helper in main has no caller. |
| UI: `lvgl/*.cpp`, `lv_conf.h`, screenshot tests/assets | Rust UI module over exact LVGL 9.5.0 bindings + Rust ST7735 flush driver | ST7735 BLACKTAB, 160x128 RGB565, rotations 1/3, fonts, light/dark themes, splash, layout, units, caching, flashes, alert priority/counters/rotation and skipped-flush repaint. |
| Monitoring: `simple_monitor.cpp`, `esc_monitors.cpp`, `bms_monitors.cpp`, `system_monitors.cpp`, `monitor_config.h`, `alert_display.cpp` | Pure Rust monitors and alert aggregator; UI consumes snapshots | Preserve all SensorID ordinals and category-relative alert bits, thresholds, hysteresis, debounce, startup grace, link establishment/clear windows, logger events, simultaneous warning/critical display and haptic cues. |
| Audio/haptics/LED: `buzzer.cpp`, `vibration_pwm.cpp`, `led.cpp`, ESC output helpers | IDF LEDC and RMT adapters, Rust-owned fixed-size requests | GPIO8 buzzer / GPIO46 vibration / GPIO21 GRB 800 kHz pixel; notes, patterns, intensity, timings, queue behavior. Buzzer and vibration need independently configurable LEDC timers, not just different channels. |
| BLE: `ble/*.cpp`, `ble_ids.h` | `esp32-nimble` candidate over IDF NimBLE, local adapter for uncovered APIs | Same advertising identity, service/characteristic UUIDs, permissions, CCCDs, read/write/notify bytes, bonding, whitelist/RPA reconnection, 60-second pairing window, link tuning, notification pacing and pairing UI. |
| FastLink: `structs.h`, `ble/fastlink_service.cpp`, `docs/fastlink-v4.md` | Pure Rust codec | Version 4, exactly 230 bytes, unchanged 198-byte prefix, alert masks at 198/206/214/222, same scaling/saturation/rounding/sentinels, packet counters, approximately 50 Hz BLE and 1 Hz keepalive during transfers. |
| Controller OTA: `ble/ota_service.cpp` | Pure Rust OTA protocol state + IDF OTA writer | Existing BLE OTA command/sector/CRC/ACK protocol, armed exclusion, duplicate-start cleanup, disconnect abort, 30-second timeout, image-length validation, inactive-slot writes and first-boot validity/rollback. |
| ESC settings: `esc_config_relay.cpp`, `esc_param_ids.h`, config BLE callbacks | Rust relay state machine driven by the single ESC owner | Read-all, scalar/array writes, batched operations, unlock interaction, node switching to 0x40, save/reboot/read-back verification, timeout/retries and all result codes/notify framing. |
| ESC firmware: `esc_flasher_relay.cpp` | Rust upload/session + CAN flash state machine | Same 32-byte image header/hardware validation, 256 KiB limit, 256-byte CAN chunks, next-index flow control, 150 ms request timeout/10 retries, 700 ms restart wait, 15-second upload timeout, exclusive disarmed sessions. |
| Wireless: `wireless_protocol.cpp`, `wireless_settings.cpp`, `espnow_bridge.cpp`, `WIRELESS_CONTROL.md` | Pure Rust packets/authority state + IDF ESP-NOW/Wi-Fi adapter | All three roles, encrypted peers and existing derivation, fixed channel, source filtering, 26-byte control / 24-byte ACK, CRC/sequence/session logic, authoritative receiver, freshness/failsafe latch, mirrored 230-byte telemetry. Remote must not initialize/transmit local CAN. |
| Settings + USB: `device_settings.cpp` | IDF NVS + bounded JSON handler over actual USB serial interface | `openppg` keys and exact u8/u16/i32/blob types, defaults/sanitization, armed-time minutes, timezone; existing JSON names and newline/CR/overflow behavior; working `reboot`, `sync`, `diag_sync`, `diag_clear`, settings and wireless updates. Stale `rbl` documentation is not an implemented command. |
| Diagnostics: `diagnostics.cpp`, `diagnostics.h`, `tools/usb_diag_logger.py` | Rust diagnostics + narrow IDF reset/RTC/core-dump adapter | `openppg_diag` binary records, 16-entry history, RTC heartbeat, planned-reset classification, JSON schema, ELF crash evidence and matching ELF availability. Do not persist Rust-default struct layouts. |
| Firmware metadata/build: `fw_marker.cpp`, `version.h`, `extra_script.py`, `merge-esp.py` | Rust retained static + build metadata/export tooling | Preserve 14-byte `OPPGFWV1` marker with u8/u8/u32 little-endian version/build, ensure linker retention, app descriptor, Git identification, OTA app-only image and distinct full recovery image. |
| Host lab: `simulator/`, `tools/host-can-protocol-probe/`, screenshot harness | Rust simulator using production core; old code remains reference oracle initially | Preserve scenario CSV, JSON/JSONL, fault injection, protocol fixtures, framebuffer output and browser lab behavior. Browser React/JS and Python support tools can stay in their present languages; their rewrite is not needed to compile device firmware from Rust. |

Pinned protocol references: SINE-ESC-CAN `2ab56a4e5b52e4456317c8ee3e3d802b232c6148`, ANT-BMS-CAN `fd54852bc6f1c9608e37af9ca7c13ea4135c095b` (`platformio.ini`). Preserve licenses and source provenance when extracting C components. Reference fixtures must use those revisions, not whichever library versions happen to be cached.

## Architecture

Use a workspace under `rust/` during migration; keep current firmware buildable until parity is demonstrated:

```text
rust/
  Cargo.toml / Cargo.lock / rust-toolchain.toml / .cargo/config.toml
  crates/controller-core/       # no_std, hardware independent; control, button, monitoring, effects
  crates/controller-protocols/  # no_std; explicit FastLink/wireless/ANT/BLE/relay byte codecs
  crates/controller-ui/         # Rust screen creation/update; one UI owner
  crates/platform-espidf/       # IDF task/peripheral/radio/NVS/OTA/USB adapters
  crates/lvgl9-sys/             # optional local exact-version binding boundary
  crates/dronecan-sys/          # pinned C transport + generated codecs, safe Rust facade
  apps/firmware-sp140/          # composition/boot, ESP32-S3 target only
  apps/controller-sim/          # native target, same production core
  components/                  # pinned C components and minimal inline/macro FFI shims
  tests/fixtures/              # C++ reference inputs/outputs, no live keys or private dumps
```

Keep MCU dependencies out of the native core/simulator dependency graph. Set native default-members; select the firmware target explicitly. Host `cargo test` must not try to compile ESP-IDF. A `no_std` core does not mean a `no_std` firmware; the device application can use `std` on IDF.

The core exposes event/time/snapshot inputs and fixed-capacity outputs/effects. Represent disarmed/armed/cruise and maintenance sessions explicitly; serialize arm/update decisions through the same authority. Use owned data rather than queueing pointers to temporary melodies/images. Keep side effects such as NVS writes, I2C, serial logging, BLE notification and audio outside the 50 Hz critical path.

One task owns ESC transport and its relay state machines. Other tasks submit bounded requests. One task owns LVGL objects and invokes all LVGL APIs; do not mark LVGL handles Send/Sync. The UI owner consumes immutable telemetry/alert snapshots. SPI devices share an explicitly managed bus, with per-device settings, bounded arbitration and LCD transfer chunks; BMS polling must not starve behind rendering. Keep I2C work out of LVGL locks.

Use configured IDF-backed Rust threads where suitable, retaining core affinity/priority using `ThreadSpawnConfiguration`, or a narrow FreeRTOS adapter for exact periodic waits. Deadline-based scheduling must match `vTaskDelayUntil`; `sleep(20 ms)` after each iteration adds work time to the period. Use bounded CAN servicing per control cycle, not an unbounded receive loop. Instrument deadline misses and queue saturation.

Default to safe Rust. Isolate `unsafe` to audited sys wrappers with documented pointer lifetimes, callbacks, buffer ownership and concurrency. Do not unwind across C callbacks. Propagate recoverable peripheral errors as Results; validate panic/abort integration against IDF crash capture rather than assuming Rust panics automatically produce existing diagnostics.

### Timing and behavior contracts to freeze

| Contract | Current value / evidence |
| --- | --- |
| Throttle/ESC owner | 20 ms / 50 Hz, core 0, priority 3 (`main.cpp:298`, `:781`) |
| Watchdog worker | 100 ms, core 0, priority 3; five-second task WDT subscribes throttle + watchdog only; idle-task checks disabled (`main.cpp:273`, `:720`, `sdkconfig.defaults`) |
| Button | 10 ms sampling, 50 ms debounce; initial click <500 ms, next press within 1500 ms, hold 2000 ms; disarmed performance hold 3000 ms, pairing hold 10000 ms (`main.cpp:79`, `:1007`) |
| UI | 33 ms task on core 1; LVGL refresh setting 30 ms (`main.cpp:620`, `lv_conf.h:41`) |
| BMS/controller sensors/monitors | 100 ms; BMS on core 1 (`main.cpp:328`, `:609`, `:631`) |
| BLE publisher | 20 ms on core 1; actual over-air delivery depends on negotiated link; transfer keepalive approximately 1 Hz (`main.cpp:465`, `fastlink_service.cpp:156`) |
| ESP-NOW | Control 50 Hz; ACK limited to 40 ms / 25 Hz, telemetry to 100 ms / 10 Hz (`espnow_bridge.cpp:382`, `:409`) |
| ESC stale / health | 50 ms telemetry freshness; 250 ms TWAI health/recovery poll; ESC's own command-loss stop approximately 300 ms (`esc.cpp:14`, `:321`) |
| BMS link | 500 ms; coherent snapshot requirement latches after first complete startup data (`main.cpp:631`) |
| Alerts | BMS first-connect grace 2000 ms once per boot; CAN establish/clear 1000 ms and loss debounce 500 ms (`monitor_config.h`) |
| Wireless | Control invalid when age >150 ms, immediate control-path disarm on detection; remote status >300 ms and telemetry >500 ms (`main.cpp:350`, `:1333`, `espnow_bridge.h`) |

Freeze comparison operators and scheduling quantization, not just numbers. For example, checking `age >150 ms` on a 20 ms cycle does not guarantee a physical reaction at exactly 150 ms. Match the baseline and measure actual worst-case latency. Preserve 32-bit wrap semantics for uptime/session sequences and 16-bit ESC runtime rollover.

### Compatibility traps requiring explicit decisions/tests

1. **BLE stack configuration and bonds.** Current defaults enable Bluedroid and disable IDF NimBLE because NimBLE-Arduino supplies its own implementation. The Rust [esp32-nimble configuration](https://github.com/taks/esp32-nimble) requires IDF NimBLE enabled and Bluedroid disabled. Reusing the NVS partition does not establish bond-store compatibility. Prove existing-phone reconnection, CCCDs and identity/RPA behavior across upgrade and rollback; if one-time re-pairing is necessary, it is a user-visible deviation requiring an explicit rollout decision, not silent parity.
2. **Time width.** `ble/config_service.cpp:136` sends `sizeof(time_t)` bytes. [esp-idf-sys build options](https://github.com/esp-rs/esp-idf/blob/master/esp-idf-sys/BUILD-OPTIONS.md) document 64-bit time for IDF 5+, versus earlier releases. Keep the observed legacy BLE width/endian explicitly while using the new SDK's native time representation internally. Configure Rust's time ABI correctly for the selected IDF.
3. **Throttle GATT asymmetry.** `config_service.cpp:107` reads u16 raw ADC in native S3 little endian; `:114` writes two-byte big-endian PWM, accepted only in cruise; `:589` notifies four-byte int values. Preserve each path until a separately versioned client migration.
4. **LVGL bindings.** The established [lvgl Rust binding releases](https://github.com/lvgl/lv_binding_rust/releases) describe LVGL 8.3.5. Do not downgrade this 9.5 application. Evaluate [lightvgl-sys 9.5.x](https://docs.rs/crate/lightvgl-sys/9.5.3), whose compatibility table maps to LVGL 9.5.0, or generate local bindings against exact headers/configuration. Test Xtensa bindgen/toolchain settings and static-inline/macro wrappers before committing to a binding.
5. **Flash layout.** Active table is `default_8MB.csv`, not `partitions.csv`: NVS 0x9000/0x5000, OTA metadata 0xe000/0x2000, app0 0x10000/0x330000, app1 0x340000/0x330000, SPIFFS 0x670000/0x180000, coredump 0x7f0000/0x10000. Use the same table for app build metadata, bootloader and flashing. Resolve existing QIO/DIO configuration inconsistencies against the known-good hardware image, not by copying contradictory defaults.
6. **USB transport.** `USBSerial` must keep working with WebSerial and diagnostic tooling. Determine actual native USB Serial/JTAG versus CDC behavior from the existing board/core and bench enumeration. Rust `println!` to UART is not a replacement for USB receive/transmit semantics.
7. **RAM.** LVGL reserves a 48 KiB heap and two 20,480-byte buffers; tasks/radio consume additional RAM. ESC flashing can allocate up to 256 KiB, first trying PSRAM then internal memory. This configuration does not establish PSRAM availability. Measure largest free block as well as free heap and prove full supported-image upload with BLE/UI active. 8 MB flash does not solve a RAM shortfall.
8. **NVS binary types.** `openppg` sea pressure is a four-byte float blob, booleans are u8, armed time u16 and timezone i32 (`device_settings.cpp:195`). `openppg-radio` holds radio configuration. `openppg_diag` stores packed diagnostic records. Use compatible raw NVS where wrapper encoding differs; do not replace these with a convenient incompatible serializer.
9. **Relay protocols and pacing.** Config DATA notifications contain type + offset u16 LE + total u16 LE + up to 198 bytes, despite an outdated nearby comment. Keep three stream passes/four terminal-status resends (`config_service.cpp:611`). Transfers quiet normal ESC requests while continuing CAN pumping (`main.cpp:1395`). Retain the 1 Hz phone keepalive and post-result settle behavior.
10. **Simulator fidelity.** The native simulator reuses real throttle/CAN code but models other controller behavior. QEMU is not full hardware validation for this repo's USB, SPI/I2C, PWM and radio paths. Preserve it as a supplementary tool; do not let a simulator-only pass establish real-time or RF parity.

## Implementation milestones and gates

The commands below distinguish **existing baseline commands** from **proposed Rust commands/interfaces to implement**. They were inspected, not executed in this assessment. Do not report them as green until run on the frozen baseline/new implementation. Adapt native target selection to the host; do not hardcode a macOS target into shared CI.

### 0. Freeze behavior and create compatibility fixtures

Scope: reference tests/fixture tooling, source provenance manifest, CI baseline; no live throttle behavior changes.

- Preserve a reproducible snapshot of this dirty tree and pinned protocol libraries before splitting work. Include the wireless additions and simulator. Compare baseline checksums before edits.
- In a dependency-prepared checkout run the baseline commands below and retain failures as failures. This worktree's hardcoded `.pio` references must be populated before the simulator/probe can run; private dependency access may be required.
- Extract byte-level golden fixtures from the exact C++ implementations for FastLink, ANT variants, DroneCAN requests/responses, every GATT characteristic, OTA commands/sectors, relay results and wireless packets. Store sanitized synthetic NVS fixtures with type metadata. Never commit live BLE bonds/radio secrets.
- Capture production state-machine behavior beyond the simulator: button gestures, cooldown, cruise transition/ramp/reset boundaries, maintenance interlocks, alert transitions, session/failsafe recovery and rollover. Existing `test_esc_errors` is a placeholder; test count is not coverage of the ESC pipeline.
- Record the GATT table and actual USB enumeration, image metadata, heap/largest-block/stack measurements and real timing traces from a baseline device.

Existing verification commands:

```sh
pio run -e OpenPPG-CESP32S3-CAN-SP140
pio test -e native-test
./simulator/run.sh --test-only
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol
cmake --build build-host-can-protocol --parallel
./build-host-can-protocol/host_can_protocol_probe
./test/test_screenshots/build_and_run.sh --local
# Full existing sequence, including merged image:
./simulator/verify.sh
```

Expected: successful baseline build/tests with archived results, or a named pre-existing failure with a reproduction. Screenshot reference creation is not a passing comparison; require existing references and fail on missing references. Old CLAUDE.md testing descriptions are stale; CI and current manifests define the real commands.

### 1. Prove the Rust platform before porting the application

Scope: new `rust/` workspace, platform adapters, minimal app, exact-version C dependencies, bench harness. Keep existing source/build intact.

- Install toolchain using [Espressif's Xtensa instructions](https://docs.espressif.com/projects/rust/book/getting-started/toolchain.html). Bootstrap the app from the [ESP-IDF Rust template](https://github.com/esp-rs/esp-idf-template). Select the native Cargo/IDF builder, not its PlatformIO fallback.
- Pin a complete known-compiling tuple of Xtensa Rust toolchain, IDF release/commit, `esp-idf-svc`/HAL/sys, `esp32-nimble`, bindgen/libclang, LVGL and flashing tools. Current upstream `esp32-nimble` 0.13.0 depends on svc 0.53.0; these are candidates, not a locally proven version matrix. Do not let implicit SDK defaults choose the release. An IDF 5.x candidate minimizes avoidable migration distance, subject to support/lifecycle and build proof.
- Bring up USB receive/transmit, raw ADC, both CAN interfaces, SPI devices, BMP3xx, PWM/RMT, minimal LVGL, NVS and crash/panic behavior.
- Prove BLE plus encrypted ESP-NOW coexistence and correct core affinity, old-phone bond migration or identify the incompatibility, same-partition boot/OTA/rollback across the actual old bootloader, and peak ESC image buffer allocation. These risks must be resolved early.
- Recreate defaults deliberately: 240 MHz, 1 kHz RTOS ticks, correct flash, task watchdog scope, rollback, coredumps and coexistence. Do not copy Arduino/autostart/Bluetooth Kconfig wholesale. SDK API changes need explicit adapters.

Proposed verification (after implementing this workspace):

```sh
cd rust
cargo +esp build --locked --release -p firmware-sp140 --target xtensa-esp32s3-espidf
cargo tree -p firmware-sp140
```

Expected: S3 ELF and boot artifacts built without Arduino libraries. Bench results must show USB/ADC/both CAN/radio/flash proof and an explicit RAM budget. Do not implement the rest against an unproven platform.

### 2. Port the shared controller core and byte codecs

Scope: `controller-core`, `controller-protocols`, native tests and reference adapters.

- Start with `throttle.cpp` arithmetic and its substantive tests, then main/button/cruise rules, ANT parsing, wireless authority, telemetry conversion, monitoring/alerts and maintenance protocol states.
- Match integer truncation, floating-point precision/rounding, NaN/sentinels and exact threshold comparisons. Use fixed-width storage; never use Rust `usize`, default enum layout, or host `unsigned long` equivalents in persisted/on-wire data.
- Separate packet parsing from effect execution. Implement explicit little/big endian fields, length checks, CRCs and state/error enums. Do not use generic Serde formats for existing binary protocols.
- Add differential tests and boundary/property tests. Exhaustively compare all 4096 raw throttle values in both modes, ramp sequences, 8-sample rollover, reset and cruise transitions. Compare full packet bytes, including alert mask offsets and negative/missing temperatures.
- Characterize ESC motor-temperature validity (`> -20 C && <= 140 C`, otherwise NaN), repeated ESC timestamp rejection, 16-bit runtime rollover, and accumulated-runtime reset after disconnect/reconnect (`inc/sp140/esc.h:7`, `src/sp140/esc.cpp:349`).

Proposed verification:

```sh
cd rust
cargo +stable test --locked -p controller-core -p controller-protocols
cargo +stable clippy -p controller-core -p controller-protocols --all-targets -- -D warnings
cargo +stable fmt --all -- --check
```

Expected: native tests pass with zero unexplained C++/Rust fixture differences. Hardware-independent core contains no Arduino/IDF dependency or unsafe application state access.

### 3. Integrate real control, CAN, sensors and feedback

Scope: firmware boot/tasks, platform adapters, Rust CAN orchestration, audio/haptics/LED, hardware tests.

- Move the ESC owner, throttle control and watchdog into configured Rust tasks. Preserve single-owner config/flash routing, TWAI recovery and startup/disconnect behavior.
- Use the pinned C libcanard and generated codecs initially; remove their Arduino-dependent C++ wrappers. Preserve transfer cleanup, retransmission behavior and pool requirements. Pure Rust replacement is a separate fixture-gated follow-up, not a prerequisite for removing Arduino.
- Implement ANT parser/MCP2515 and shared SPI arbitration. Validate actual 16 MHz oscillator and register timing against the board. Port BMP compensation/settings and cache/vario semantics, ADC transfer characteristics and independent PWM timer usage.
- Keep state transitions immediate and slow effects deferred. New ownership must not make a disarm wait behind a full maintenance/log queue. Reproduce existing role-specific suppression of unused CAN and sensor producers.

Verification: previous native tests + proposed firmware build command. Bench trace must show 20 ms command cadence under UI/radio/bus load, unchanged ADC-to-command response, exact disconnect/restore timing, actual ESC idle on control interruption, successful bus-off recovery and watchdog reset/crash evidence. UI/BMS/BLE task stalls alone must not acquire new task-WDT subscriptions. Archive measured maxima/deadline misses; do not infer timing from average CPU load.

### 4. Port persistence, USB, BLE, wireless and both update flows

Scope: Rust service handlers/adapters and core maintenance state; interoperability fixtures and phone/USB bench tests.

- Preserve the complete UUID/property/encoding inventory and advertising/security/reconnection user flows. Handle notifications/CCCDs, MTU negotiation, result streaming, pairing controls and phone keepalive explicitly.
- Keep the existing WebSerial framing/schema and command set; use bounded buffering and recovery after oversized/malformed lines. Do not confuse old `rbl` comments with current `reboot` implementation.
- Read/write all existing NVS representations, diagnostic history and radio settings without erasing user state. Preserve settings on upgrade and rollback; test binary sizes/schema and compiler ABI assumptions.
- Implement controller BLE OTA using the existing 20-byte command and 4096-byte sector/CRC/ACK protocol, with IDF inactive-slot writer and one session owner. Preserve all abort/interlock/timeout/length cases.
- Port the entire ESC parameter and firmware relay sequences, including save/reboot/read-back verification, result retransmission, chunk flow control and RAM allocation failure behavior.
- Complete wireless standalone/receiver/remote modes, stale-control disarm, fresh DISARMED handshake after loss/restart, old sequence/session rejection, application ACK authority, independent stale-status/telemetry policies and encrypted interoperability with old firmware peers.

Verification: native byte fixtures + firmware build, then actual existing phone app and config tool. Cover read-all/batch write/save/reboot/verify; full supported ESC flash; successful controller OTA plus disconnect, duplicate START, bad CRC/length, timeouts, power interruption and first-boot rollback. Test both controller images in both OTA slots, upgrades and reversions, and two-controller mixed old/new firmware in each role. A re-pair requirement or external client change is an unresolved compatibility deviation until explicitly agreed.

### 5. Port screen logic and connect the simulator to the production core

Scope: Rust UI over LVGL 9.5, framebuffer harness, native simulator backend/CLI, browser-lab adapter where needed.

- Translate all active screen construction/update/alert logic to Rust, retaining exact LVGL configuration/fonts/assets and safe object lifetimes. Retain two buffers initially; optimize only after parity.
- Run screenshot comparisons against existing reference BMPs. Add timed snapshots for flashing, alert rotation, theme/rotation changes, pairing, stale links and skipped-flush recovery. Use deterministic virtual time.
- Replace the separate simulator controller model with production Rust core/monitors/codecs. Keep the old implementation as a reference during comparison, then preserve the existing scenario/JSON/JSONL/framebuffer interface for the browser lab. Keep physical CAN/fault models useful without requiring Arduino in the new runner.

Proposed verification:

```sh
cd rust
cargo +stable test --locked -p controller-sim
cargo +stable run --locked -p controller-sim -- \
  --scenario ../simulator/scenarios/nominal-flight.csv --duration 10000
```

Expected: current scenarios pass; reference state/CAN outputs match or differences are investigated against production source, since parts of the old simulator are models. Screenshot tests fail on pixel differences/missing references. Existing browser lab remains operational against the Rust backend. Simulator success is not hardware/RF proof.

### 6. Complete release verification, then remove the legacy production path

Scope: build/release CI, export tooling, documentation, legacy source/dependency cleanup after gates pass.

- Reproduce firmware version marker and build metadata in a retained Rust static. Test byte scanning of the exported app-only `.bin` against the existing backend format. LTO/section GC must not remove it.
- Export app-only OTA and full recovery images separately, using the same custom partition table and selected bootloader. Verify app size <=0x330000 and no partition overlaps. Preserve NVS by default when flashing/upgrading; never copy an example runner that erases OTA data or NVS into the routine workflow.
- Run host tests/fmt/clippy, S3 build, packet differential tests, screenshot parity, native lab scenarios and the hardware matrix in CI/bench workflows. Archive ELF/map/version for crash decoding and load/RAM/stack evidence.
- Require explicit successful evidence for every row of the inventory before deleting the corresponding old implementation. Then remove PlatformIO/Arduino from the production build and dependency graph. Keep any needed legacy reference tests in a clearly isolated test-only harness until replacements cover them.
- Update README/build/recovery instructions and simulator tooling. New firmware build/test dependencies must not rely on `.pio` caches or the original developer's absolute paths.

Final automated commands/interfaces to provide:

```sh
cd rust
cargo +stable test --locked                    # native default-members only
cargo +stable fmt --all -- --check
cargo +stable clippy --all-targets -- -D warnings
cargo +esp build --locked --release -p firmware-sp140 --target xtensa-esp32s3-espidf
```

Add a release check script that verifies marker bytes, target/image metadata, partition table and slot size, production link map exclusions, and expected artifacts. `cargo tree` alone cannot prove absence of C/Arduino objects. Do not claim a no-Arduino build solely because `.rs` files exist or Cargo is the entry point.

## Done criteria

- [ ] A clean checkout builds the ESP32-S3 application through Cargo with pinned tools/dependencies and no Arduino SDK/library objects linked.
- [ ] Controller, service, UI and driver orchestration code is Rust; remaining C dependencies are inventoried (IDF, LVGL, initial DroneCAN/codecs and any justified sensor API).
- [ ] All inventory rows have host/bench evidence, including both CAN buses, all wireless roles, USB, BLE configuration, controller OTA, ESC config/flashing and crash diagnostics.
- [ ] Existing clients/peers and stored data remain compatible; any agreed deviations are explicitly documented/versioned.
- [ ] Native tests exercise the same production core rather than a separately rewritten controller model.
- [ ] UI reference images and timed behaviors pass; browser simulator/lab still works.
- [ ] Worst-case control timing, ADC response, radio coexistence, queue pressure, free/largest heap block, stack watermarks and full ESC image upload have measured evidence.
- [ ] Old-bootloader upgrade, both OTA slots, rollback, interrupted transfer and recovery-image use are demonstrated with NVS preserved.
- [ ] Version marker, OTA app-only `.bin`, full recovery image, ELF and map artifacts are produced and validated.
- [ ] README and the advisor index reflect the actual completed state.

## Stop conditions and unresolved feasibility questions

Pause the affected milestone and report the concrete incompatibility if:

- Source/dirty-tree baseline or pinned private library revisions cannot be recovered.
- Chosen Rust toolchain/IDF/BLE/LVGL tuple cannot compile or coexist on the actual board.
- A proposed driver silently changes ADC range, BMP compensation, MCP oscillator/timing, CAN protocol generation or on-wire encoding.
- Supporting current maximum ESC image size exceeds available memory. Do not quietly shrink the supported image limit or assume PSRAM.
- An old bootloader cannot boot/rollback the selected IDF image with the current partition layout. Do not silently require a fleet-wide full erase/USB reflash.
- Bond reuse, GATT behavior or existing app interoperability cannot be preserved. Report whether a one-time re-pair or client update would be needed.
- Measured throttle/radio deadlines regress, a maintenance action blocks/disrupts flight control, or watchdog scope changes.
- A requirement depends on the absent factory/QC branch, an external phone/backend implementation, or unavailable hardware. Record that boundary rather than inventing behavior.

## Effort and sequencing

Planning estimate, not a quote: roughly **10–16 focused engineer-weeks plus hardware soak/release time** for an engineer comfortable with Rust and this controller. Characterization/platform proof and CAN/BLE/OTA interoperability dominate uncertainty; ownership cleanup and translating pure arithmetic are easier. A no-IDF/no-C version or missing external client/specification access can take substantially longer. Re-estimate after milestones 0–1.

The best first implementation slice is **baseline fixtures + native Rust throttle/control/protocol tests + an Arduino-free S3 platform spike**. Do the USB/radio/bond/bootloader/RAM experiments early. Porting only throttle and declaring the controller migrated would leave most compatibility risk unresolved.

## Maintenance notes

- Preserve explicit protocol versions, SensorID bit assignments, native-vs-wire time widths and NVS storage types when changing features later.
- Upgrade toolchain, IDF, radio and LVGL separately with the same interoperability/timing matrix; do not float them together.
- Keep FFI adapters small and review callback lifetimes, C allocator ownership, thread constraints, static buffers and panic paths on every dependency update.
- Treat release image/partition/bootloader compatibility as a versioned product interface. Normal application OTA must not assume it installs the new bootloader's rollback configuration.
- Security vulnerability hunting, dependency CVE auditing, external mobile/backend code review, and electrical/flight qualification were outside this migration assessment. No flight-readiness claim is made by the source review or proposed host tests.
