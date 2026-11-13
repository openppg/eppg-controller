# OpenPPG controller hardware lab

The lab has complementary native functional and ESP32-S3 instruction-level
execution paths. Neither is an RF/electrical model. The native Rust branch uses
the firmware's actual controller, monitors, codecs, ESC sessions and LVGL UI;
the Arduino-removal branch retains the C++ reference controller model plus the
actual migrated throttle, wire libraries and production C++ UI. QEMU runs the
actual complete firmware image and FreeRTOS tasks with explicit peripheral
models where the S3 machine lacks hardware support.

## Native tests and simulator

No Arduino SDK or PlatformIO is needed. Install a C/C++ compiler and CMake.
The pinned CAN codecs and LVGL are vendored in `components/`; original test-only
CAN wrappers live in `simulator/reference-can/` to provide an independent oracle.

```sh
cmake -S tools/host-native-tests -B build-host-native-tests
cmake --build build-host-native-tests --parallel
ctest --test-dir build-host-native-tests --output-on-failure
./simulator/run.sh --native --test-only
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol
cmake --build build-host-can-protocol --parallel
./build-host-can-protocol/host_can_protocol_probe
cmake -S tools/host-screenshot-offline -B build-host-screenshots
cmake --build build-host-screenshots --parallel
ctest --test-dir build-host-screenshots --output-on-failure
```

GoogleTest is fetched at its pinned version if missing. Set `GTEST_DIR` to an
existing source tree for an offline build. Source paths can be supplied explicitly
with `PROJECT_ROOT`, `SINE_DIR`, `BMS_DIR`, `DRONECAN_DIR` and `LVGL_DIR`; no
implicit user-specific path is used. The pre-migration baseline can use
`EPPG_FIRMWARE_DEPS` pointing to its existing library cache.

On the Rust branch, install native Rust and libclang, then:

```sh
./simulator/run.sh --rust --test-only
./simulator/run.sh --rust simulator/scenarios/nominal-flight.csv 10000
cd rust
cargo +stable run --locked -p controller-sim -- \
  --scenario ../simulator/scenarios/nominal-flight.csv --duration 10000 \
  --trace /tmp/flight.jsonl --framebuffer /tmp/flight.bmp
```

`run.sh` automatically chooses Rust when that workspace is present; `--native`
always selects the C++ reference path. Scenario CSV rows are
`time_ms,action,value`. Events must be ordered. Existing throttle/mode/arm/cruise,
altitude, sensor values, bus loss/corruption/bus-off and task-stall injections
remain available. Sample scenarios wait until 600 ms before arm, matching the
production boot/rearm cooldown. The Rust runner also supports actual button
edges, maintenance exclusion and wireless session packets; see
`rust/apps/controller-sim/README.md` on that branch.

`simulator/verify.sh --host-only` runs native regression, controller/device,
wire-protocol, screenshot and HTTP-lab tests. Add `--rust` to include the Rust
production-core tests, strict lint and renderer. Without `--host-only` it also builds
and validates the selected real ESP32-S3 firmware; source ESP-IDF's environment
and the Rust Xtensa environment for `--rust` first. `--qemu` additionally builds/runs
that selected application's QEMU variant; `--rust --qemu` verifies Rust control,
CAN/display progress and full-size flash staging. Set `QEMU_BIN` or install the
pinned binary with `bash simulator/tools/install_qemu.sh` first. The checks fail
on missing screenshot goldens rather than creating
new expectations silently.

## Interactive browser lab

Build the native simulator and framebuffer renderer above, or build
`controller-sim` on the Rust branch, then:

```sh
cd simulator/ui
npm ci
npm start
```

Open `http://127.0.0.1:4174/`. The lab runs a new deterministic simulation per
request and renders live 160x128 controller pixels after every input. The Rust
backend renders the production Rust LVGL implementation directly. The native
backend invokes the C++ production UI framebuffer renderer. Both cross real
DroneCAN/ANT byte parsers before telemetry reaches the screen.

The server automatically selects an existing Rust binary, otherwise the native
reference. Set `EPPG_SIM_BACKEND=rust|native` to choose explicitly. Advanced
source-comparison tests may set `EPPG_SIMULATOR_BINARY` and
`EPPG_FRAMEBUFFER_BINARY`; normal usage needs neither. `npm test` verifies the
local API, arm/ramp response, raw CAN fault propagation, input bounds and live
BMP contents. The server binds only loopback. `/api/health` identifies the
selected backend; `/api/simulate` retains the existing events/duration schema.

## ESP32-S3 QEMU

Use Espressif's `qemu-system-xtensa`, not a generic QEMU package. Source the
ESP-IDF v5.5.3 environment, set `QEMU_BIN` if it is not on PATH, then:

```sh
./simulator/tools/run_qemu.sh --build --smoke
./simulator/tools/run_qemu.sh
./simulator/tools/run_qemu.sh --gdb
```

`--build` creates the `EPPG_QEMU` image in `build-idf-qemu`. Its separate SDK
configuration uses QEMU-compatible DIO flash and UART0 console. The physical
board image keeps its own flash/USB settings. The launcher merges the generated
IDF `flasher_args.json`, so bootloader, partition table and application offsets
come from the actual build rather than guessed constants. Each QEMU run owns a
writable copy and preserves the input release image.

`--smoke` is bounded and requires application boot, an armed state with a
fresh throttle task, changing ST7735 pixels, bidirectional CAN frames and
throttle commands above idle. It writes `build-idf-qemu/qemu-smoke.json` with the exact image
hash, UART log, expected/missing markers, individual control checks and pass/fail result. A ROM banner alone
cannot pass. `--image FILE` accepts an already-merged image. `--trace` records
executed instructions; stop the interactive run when enough evidence is captured.

The QEMU peripheral backend decodes actual ST7735 command/pixel SPI traffic,
MCP2515 register operations with finite receive buffers, and BMP3xx register/OTP
calibration data. An ESC node at the raw CAN frame seam runs the same pinned C
libcanard/DSDL codecs. The default input sequence exercises the real button
click/hold task and later ramps raw ADC. The UART evidence includes pixels
written, framebuffer hash, CAN frame counts and command units.

The simulator does not claim that software models reproduce RF propagation,
ADC noise, sensor factory calibration, analog CAN transceivers or exact external
motor/battery physics. QEMU TWAI endpoint wrappers bypass the physical TWAI MMIO
interrupt path; use real hardware for that driver timing. BLE/ESP-NOW application
codecs/state machines can be tested natively; RF is not implemented by this
QEMU machine. See [ARCHITECTURE.md](ARCHITECTURE.md) for the boundaries.
