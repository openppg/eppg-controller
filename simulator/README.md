# OpenPPG controller emulator

This directory is a macOS-local, deterministic controller simulation. It is
the executable hardware-aware layer beneath the existing LVGL screenshot
emulator. It runs the **production `src/sp140/throttle.cpp` pipeline**, the
unmodified SINE ESC/libcanard stack, and the unmodified ANT BMS parser. It also
models a focused subset of the task cadences, state machine, barometer, BLE
telemetry, alerts, and watchdog behavior.

It is intentionally virtual-time driven. A ten-second flight completes in
milliseconds and produces the same result on every run.

## Run

```bash
./simulator/run.sh
./simulator/run.sh simulator/scenarios/can-loss.csv 4000
./simulator/run.sh simulator/scenarios/watchdog.csv 7000
./simulator/run.sh simulator/scenarios/hardware-can-faults.csv 6500
```

Run the complete firmware + native logic + simulator + real CAN protocol +
LVGL framebuffer validation stack with one command:

```bash
./simulator/verify.sh
```

To write one JSON snapshot per simulated UI frame:

```bash
build-simulator/eppg-sim \
  --scenario simulator/scenarios/nominal-flight.csv \
  --duration 10000 --trace /tmp/eppg-flight.jsonl
```

Scenario rows are `time_ms,action,value`. Supported actions are `throttle`
(0..4095), `arm`, `cruise`, `mode` (`chill`/`sport`), `altitude`, `esc_link`,
`bms_link`, `barometer_link`, `soc`, `bms_temp`, `esc_temp`, `esc_rpm`,
`esc_amps`, `esc_voltage`, `bms_voltage`, `bms_current`, `cpu_temp`, and
`stall_throttle`. Wire fault actions are `esc_drop_every`, `bms_drop_every`,
`esc_corrupt_every`, `bms_corrupt_every` (`0` disables; `N` affects each Nth
frame), `esc_bus_off`, and `bms_bus_off`. Events must be ordered by time.

## Hardware-aware CAN model

The simulator has two independent bounded classic-CAN networks:

- ESP32-S3 TWAI to SINE ESC at 1 Mbit/s. Controller throttle commands are real
  DroneCAN service requests. A virtual ESC node decodes them, evolves the motor
  plant, and returns the exact multi-frame `SetThrottleSettings2` service
  response with the request transfer ID. It also emits `BroadcastStatusInfo`;
  the production SINE library performs CRC checking, reassembly, and decode.
- MCP2515 to ANT BMS at 250 kbit/s. The battery plant emits the documented ANT
  extended-ID frames with their real integer scaling and cell endianness; the
  production `BMS_CAN` class is the only path back to controller telemetry.

Each frame receives an on-wire completion time derived from bitrate, extended
CAN overhead, intermission, and bit stuffing. Queues are bounded to 64 frames.
JSON output includes per-bus frames, bytes, drops, corruption, overflow, and
bus-off counts. Clearing ESC bus-off waits 250 ms to match the firmware health
poll/recovery interval. ESC connection freshness is 50 ms; BMS freshness is
500 ms.

## Standalone CAN protocol probe

The transport probe goes below the simulator's decoded telemetry layer. It
compiles the unmodified SINE ESC/DroneCAN and ANT BMS libraries against
in-memory TWAI and MCP2515 queues, then verifies real multi-frame wire payload
serialization, reassembly, and parsing:

```bash
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol
cmake --build build-host-can-protocol --parallel
./build-host-can-protocol/host_can_protocol_probe
```

The main simulator now uses this same protocol seam continuously. The small
probe remains useful as a fast isolated vendor-library check.

## Firmware bytecode

The real ESP32-S3 build remains authoritative. Build it and inspect its ELF:

```bash
pio run -e OpenPPG-CESP32S3-CAN-SP140
python3 simulator/tools/inspect_firmware.py
python3 simulator/tools/inspect_firmware.py \
  --disassembly /tmp/eppg-controller.disassembly.txt
```

The inspector reports the SHA-256, Xtensa sections/addresses and important
firmware task symbols. The optional output is a complete source-interleaved
Xtensa disassembly.

Espressif's instruction-level QEMU can also consume the merged flash image:

```bash
QEMU_BIN=/path/to/qemu-system-xtensa ./simulator/tools/run_qemu.sh
./simulator/tools/run_qemu.sh --gdb
./simulator/tools/run_qemu.sh --trace
```

The launcher deliberately does not auto-download a toolchain. Espressif
documents macOS arm64 binaries and the `esp32s3` machine here. It checks
`QEMU_BIN`, `PATH`, then the workspace-local `.tools/qemu/bin` installation:

Use `Ctrl-A`, then `x`, to exit the serial/monitor session. `--gdb` waits for
GDB on port 1234; `--trace` writes `qemu-instructions.log`.

- [ESP-IDF ESP32-S3 QEMU guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/qemu.html)
- [ESP32-S3 QEMU machine/flash guide](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/esp32s3/README.md)
- [QEMU peripheral support matrix](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/README.md)

## Fidelity boundary

This is a hardware-aware functional controller model, not full or cycle-accurate
ESP32-S3 emulation. Selected task periods and safety timing use firmware
constants, but macOS does not reproduce Xtensa instruction timing, dual-core
race behavior, ADC noise spectra, CAN arbitration between simultaneous nodes,
transceiver analog behavior, RF/BLE PHY behavior, or sensor electrical faults.
CAN timing is frame-level and bitrate-aware rather than sampled-bit-level. Use
the native simulator for fast controller behavior and repeatable fault
scenarios; use the built ELF/disassembly and real hardware-in-loop tests for
instruction, interrupt-latency, and electrical timing claims.

The instruction-level tier is complementary. Espressif's current ESP32-S3
QEMU supports both CPU cores, memory/flash, interrupts, SysTimer/timer groups,
GDMA and TWAI/CAN. Its published matrix does not support the controller's
Bluetooth, Wi-Fi/ESP-NOW, general-purpose SPI, I2C, LEDC, USB, or GPIO
matrix/IOMUX. The unmodified image therefore cannot complete a faithful
`setup()` without new QEMU devices or conditional firmware shims.

The existing display path remains available separately:

```bash
./test/test_screenshots/build_and_run.sh --local
```

## Interactive hardware lab

The local UI combines the deterministic C++ controller, the real SINE/ANT CAN
parsers, and the production LVGL update code. The controller enclosure has one
button with the production click-then-hold arm gesture. The 160×128 screen is
rendered from a fresh native LVGL framebuffer after each event; it is not a
pre-recorded state image. While armed, virtual time and the displayed armed
timer advance once per second.

Build the native simulator and LVGL renderer, then start the UI:

```bash
./simulator/run.sh --test-only
./test/test_screenshots/build_and_run.sh --local
cd simulator/ui
npm install
npm start
```

Open `http://127.0.0.1:4174/`. **Inject Telemetry** opens the wire-level ESC
and BMS editor. Applied values are serialized into DroneCAN and ANT CAN frames,
respectively, and only reach the display after the production vendor libraries
decode them. Light mode is the default controller-screen theme.
