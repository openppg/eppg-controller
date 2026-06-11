# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

This is an Arduino/ESP32 project using PlatformIO for build management.

**Build Commands:**
- `pio run` - Build the project (compiles firmware)
- `pio run --target upload` - Flash firmware to connected device  
- `pio run --target clean` - Clean build artifacts
- `pio device list` - List connected devices/serial ports
- `pio device monitor` - Serial monitor for debugging

**Environment:**
- Default target: `OpenPPG-CESP32S3-CAN-SP140` (ESP32-S3 based)
- Build type: debug (configurable in platformio.ini)
- Monitor speed: 115200 baud

**Binary Generation:**
For creating update files, use esptool to merge binaries:
```bash
esptool.py --chip esp32s3 merge_bin \
  -o .pio/build/OpenPPG-CESP32S3-CAN-SP140/merged-firmware.bin \
  --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x0 .pio/build/OpenPPG-CESP32S3-CAN-SP140/bootloader.bin \
  0x8000 .pio/build/OpenPPG-CESP32S3-CAN-SP140/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/OpenPPG-CESP32S3-CAN-SP140/firmware.bin
```

**Toolchain quirks (read before building):**
- Always use `.venv-pio313/bin/pio` (PlatformIO Core 6.1.19); the bare `pio` asdf shim fails.
- Building the firmware rewrites the tracked `sdkconfig.OpenPPG-CESP32S3-CAN-SP140` to LF line endings (content unchanged). After any firmware build, run `git checkout -- sdkconfig.OpenPPG-CESP32S3-CAN-SP140` so diffs stay clean.
- NEVER delete the committed sdkconfig to "regenerate it from sdkconfig.defaults" — it holds non-default values (log level, mbedtls options, ISR stack size, partition-table selection) that the defaults file does not capture; a regeneration silently reverts them. Make targeted edits to the committed sdkconfig and mirror the intent in `sdkconfig.defaults`.
- Do NOT run the screenshot suite via `pio test -e native-screenshot` — that env intentionally compiles no `src/` files and fails at link. Use `./test/test_screenshots/build_and_run.sh` (what CI runs; pass `--update-references` to regenerate the baseline BMPs).

**Testing:**
- Host unit tests (googletest): `.venv-pio313/bin/pio test -e native-test` — suites in `/test/`: `test_throttle`, `test_hysteresis`, `test_multilogger`, `test_simplemonitor`, `test_diagnostics`, `test_esc_errors` (placeholder). CI runs these on every push.
- Screenshot regression tests: `./test/test_screenshots/build_and_run.sh` — renders the LVGL screens in a host emulator and compares against reference BMPs (also in CI).
- Hardware-in-the-loop testing remains the final validation layer for anything touching CAN, BLE, the display, or motor behavior.

## Code Architecture

**Project Structure:**
- `/src/sp140/` - Main application source code
- `/inc/sp140/` - Header files and configuration
- `/src/assets/` - Fonts, images, and UI resources
- `/libraries/` - Custom/vendored libraries (set by `lib_dir` in platformio.ini)
- `platformio.ini` - Build configuration

**Core Components:**

**Main Controller (`src/sp140/main.cpp`):**
The entry point that coordinates all subsystems. Handles:
- Hardware initialization and SPI bus management
- Main control loop and state management  
- Button input processing with debouncing
- Safety interlocks and watchdog management

**Key Subsystems:**
- **ESC Control** (`esc.h/cpp`) - Motor speed control and CAN communication
- **BMS Interface** (`bms.h/cpp`) - Battery management system communication
- **Throttle Processing** (`throttle.h/cpp`) - Input filtering and response curves
- **Display/LVGL** (`lvgl/`) - LCD graphics and user interface
- **Altimeter** (`altimeter.h/cpp`) - Barometric altitude sensing
- **Device State** (`device_state.h`) - DeviceState enum; the arm/disarm/cruise state machine itself lives in `main.cpp`
- **Mode Control** (`mode.h/cpp`) - Flight modes (manual, cruise, etc.)

**Hardware Platform:**
- ESP32-S3 microcontroller (M5Stack STAMPS3 board)
- CAN bus communication for ESC and BMS
- Shared SPI bus for display and peripherals
- NeoPixel RGB LEDs for status indication
- Analog throttle input with ResponsiveAnalogRead filtering

**Configuration:**
- Hardware pin definitions in `esp32s3-config.h`
- Device settings and globals in `globals.h`
- Build flags and library versions in `platformio.ini`

**Data Structures:**
- Telemetry data structures defined in `structs.h`
- Temperature monitoring with multiple sensor states
- BMS pack data with cell-level monitoring
- ESC telemetry with real-time performance data

This codebase implements a safety-critical paramotor throttle controller with real-time requirements and hardware integration.
