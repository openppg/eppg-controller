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

**Testing:**
- Basic test files exist in `/test/` directory
- No automated test framework configured - testing is primarily hardware-in-the-loop

## Code Architecture

**Project Structure:**
- `/src/sp140/` - Main application source code
- `/inc/sp140/` - Header files and configuration
- `/src/assets/` - Fonts, images, and UI resources
- `/lib/` - Custom libraries
- `platformio.ini` - Build configuration

**Core Components:**

**Main Controller (`sp140.ino`):**
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
- **Device State** (`device_state.h/cpp`) - System state machine management
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