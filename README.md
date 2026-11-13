# EPPG Controller

[![Build](https://github.com/openppg/eppg-controller/actions/workflows/config.yml/badge.svg)](https://github.com/openppg/eppg-controller/actions/workflows/config.yml)

Native ESP-IDF firmware for the ESP32-S3 OpenPPG SP140 Throttle Controller.
The application is C++, uses FreeRTOS directly, and has no Arduino SDK dependency.

### Key Features

- CAN bus communication with ESC and BMS
- LVGL-based LCD display interface
- Bluetooth Low Energy (BLE) connectivity via NimBLE
- Barometric altitude sensing (BMP3XX)
- Multiple flight modes (manual, cruise)
- Real-time sensor monitoring with hysteresis-based alerting
- Web-based configuration via [config.openppg.com](https://config.openppg.com) (no firmware flash needed)
- Optional encrypted ESP-NOW controller-to-controller control and mirrored telemetry; see [WIRELESS_CONTROL.md](WIRELESS_CONTROL.md)

#### This master branch is only for testing the latest firmware for the SP140 controllers running the ESP32-S3 processor.

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

Install [ESP-IDF 5.5.3](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32s3/get-started/index.html)
with its ESP32-S3 tools, then activate its `export.sh` environment. Dependencies
are vendored under `components/`; PlatformIO and Arduino are not used.

```sh
IDF_COMPONENT_MANAGER=0 idf.py -B build-idf build
python3 tools/check_native_release.py build-idf
idf.py -B build-idf -p PORT flash monitor
```

The active 8 MB partition layout, NVS types/keys, BLE UUIDs and packet formats
are preserved. Firmware starts in `src/sp140/main.cpp:app_main`. The native
platform boundary is `inc/platform/`, implemented in `components/controller_platform/`.
[Dependency versions and licenses](components/README.md) describe the retained C libraries.

Use `build-idf/openppg_controller.bin` for existing BLE OTA clients. Keep its
matching `.elf` and `.map` for crash decoding. A complete recovery image is separate:

```sh
python3 simulator/tools/merge_idf_flash.py build-idf build-idf/openppg-full-recovery.bin
```

The recovery image contains erased data regions; normal OTA and developer flash
commands preserve NVS. Application OTA does not replace an installed bootloader.
ESC firmware uploads retain their 256 KiB limit with a flash staging fallback
when RAM is insufficient; [staging ownership and interruption behavior](components/README.md#esc-image-staging)
protect unknown existing partition data.

## Testing

Host tests compile production controller logic, protocol codecs and LVGL screens.
They need a C/C++ compiler and CMake; GoogleTest 1.15.2 is fetched if `GTEST_DIR`
is not supplied. No board, ESP-IDF or Arduino installation is required for host tests.

```sh
cmake -S tools/host-native-tests -B build-host-native-tests
cmake --build build-host-native-tests --parallel
ctest --test-dir build-host-native-tests --output-on-failure
./simulator/run.sh --test-only
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol
cmake --build build-host-can-protocol --parallel
./build-host-can-protocol/host_can_protocol_probe
cmake -S tools/host-screenshot-offline -B build-host-screenshots
cmake --build build-host-screenshots --parallel
ctest --test-dir build-host-screenshots --output-on-failure
```

`./simulator/verify.sh` runs the firmware build and all host gates from an active
IDF environment. CI performs the same checks and retains release ELF/map files.

## ESP32-S3 instruction-level emulator

The QEMU variant executes this application's actual ESP32-S3 instructions,
FreeRTOS tasks, control logic, monitoring, display driver and protocol adapters.
MCP2515/ST7735/BMP3xx registers and a separate DroneCAN ESC peer provide modeled
peripherals. BLE/Wi-Fi RF and electrical timing need physical hardware validation.

```sh
IDF_COMPONENT_MANAGER=0 idf.py -B build-idf-qemu \
  -D EPPG_QEMU=ON -D SDKCONFIG=build-idf-qemu/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.qemu.defaults' build
python3 simulator/tools/merge_idf_flash.py build-idf-qemu build-idf-qemu/qemu-flash.bin
python3 simulator/tools/qemu_smoke.py --image build-idf-qemu/qemu-flash.bin \
  --qemu /path/to/qemu-system-xtensa --timeout 30 \
  --expect '=== Boot complete ===' --expect 'pwm_01us=16000' \
  --report build-idf-qemu/qemu-smoke.json
```

Use Espressif's QEMU with the `esp32s3` machine. The emulator image uses DIO and
UART0 because the QEMU NOR/USB models differ from the board; physical firmware
continues to use its configured QIO flash and USB Serial/JTAG console. The smoke
scenario drives the real click/hold arming gesture and raw ADC throttle ramp.
Reports retain full UART output and the input image hash.

## Config tool

The open source web based config tool for updating certain settings over USB (without needing to flash firmware) can be found at [config.openppg.com](https://config.openppg.com).

## Help improve these docs

Pull requests are welcome for these instructions and code changes.
