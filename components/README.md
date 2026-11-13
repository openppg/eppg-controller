# Native firmware dependencies

Production builds use ESP-IDF 5.5.3 and do not fetch or link the Arduino SDK.
The application starts at `app_main`; all GPIO, time, USB Serial/JTAG, ADC,
SPI, I2C, PWM, pixel, Wi-Fi and BLE services use native ESP-IDF drivers.

| Component | Source/version | Local changes |
| --- | --- | --- |
| `lvgl` | LVGL 9.5.0, formerly pinned by PlatformIO | Original library sources/configuration; native IDF CMake registration |
| `dronecan` | OpenPPG SINE-ESC-CAN `2ab56a4e5b52e4456317c8ee3e3d802b232c6148` | Original libcanard v0 C and generated DSDL C codecs; C++ adapter/client use native platform clock, console and TWAI |
| `ant_bms` | OpenPPG ANT-BMS-CAN `fd54852bc6f1c9608e37af9ca7c13ea4135c095b` | Existing parser preserved; native MCP2515 frame input, standard C++ strings/time |
| `bmp3` | Bosch BMP3 Sensor API 2.0.1 from the previously pinned BMP3XX dependency | Original C compensation/API, native I2C callbacks and calibration CRC check |
| `controller_platform` | This repository | Native USB, GPIO, ADC, SPI, I2C, ST7735, MCP2515, BMP3, LEDC/RMT and NimBLE GAP/GATT integration |

Upstream notices/licenses remain alongside the vendored code. `dronecan/src/canard.c`
and related files carry their original licenses, independently of the SINE wrapper.
The exact old Arduino-dependent wrappers remain only in `simulator/reference-can`
as test oracles. Their host transport stubs are not part of a firmware image.

ESP-IDF's C components, LVGL, Bosch compensation and libcanard/codecs remain C.
This branch removes Arduino while preserving the C++ application language.

The C ABI in `inc/platform/peripheral.h` is shared with the Rust migration and
the explicitly selected QEMU peripheral backend. `EPPG_QEMU` never enables radio
hardware: the emulator build reports that limitation when it starts.
