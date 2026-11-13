# Controller development

Production firmware is a native ESP-IDF 5.5.3 C++ application targeting ESP32-S3.
Arduino and PlatformIO are not part of the build. `src/sp140/main.cpp` exposes
`app_main`, creates the FreeRTOS tasks, and owns the control state. `components/`
contains pinned libraries and native platform drivers; see its README for provenance.

Activate an ESP-IDF environment, then:

```sh
IDF_COMPONENT_MANAGER=0 idf.py -B build-idf build
python3 tools/check_native_release.py build-idf
```

Host regressions are documented in `tools/host-native-tests/README.md`; build with
CMake and run CTest. `./simulator/verify.sh` runs all firmware/host gates.
`tools/host-screenshot-offline` compares production LVGL 9.6 screens with existing
references and fails on missing/different references. QEMU instructions are in README.

Preserve both CAN buses (TWAI ESC 1 Mbps; SPI MCP2515 BMS 250 kbps), the 50 Hz
control deadline and single-owner ESC adapter, all wire encodings/UUIDs and NVS
representations. BLE/Wi-Fi services and update sessions must not block throttle.
Archive matching release ELF/map files; application OTA images and full recovery
images are distinct artifacts. Native/QEMU success does not establish RF or physical
hardware qualification.
