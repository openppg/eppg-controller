# Offline host screenshot build probe

This is an isolated macOS/native build of the existing LVGL screenshot test.
It reuses dependencies already cached by PlatformIO, so CMake does not need to
download LVGL or GoogleTest.

From the repository root:

```sh
pio pkg install -e native-screenshot
cmake -S tools/host-screenshot-offline -B build-host-screenshot \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-host-screenshot --parallel
./build-host-screenshot/screenshot_tests
```

The test executable must be launched from the repository root because the
existing test reads and writes `test/test_screenshots/{reference,output}`.

This validates the application UI and simulated display framebuffer, but it is
not ESP32-S3 instruction-level emulation. Hardware, RTOS scheduling, and radio
behavior need separate simulation layers.
