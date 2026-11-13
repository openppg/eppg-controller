# Emulation architecture and hardware map

## Controller hardware

| Function | Device / bus | ESP32-S3 connection | Firmware cadence |
|---|---|---|---|
| Throttle | 12-bit ADC | GPIO 7 | 20 ms / 50 Hz |
| ESC | internal TWAI, 1 Mbit/s DroneCAN | RX 3, TX 2 | command/telemetry at 20 ms |
| BMS | MCP2515, 250 kbit/s CAN on shared HSPI | CS 5; SCLK 12, MOSI 11, MISO 13 | 100 ms / 10 Hz |
| Display | ST7735, 160×128 RGB565 on shared HSPI | CS 10, DC 14, reset 15 | 33 ms / ~30 Hz |
| Altimeter | BMP3XX I2C, 25 Hz ODR | SDA 44, SCL 41; active GPIO40/42 setup; GPIO9 helper unused | altitude at ~30 Hz; pressure/temperature published at 10 Hz |
| Button | active-low digital input | GPIO 1 | 10 ms, 50 ms debounce |
| Buzzer | LEDC channel 1 | GPIO 8 | queued notes |
| Vibration | LEDC channel 0 | GPIO 46 | queued patterns |
| Status LED | NeoPixel | GPIO 21 | 500 ms status task |
| Radio | NimBLE + encrypted ESP-NOW | integrated BLE/Wi-Fi | BLE notify 50 Hz; ESP-NOW control 50 Hz |
| Persistence | NVS / Preferences and OTA partitions | internal flash | event-driven |

## Production RTOS topology

| Task | Core / priority | Period or wait |
|---|---|---|
| throttle | core 0 / 3 | 20 ms; subscribed to five-second WDT |
| watchdog | core 0 / 3 | 100 ms |
| ButtonHandler | core 0 / 2 | 10 ms |
| UI | core 1 / 2 | 33 ms |
| BMS | core 1 / 2 | 100 ms |
| CtrlSensor | unpinned / 2 | 100 ms |
| BLENotify | core 1 / 1 | 20 ms |
| Monitoring | unpinned / 1 | 100 ms |
| blinkLed | unpinned / 1 | 500 ms |
| Audio | core 1 / 1 | queue-driven |
| Vibration | core 1 / 2 | queue-driven |
| WebSerial | unpinned / 1 | 100 ms |
| AlertAgg | unpinned / 1 | queue wait up to 500 ms |

The native simulator currently executes the safety-critical periodic spine in
virtual time. Its `TaskStats` values make cadence regressions testable. Audio,
vibration, WebSerial, full NimBLE/GATT, ESP-NOW RF behavior, NVS and OTA are
represented only by state/counters or remain at the fidelity boundary.

## Three complementary execution tiers

1. **Hardware-aware controller simulation** (`simulator/`): production
   throttle code, deterministic RTOS cadence, state/fault scenarios, alerts,
   watchdog, JSONL telemetry, and two bitrate-aware bounded CAN networks. Real
   SINE/libcanard and ANT BMS code forms the boundary between controller state
   and the ESC/battery plants.
2. **Isolated protocol/display execution**: the host CAN probe independently
   checks the real SINE DroneCAN and ANT BMS byte parsers over minimal fake
   transports; the screenshot test runs the real LVGL screens into a
   framebuffer.
3. **Instruction execution**: the native ESP-IDF 5.5.3 build produces the actual
   dual-core Xtensa ELF and an 8 MB merged QEMU image. The firmware inspector
   exposes sections, symbols, hashes, and source-interleaved disassembly.

These tiers avoid conflating functional correctness with electrical or
cycle-accurate timing. The CAN and behavioral tiers are now joined at the wire
frame boundary. The next useful fidelity increments are compiling more
production logic against host platform interfaces, modeling CAN
arbitration/error counters at sampled-bit level where needed, and adding missing
QEMU devices only where instruction-level behavior changes a result.

## Important physical timing boundaries

- ESC throttle updates must remain below the ESC's roughly 300 ms loss timeout;
  production sends at 20 ms.
- Firmware marks ESC telemetry stale at 50 ms and polls TWAI health at 250 ms.
- BMS link timeout is 500 ms, after a coherent-snapshot gate.
- CAN links must be continuously established for 1000 ms; loss then debounces
  for 500 ms before an alert.
- BMP3XX boot waits 20 ms and retries initialization up to four times with
  50 ms gaps; vertical speed spans up to 50 samples.
- ESP-NOW receiver control loss fails safe at 150 ms; remote status expires at
  300 ms and mirrored telemetry at 500 ms.
- The hardware task watchdog resets after five seconds and deliberately watches
  the throttle and watchdog tasks, not UI/BMS/BLE stalls.

## QEMU reality check

Espressif's official support matrix lists dual-core Xtensa, flash/MMU, interrupt
matrix, SysTimer, timer groups, GDMA, crypto and TWAI/CAN for ESP32-S3. It lists
Wi-Fi, Bluetooth, USB, GP-SPI, I2C, LEDC and GPIO matrix/IOMUX as unsupported.
The native firmware targets ESP-IDF 5.5.3. Its explicit `EPPG_QEMU` variant
runs the actual `app_main`, FreeRTOS tasks, control/monitor code and protocol
adapters. The platform layer dispatches unsupported physical peripherals to
register-level MCP2515/ST7735/BMP3xx models and a libcanard ESC peer. QEMU uses
DIO flash and UART0 because its NOR model lacks the QIO-enable register and its
S3 machine has no USB Serial/JTAG device. Native board builds retain QIO/USB.
BLE/Wi-Fi RF are explicitly disabled in the QEMU build; no RF fidelity is claimed.
The UART smoke report retains boot, framebuffer/checksum and decoded CAN output.

Sources:

- [ESP32-S3 QEMU guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/qemu.html)
- [Espressif QEMU feature matrix](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/README.md)
- [ESP32-S3 QEMU machine guide](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/esp32s3/README.md)
