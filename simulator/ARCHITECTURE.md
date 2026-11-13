# Execution tiers and hardware fidelity

| Tier | Executes | Useful evidence | Boundary |
| --- | --- | --- | --- |
| C++ native reference | Migrated production throttle, pinned legacy CAN oracle, production C++ LVGL screens, deterministic controller model | Before/after arithmetic and wire parity, fault scenarios, 34 screen goldens | Main state/monitor model is partial; no RTOS/MMIO timing claim |
| Rust native application | Production Rust Controller, Monitors, Esc/relay/flasher, DroneCAN bridge, ANT parser and Ui | Same code as firmware, raw CAN faults, full update-session sequencing, live framebuffer | Virtual time/physical plants; application wireless protocol rather than RF |
| ESP32-S3 QEMU application | Actual linked Xtensa image, startup, dual-core FreeRTOS, IDF flash/NVS and controller tasks | Instruction execution, task interaction, boot/flash failure evidence, real pixel/packet traffic | Unsupported peripherals have explicit platform-boundary models |
| Physical hardware | Actual firmware on the board | Worst-case interrupt deadlines, ADC transfer/noise, USB/phone compatibility, radio coexistence and electrical faults | Required for claims outside the software model |

## Board and cadence

| Function | Hardware / connection | Cadence |
| --- | --- | --- |
| Throttle | 12-bit ADC GPIO7 | 20 ms / 50 Hz |
| ESC | TWAI TX2/RX3, external transceiver, 1 Mbps DroneCAN v0 | command/telemetry20 ms |
| BMS | MCP2515 CS5, 16 MHz oscillator, 250 kbps; shared SPI SCLK12/MOSI11/MISO13 | producer100 ms |
| Display | ST7735 BLACKTAB160x128 RGB565, CS10/DC14/reset15, shared SPI | UI33 ms |
| Barometer | BMP3xx I2C SDA44/SCL41, active GPIO40/42 startup | 25 Hz ODR, UI/sensor caches |
| Button | GPIO1 active low | 10 ms,50 ms debounce |
| Feedback | GPIO8 buzzer LEDC, GPIO46 vibration LEDC with separate timers; GPIO21 GRB pixel | queued patterns |
| Radio | integrated BLE plus encrypted ESP-NOW peer | BLE50 Hz; control50 Hz, ACK25 Hz, mirrored telemetry10 Hz |
| Persistence | NVS, OTA0/OTA1, OTA metadata and coredump on 8 MB flash | event-driven |

The five-second task watchdog explicitly subscribes the throttle and watchdog
workers. It does not subscribe idle/UI/BMS/BLE tasks. ESC command loss stops the
modeled motor after roughly300 ms, preceding watchdog recovery. ESC telemetry
expires at50 ms, TWAI health/recovery polls250 ms, and BMS freshness is500 ms.
Link alert establishment/clear is1000 ms and loss debounce500 ms. Wireless control
is stale when age>150 ms at a control check, remote status>300 ms and mirrored
telemetry>500 ms. Comparison operators and scheduling quantization matter.

## CAN and device models

Native CAN queues are bounded and frame completion time includes bitrate,
extended-frame overhead, intermission and stuffing allowance. Both sides of the
Rust ESC network use real libcanard transfer CRC and fragmentation/reassembly.
The ANT parser consumes actual raw frames, including its endian/type detection.
There is no sampled-bit arbitration or analog wire model. ANT payload corruption
may remain valid at its application parser because it has no end-to-end CRC;
that behavior is intentionally observable.

The QEMU peripheral backend models ST7735 command/data windows and RGB565 pixel
writes, MCP2515 register commands/mode/finite receive buffers, and BMP3xx raw
ADC/OTP/configuration registers. BMP calibration is a synthetic valid-CRC
fixture, not a measured physical sensor. The separate NOR host model enforces
one-to-zero programming, sector erasure and retained partial writes after an
injected power cut. Actual IDF NVS/flash code executes against QEMU's NOR image
when using the instruction tier.

QEMU CAN wrappers attach a virtual ESC at the frame API. That exercises the
application and codecs on the Xtensa CPU, but deliberately bypasses native
TWAI-controller MMIO/interrupt timing. The machine's USB, Wi-Fi/Bluetooth,
general-purpose SPI, I2C, LEDC and GPIO matrix do not represent the complete
board. Physical RF/encryption timing, EMI, ADC noise, USB host enumeration,
CAN-transceiver behavior and motor/battery dynamics require hardware evidence.

## Regression expectations

Goldens are inputs: screenshot comparison must fail if a reference is missing.
The Rust native simulator uses production interlocks and gestures. The original
C++ scenario model accepted arming during the first500 ms; sample events now
wait600 ms so both tiers exercise valid production actions. Tests distinguish
the two independent protocols and separately exercise controller versus ESC
firmware updates. Emulator success cannot certify flight readiness.

Primary upstream references:

- [Espressif S3 QEMU guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/qemu.html)
- [Espressif QEMU peripheral matrix](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/README.md)
