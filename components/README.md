# Native firmware dependencies

Production builds use ESP-IDF 5.5.3 and do not fetch or link the Arduino SDK.
The application starts at `app_main`; all GPIO, time, USB Serial/JTAG, ADC,
SPI, I2C, PWM, pixel, Wi-Fi and BLE services use native ESP-IDF drivers.

| Component | Source/version | Local changes |
| --- | --- | --- |
| `lvgl` | LVGL 9.6.0, formerly pinned by PlatformIO | Upstream tag `v9.6.0` (`80ca777e37a2b176770726a02e07a6fb79ef0b39`); native IDF CMake registration |
| `dronecan` | OpenPPG SINE-ESC-CAN `2ab56a4e5b52e4456317c8ee3e3d802b232c6148` | Original libcanard v0 C and generated DSDL C codecs; C++ adapter/client use native platform clock, console and TWAI |
| `ant_bms` | OpenPPG ANT-BMS-CAN `fd54852bc6f1c9608e37af9ca7c13ea4135c095b` | Native MCP2515 frame input, standard C++ strings/time; corrected three-frame identity indexing and short-frame guards |
| `bmp3` | Bosch BMP3 Sensor API 2.0.1 from the previously pinned BMP3XX dependency | Original C compensation/API, native I2C callbacks and calibration CRC check |
| `controller_platform` | This repository | Native USB, GPIO, ADC, SPI, I2C, ST7735, MCP2515, BMP3, LEDC/RMT and NimBLE GAP/GATT integration |
| `image_staging` | This repository | Portable NOR image store with one 4 KiB scratch sector and offset/retry semantics |

Upstream notices/licenses remain alongside the vendored code. `dronecan/src/canard.c`
and related files carry their original licenses, independently of the SINE wrapper.
The exact old Arduino-dependent wrappers remain only in `simulator/reference-can`
as test oracles. Their host transport stubs are not part of a firmware image.

One intended-behavior correction differs from the pinned ANT reference: the
three battery identity IDs differ in PF bits16..23. The previous full-ID
subtraction discarded the second/third eight-byte sections. Native regression
tests now require the complete 24-byte identity and reject lookalike/short IDs.

ESP-IDF's C components, LVGL, Bosch compensation and libcanard/codecs remain C.
This branch removes Arduino while preserving the C++ application language.

The C ABI in `inc/platform/peripheral.h` is shared with the Rust migration and
the explicitly selected QEMU peripheral backend. `EPPG_QEMU` never enables radio
hardware: the emulator build reports that limitation when it starts.

## ESC image staging

ESC updates still accept complete images up to 256 KiB, including their 32-byte
header, before starting the CAN bootloader transaction. If PSRAM/internal RAM
cannot hold the image, native storage uses the existing `spiffs` partition.
This firmware does not mount or use a filesystem there. Its first 4 KiB sector
contains an immutable `OPPGESC1` ownership marker; the following 256 KiB is the
image staging region. The partition table and NVS locations do not change.

On first use, the store requires the entire partition to be erased. It refuses
unknown existing data rather than erasing it. Later updates erase only the
owned image area. Interrupted preparation/upload can restart without losing
ownership. Offset retries that need NOR zero-to-one transitions rewrite one
sector using the 4 KiB scratch buffer. Native tests exercise actual store code
against the NOR model, including power cuts and preservation of unknown data.

The application reserves disarmed maintenance authority before preparation;
allocation/erase runs on a separate maintenance worker, and `RECEIVING` is
published only when storage is ready. Flash writes run outside spinlocks; the
control owner reads small CAN payloads with a non-waiting store mutex. The
matching Rust application uses the same low-level C ABI in `platform/staging.h`.

LVGL 9.6.0 archive SHA-256: `b20ee3acc1bba13c62d854f9ebd62e4c51e0b443b1e0225892e86442defa84df`. Public `include/` and implementation `src/` are retained unchanged from that tag.
