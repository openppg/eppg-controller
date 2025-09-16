# EPPG Controller

![Build](https://github.com/openppg/eppg-controller/actions/workflows/config.yml/badge.svg)

Arduino based logic for OpenPPG SP140 Throttle Controller

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

OpenPPG supports flashing the firmware PlatformIO. Older versions were also compatible with Arduino IDE.

## Using PlatformIO

(Mac, Windows, and Linux)

### Setup

1. Follow the instructions here for using with VSCode https://platformio.org/install/ide?install=vscode
2. Extract the downloaded code from the repo [here](https://github.com/openppg/eppg-controller/archive/master.zip) (or `git clone` it)
3. Open the folder using the PlatformIO "open project" option inside of VSCode.
4. The first time you build the project it will download the libraries.

#### Download and Prepare OpenPPG Code

1. Download the latest controller code zip from [here](https://github.com/openppg/eppg-controller/archive/master.zip)

#### Flash the OpenPPG Code

1. First make sure the code compiles by hitting the check button in the bottom left "Build".
2. Connect the controller to your computer by using the USB port on the bottom of the controller.
3. Flash the firmware by clicking "Upload" in the bottom left.

### Building binary update file

esp32 uses esptool to build a binary file from the compiled .bin file.

```bash
$ esptool.py --chip esp32s3 merge_bin \
  -o .pio/build/OpenPPG-CESP32S3-CAN-SP140/merged-firmware.bin \
  --flash_mode dio \
  --flash_freq 80m \
  --flash_size 8MB \
  0x0 .pio/build/OpenPPG-CESP32S3-CAN-SP140/bootloader.bin \
  0x8000 .pio/build/OpenPPG-CESP32S3-CAN-SP140/partitions.bin \
  0xe000 /Users/username/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/OpenPPG-CESP32S3-CAN-SP140/firmware.bin
```

## Config tool

The open source web based config tool for updating certain settings over USB (without needing to flash firmware) can be found at https://config.openppg.com.


## Help improve these docs

Pull requests are welcome for these instructions and code changes.
