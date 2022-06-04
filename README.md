# EPPG Controller

![Build](https://github.com/openppg/eppg-controller/actions/workflows/config.yml/badge.svg)

Arduino based logic for OpenPPG Throttle Controller

#### This master branch is only for testing the latest firmware for the X4 and SP140

It may not be stable and is not recommended for flying.
See stable releases [here](https://github.com/openppg/eppg-controller/releases)

> For batch 3 (non-telemetry) controllers please see the [batch-3 branch](https://github.com/openppg/eppg-controller/tree/batch-3).

> For batch 2 (Arduino nano based) controllers please see the [batch-2 branch](https://github.com/openppg/eppg-controller/tree/batch-2).

## Build and flash firmware

OpenPPG supports flashing the firmware PlatformIO. Older versions were also compatible with Arduino IDE

## Using PlatformIO

Suitable for Mac, Windows, and Linux

### Setup

1. Follow the instructions here for using with VSCode https://platformio.org/install/ide?install=vscode
2. Extract the downloaded code from the repo [here](https://github.com/openppg/eppg-controller/archive/master.zip) (or `git clone` it)
3. Open the folder using the PlatformIO "open project" option inside of VSCode

### Flash the OpenPPG Code

1. Click the "PlatformIO Build" button inside of VSCode or enter `platformio run --target upload` in the command line. PlatformIO will automatically download libraries the first time it runs

#### Install the driver

Batch 3+ OpenPPG controllers and all SP140 controllers are powered by Atmel’s SAMD21G18A MCU, featuring a 32-bit ARM Cortex® M0+ core. On some operating systems you may need extra drivers to communicate with it.

#### Download and Prepare OpenPPG Code

1. Download the latest controller code zip from [here](https://github.com/openppg/eppg-controller/archive/master.zip)

#### Flash the OpenPPG Code

1. First make sure the code compiles by hitting the check button in the bottom left "Build"
2. Connect the controller to your computer by using the micro USB port on the bottom of the controller
3. Flash the firmware by clicking "Upload" in the bottom left

## Bootloader

The latest batches of OpenPPG X4 and SP140 controllers use the UF2 bootloader (compatible with Arduino).
It makes firmware updates as simple as drag and drop.
Learn more here https://github.com/openppg/uf2-samdx1

### Building .uf2 update file

The uf2 bootloader can update firmware with a .uf2 binary file built from a complied .bin firmware file. The .bin file is automatically built when "verifying" the firmware in either Ardion IDE or PIO.
Using the uf2-samdx repo above python tool the command to build a compatible .uf2 file should look something like:

```bash
$ python3 utils/uf2conv.py eppg-controller/.pio/build/OpenPPG\ CM0/firmware.bin -c -o sp140-update.uf2
```

## Config tool

The open source web based config tool for updating certain settings over USB (without needing to flash firmware) can be found at https://config.openppg.com.


## Help improve these docs

Pull requests are welcome for these instructions and code changes
