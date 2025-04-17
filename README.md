# EPPG Controller

![Build](https://github.com/openppg/eppg-controller/actions/workflows/config.yml/badge.svg)

Arduino based logic for OpenPPG SP140 Throttle Controller

#### This master branch is only for testing the latest firmware for the and SP140

X4 code has been migrated to a separate repo - https://github.com/openppg/x4-controller

It may not be stable and is not recommended for flying.
See stable releases [here](https://github.com/openppg/eppg-controller/releases)
> Version 6.1 was intially released only for the reworked RP2040 module based controller and being ported to the original RP2040 PCB which is currently in testing. The version shipped with that hardware can be found on the [rp2040-module-release branch](https://github.com/openppg/eppg-controller/tree/rp2040-module-release)

> Version 6.0 introduced [FreeRTOS](https://www.freertos.org/index.html) and is currently only working with RP2040 processors. For M0/SAMD21 processors please use [version/5 branch](https://github.com/openppg/eppg-controller/tree/version/5).

> For batch 3 (non-telemetry) controllers please see the [batch-3 branch](https://github.com/openppg/eppg-controller/tree/batch-3).

> For batch 2 (Arduino nano based) controllers please see the [batch-2 branch](https://github.com/openppg/eppg-controller/tree/batch-2).

## Build and flash firmware

OpenPPG supports flashing the firmware PlatformIO. Older versions were also compatible with Arduino IDE.

## Using PlatformIO

Suitable for Mac, Windows, and Linux

### Setup

1. Follow the instructions here for using with VSCode https://platformio.org/install/ide?install=vscode
2. Extract the downloaded code from the repo [here](https://github.com/openppg/eppg-controller/archive/master.zip) (or `git clone` it)
3. Open the folder using the PlatformIO "open project" option inside of VSCode.

### Flash the OpenPPG Code

1. Click the "PlatformIO Build" button inside of VSCode or enter `platformio run --target upload` in the command line. PlatformIO will automatically download libraries the first time it runs.

#### Install the driver

Batch 3+ OpenPPG controllers and early SP140 controllers are powered by Atmel's SAMD21G18A MCU, featuring a 32-bit ARM Cortex® M0+ core. On some operating systems you may need extra drivers to communicate with it.
Newer SP140 controllers feature the RP2040 MCU which is a dual-core variant of the ARM Cortex® M0+.

#### Download and Prepare OpenPPG Code

1. Download the latest controller code zip from [here](https://github.com/openppg/eppg-controller/archive/master.zip)

#### Flash the OpenPPG Code

1. First make sure the code compiles by hitting the check button in the bottom left "Build".
2. Connect the controller to your computer by using the USB port on the bottom of the controller.
3. Flash the firmware by clicking "Upload" in the bottom left.

## Bootloader

The latest batches of OpenPPG SP140 controllers use the UF2 bootloader (compatible with Arduino).
It makes firmware updates as simple as drag and drop.
Learn more here https://github.com/openppg/uf2-samdx1

### Building .uf2 update file

The uf2 bootloader can update firmware with a .uf2 binary file built from a complied .bin firmware file. The .bin file is automatically built when "verifying" the firmware in either Arduino IDE or PIO.
Using the uf2-samdx repo above python tool the command to build a compatible .uf2 file should look something like:

```bash
$ python3 utils/uf2conv.py eppg-controller/.pio/build/OpenPPG\ CM0/firmware.bin -c -o sp140-update.uf2
```

## Config tool

The open source web based config tool for updating certain settings over USB (without needing to flash firmware) can be found at https://config.openppg.com.

## State Management Architecture

The controller uses a queue-based architecture for handling state changes (like DISARMED -> ARMED) and providing feedback to the user through melodies and vibrations:

### Components:

1. **Event Handler (handleButtonEvent)**: Detects user actions and sends state change requests to the state manager task.

2. **State Manager Task**: Processes state change requests, validates them, updates the system state while holding the stateMutex for the shortest possible time, and queues notification requests.

3. **Notification Task**: Handles feedback requests from the State Manager Task, playing sounds and vibration patterns without blocking other critical tasks.

### Queues:

- **stateChangeRequestQueue**: Receives requests from button events to change the system state.
- **notificationQueue**: Used by the State Manager Task to send feedback commands to the Notification Task.

### Workflow Example: DISARMED -> ARMED

1. User performs a long press on the button.
2. The AceButton library detects this and calls `handleButtonEvent` with `kEventLongPressed`.
3. `handleButtonEvent` sends an `ARM_DISARM_REQUEST` to the `stateChangeRequestQueue`.
4. The State Manager Task receives the request and:
   - Validates the preconditions (throttle not engaged, cooldown period passed)
   - Acquires the stateMutex
   - Updates the state to ARMED and performs minimal setup
   - Releases the stateMutex
   - Sends notification requests for melody and vibration feedback
5. The Notification Task receives the requests and plays the appropriate melody and vibration pattern.

This architecture ensures that critical operations aren't delayed by user feedback actions, making the system more responsive and robust.

## Help improve these docs

Pull requests are welcome for these instructions and code changes.
