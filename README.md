# EPPG Controller

Arduino based logic for OpenPPG Throttle Controller

#### This branch is only for the batch 4 and 5 OpenPPG hardware

> For batch 4 controllers please see the [batch-4 branch](https://github.com/openppg/eppg-controller/tree/batch-4).

> For batch 3 (non-telemetry) controllers please see the [batch-3 branch](https://github.com/openppg/eppg-controller/tree/batch-3).

> For batch 2 (Arduino nano based) controllers please see the [batch-2 branch](https://github.com/openppg/eppg-controller/tree/batch-2).

## MacOS

Tested on MacOS Mojave (10.14)

### Install Arduino IDE

1. Download the latest version for Mac OS X [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software)
2. Expand the zip and Copy the Arduino application into the Applications folder

### Install the driver

The Batch 3+ OpenPPG controller is powered by Atmel’s SAMD21G18A MCU, featuring a 32-bit ARM Cortex® M0 core. In order to communicate with it follow the instructions here to set up your computer <https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup> and set up the Arduio IDE <https://learn.adafruit.com/adafruit-feather-m0-basic-proto/using-with-arduino-ide>

### Download and Prepare OpenPPG Code

1. Download the latest controller code zip from [here](https://github.com/openppg/eppg-controller/archive/master.zip)
2. Extract and open "eppg-controller.ino" in the Arduino IDE
3. Open Library Manager (Sketch -> Include Library -> Manage Libraries)
4. Install the following libraries by searching and installing the latest versions:
- `AceButton`
- `Adafruit GFX Library`
- `Adafruit_DRV2605`
- `ArduinoJson`
- `Adafruit_SSD1306`
- `Adafruit SleepyDog`
- `Adafruit_TinyUSB`
- `ArduinoThread`
- `extEEPROM`
- `ResponsiveAnalogRead`
- `Time`(search "Timekeeping")

### Flash the OpenPPG Code

1. First make sure the code compiles by hitting the check button in the top right name "Verify"
2. Connect the controller to your computer by using the micro USB port on the bottom of the controller
3. Under the Tools menu select "Adafruit Feather M0" for the board and "TinyUSB" for the stack
4. Select the proper port (Tools -> Port). It should show up as something like `COM5 (Arduino/Genuino Zero (Native USB port))` or `Feather M0` or `/dev/cu.usbmodem14201`
6. Click the right arrow in the top right named "Upload"
7. Wait for the code to flash and the Arduino IDE to say "Done" at the bottom. Success!

## Windows Instructions Coming Soon
