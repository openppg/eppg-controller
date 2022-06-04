// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_GLOBALS_H_
#define INC_SP140_GLOBALS_H_

byte escData[ESC_DATA_SIZE];
unsigned long cruisedAtMilis = 0;
unsigned long transmitted = 0;
unsigned long failed = 0;
bool cruising = false;
int prevPotLvl = 0;
int cruisedPotVal = 0;
float throttlePWM = 0;
float batteryPercent = 0;
float prevBatteryPercent = 0;
bool batteryFlag = true;
bool throttledFlag = true;
bool throttled = false;
unsigned long throttledAtMillis = 0;
unsigned int throttleSecs = 0;
float minutes = 0;
float prevMinutes = 0;
float seconds = 0;
float prevSeconds = 0;
float hours = 0;  // logged flight hours
float wattsHoursUsed = 0;

uint16_t _volts = 0;
uint16_t _temperatureC = 0;
int16_t _amps = 0;
uint32_t _eRPM = 0;
uint16_t _inPWM = 0;
uint16_t _outPWM = 0;

// ESC Telemetry
float prevVolts = 0;
float prevAmps = 0;
float watts = 0;
float prevKilowatts = 0;
float prevKwh = 0;

// ALTIMETER
float ambientTempC = 0;
float altitudeM = 0;
float aglM = 0;
float lastAltM = 0;

Adafruit_BMP3XX bmp;
Servo esc;  // Creating a servo class with name of esc

static STR_DEVICE_DATA_140_V1 deviceData;

uint16_t bottom_bg_color = DEFAULT_BG_COLOR;

#endif  // INC_SP140_GLOBALS_H_
