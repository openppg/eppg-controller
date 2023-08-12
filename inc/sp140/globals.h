// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_GLOBALS_H_
#define INC_SP140_GLOBALS_H_

byte escData[ESC_DATA_SIZE];
byte escDataV2[ESC_DATA_V2_SIZE];
unsigned long cruisedAtMillis = 0;
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

float watts = 0;
float wattHoursUsed = 0;

// sensor states
bool vibePresent = false;

uint16_t _volts = 0;
uint16_t _temperatureC = 0;
int16_t _amps = 0;
uint32_t _eRPM = 0;
uint16_t _inPWM = 0;
uint16_t _outPWM = 0;

Servo esc;  // Creating a servo class with name of esc

static STR_DEVICE_DATA_140_V1 deviceData;

#endif  // INC_SP140_GLOBALS_H_
