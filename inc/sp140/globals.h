// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_GLOBALS_H_
#define INC_SP140_GLOBALS_H_

byte escData[ESC_DATA_SIZE];
byte escDataV2[ESC_DATA_V2_SIZE];
unsigned long cruisedAtMillis = 0;
volatile bool cruising = false;
int prevPotLvl = 0;
int cruisedPotVal = 0;
volatile float throttlePWM = 0;
bool throttledFlag = true;

float watts = 0;
float wattHoursUsed = 0;

// sensor states
bool vibePresent = false;

int16_t _amps = 0;

Servo esc;  // Creating a servo class with name of esc

static STR_DEVICE_DATA_140_V1 deviceData;

#endif  // INC_SP140_GLOBALS_H_
