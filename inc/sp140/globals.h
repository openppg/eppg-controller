// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_GLOBALS_H_
#define INC_SP140_GLOBALS_H_

#include "sp140/shared-config.h"
#include "sp140/structs.h"

extern byte escData[ESC_DATA_SIZE];
extern byte escDataV2[ESC_DATA_V2_SIZE];
extern unsigned long cruisedAtMillis;
extern volatile bool cruising;
extern int prevPotLvl;
extern int cruisedPotVal;
extern volatile float throttlePWM;
extern bool throttledFlag;

extern float watts;
extern float wattHoursUsed;

extern int16_t _amps;

extern STR_DEVICE_DATA_140_V1 deviceData;

extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_GLOBALS_H_
