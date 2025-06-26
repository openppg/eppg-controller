// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_GLOBALS_H_
#define INC_SP140_GLOBALS_H_

#include "sp140/shared-config.h"
#include "sp140/structs.h"
#include "sp140/ble.h"
#include "sp140/alert_system.h"

extern unsigned long cruisedAtMillis;
extern int cruisedPotVal;

extern float watts;
extern float wattHoursUsed;

extern STR_DEVICE_DATA_140_V1 deviceData;

extern STR_ESC_TELEMETRY_140 escTelemetryData;

extern UnifiedBatteryData unifiedBatteryData;  // Instance to hold battery data

extern STR_BMS_TELEMETRY_140 bmsTelemetryData;

// Hardware/Connection Status Flags
extern bool bmsCanInitialized;
extern bool escTwaiInitialized;

#endif  // INC_SP140_GLOBALS_H_
