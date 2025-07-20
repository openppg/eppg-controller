// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_GLOBALS_H_
#define INC_SP140_GLOBALS_H_

#include "sp140/shared-config.h"
#include "sp140/structs.h"
#include "sp140/ble.h"
#include "sp140/simple_monitor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

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

// Thread Safety Mutexes
extern SemaphoreHandle_t telemetryMutex;      // For bmsTelemetryData, escTelemetryData
extern SemaphoreHandle_t buttonMutex;         // For button state variables
extern SemaphoreHandle_t alertStateMutex;     // For alert system state
extern SemaphoreHandle_t unifiedDataMutex;    // For unifiedBatteryData

// Thread-safe wrapper functions
void updateBMSDataThreadSafe(const STR_BMS_TELEMETRY_140& newData);
void updateESCDataThreadSafe(const STR_ESC_TELEMETRY_140& newData);
STR_BMS_TELEMETRY_140 getBMSDataThreadSafe();
STR_ESC_TELEMETRY_140 getESCDataThreadSafe();
void updateUnifiedBatteryDataThreadSafe(const UnifiedBatteryData& newData);
UnifiedBatteryData getUnifiedBatteryDataThreadSafe();

// Global telemetry data initialization
void initializeGlobalTelemetryData();

#endif  // INC_SP140_GLOBALS_H_
