/*
 * globals.cpp
 *
 * This file exists to define global variables that are used across multiple
 * files in the project. It's paired with globals.h, which declares these
 * variables as extern.
 *
 * IMPORTANT NOTE:
 * Global variables should be avoided whenever possible as they can lead to:
 * 1. Difficulty in tracking state changes
 * 2. Increased coupling between different parts of the code
 * 3. Potential thread-safety issues in multi-threaded environments
 * 4. Challenges in unit testing
 *
 * Instead, consider using:
 * - Function parameters
 * - Return values
 * - Class member variables
 * - Dependency injection
 *
 * If you find yourself needing to add a new global variable, first consider
 * if there's a way to refactor the code to avoid it. If a global is truly
 * necessary, add it here and in globals.h, and document why it's needed.
 */

#include "Arduino.h"
#include <cstring>  // For memset

#include "sp140/globals.h"

STR_DEVICE_DATA_140_V1 deviceData;

// Global telemetry data definitions
STR_BMS_TELEMETRY_140 bmsTelemetryData;
STR_ESC_TELEMETRY_140 escTelemetryData;
UnifiedBatteryData unifiedBatteryData;

// Hardware/Connection Status Flags
bool bmsCanInitialized = false;

// Other global variables declared as extern in globals.h
unsigned long cruisedAtMillis = 0;
int cruisedPotVal = 0;
float watts = 0;
float wattHoursUsed = 0;

// Initialize the global variables in a function that will be called from setup()
void initializeGlobalTelemetryData() {
  // Initialize BMS telemetry data with sentinel values instead of zeros
  memset(&bmsTelemetryData, 0, sizeof(bmsTelemetryData));
  bmsTelemetryData.bmsState = TelemetryState::NOT_CONNECTED;
  // Use sentinel values for numeric fields to distinguish from "no data"
  bmsTelemetryData.soc = -1.0f;  // -1 indicates no data
  bmsTelemetryData.battery_voltage = -1.0f;
  bmsTelemetryData.battery_current = -1.0f;
  bmsTelemetryData.power = -1.0f;
  bmsTelemetryData.highest_cell_voltage = -1.0f;
  bmsTelemetryData.lowest_cell_voltage = -1.0f;
  bmsTelemetryData.highest_temperature = -1.0f;
  bmsTelemetryData.lowest_temperature = -1.0f;
  bmsTelemetryData.voltage_differential = -1.0f;
  
  // Initialize ESC telemetry data with sentinel values
  memset(&escTelemetryData, 0, sizeof(escTelemetryData));
  escTelemetryData.escState = TelemetryState::NOT_CONNECTED;
  escTelemetryData.running_error = 0;
  escTelemetryData.selfcheck_error = 0;
  // Use sentinel values for numeric fields
  escTelemetryData.volts = -1.0f;
  escTelemetryData.amps = -1.0f;
  escTelemetryData.cap_temp = -1.0f;
  escTelemetryData.mcu_temp = -1.0f;
  escTelemetryData.motor_temp = -1.0f;
  
  // Initialize unified battery data with sentinel values
  memset(&unifiedBatteryData, 0, sizeof(unifiedBatteryData));
  unifiedBatteryData.volts = -1.0f;
  unifiedBatteryData.amps = -1.0f;
  unifiedBatteryData.power = -1.0f;
  unifiedBatteryData.soc = -1.0f;
}

// Thread Safety Mutexes - initialized to NULL, will be created in setup()
SemaphoreHandle_t telemetryMutex = NULL;
SemaphoreHandle_t buttonMutex = NULL;
SemaphoreHandle_t alertStateMutex = NULL;
SemaphoreHandle_t unifiedDataMutex = NULL;

// Thread-safe wrapper functions implementation
void updateBMSDataThreadSafe(const STR_BMS_TELEMETRY_140& newData) {
  if (telemetryMutex && xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    bmsTelemetryData = newData;
    xSemaphoreGive(telemetryMutex);
  }
}

void updateESCDataThreadSafe(const STR_ESC_TELEMETRY_140& newData) {
  if (telemetryMutex && xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    escTelemetryData = newData;
    xSemaphoreGive(telemetryMutex);
  }
}

STR_BMS_TELEMETRY_140 getBMSDataThreadSafe() {
  STR_BMS_TELEMETRY_140 result;
  if (telemetryMutex && xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    result = bmsTelemetryData;
    xSemaphoreGive(telemetryMutex);
  } else {
    // If mutex fails, return the actual data anyway (not zeroed)
    result = bmsTelemetryData;
  }
  return result;
}

STR_ESC_TELEMETRY_140 getESCDataThreadSafe() {
  STR_ESC_TELEMETRY_140 result;
  if (telemetryMutex && xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    result = escTelemetryData;
    xSemaphoreGive(telemetryMutex);
  } else {
    // If mutex fails, return the actual data anyway (not zeroed)
    result = escTelemetryData;
  }
  return result;
}

void updateUnifiedBatteryDataThreadSafe(const UnifiedBatteryData& newData) {
  if (unifiedDataMutex && xSemaphoreTake(unifiedDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    unifiedBatteryData = newData;
    xSemaphoreGive(unifiedDataMutex);
  }
}

UnifiedBatteryData getUnifiedBatteryDataThreadSafe() {
  UnifiedBatteryData result;
  if (unifiedDataMutex && xSemaphoreTake(unifiedDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    result = unifiedBatteryData;
    xSemaphoreGive(unifiedDataMutex);
  } else {
    // If mutex fails, return the actual data anyway (not zeroed)
    result = unifiedBatteryData;
  }
  return result;
}
