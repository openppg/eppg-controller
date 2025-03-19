#ifndef INC_SP140_DEBUG_H_
#define INC_SP140_DEBUG_H_

#include <Arduino.h>
#include "sp140/structs.h"

// Helper function to generate smooth oscillating values
float oscillate(float& current_value, const float min, const float max, const float step);

// Function to generate fake telemetry data
void generateFakeTelemetry(STR_ESC_TELEMETRY_140& escTelemetry,  // NOLINT(runtime/references)
                          STR_BMS_TELEMETRY_140& bmsTelemetry,    // NOLINT(runtime/references)
                          UnifiedBatteryData& unifiedBatteryData,  // NOLINT(runtime/references)
                          float& altitude);                        // NOLINT(runtime/references)

// Function to print device data for debugging
void printDeviceData(const STR_DEVICE_DATA_140_V1& deviceData);

#endif  // INC_SP140_DEBUG_H_
