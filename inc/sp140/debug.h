#ifndef INC_SP140_DEBUG_H_
#define INC_SP140_DEBUG_H_

#include <Arduino.h>
#include "sp140/structs.h"

// Function to generate fake telemetry data
void generateFakeTelemetry(STR_ESC_TELEMETRY_140& escTelemetry,  // NOLINT(runtime/references)
                          STR_BMS_TELEMETRY_140& bmsTelemetry,    // NOLINT(runtime/references)
                          UnifiedBatteryData& unifiedBatteryData,  // NOLINT(runtime/references)
                          float& altitude);                        // NOLINT(runtime/references)

#endif  // INC_SP140_DEBUG_H_
