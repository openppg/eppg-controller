#ifndef INC_SP140_BLE_CONTROLLER_SERVICE_H_
#define INC_SP140_BLE_CONTROLLER_SERVICE_H_

#include "sp140/structs.h"

class NimBLEServer;

// Initialize the controller telemetry BLE service
void initControllerBleService(NimBLEServer* server);

// Update controller telemetry (altitude, baro, vario, mcu_temp, uptime)
// Call this at 10Hz along with other packed telemetry updates
void updateControllerPackedTelemetry(float altitude, float baro_temp,
                                      float vario, float mcu_temp);

#endif  // INC_SP140_BLE_CONTROLLER_SERVICE_H_
