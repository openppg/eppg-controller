#ifndef INC_SP140_BLE_ESC_SERVICE_H_
#define INC_SP140_BLE_ESC_SERVICE_H_

#include "sp140/structs.h"

class NimBLEServer;

void initEscBleService(NimBLEServer* server);
void updateESCTelemetryBLE(const STR_ESC_TELEMETRY_140& telemetry);

#endif  // INC_SP140_BLE_ESC_SERVICE_H_
