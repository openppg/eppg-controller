#ifndef INC_SP140_BLE_BMS_SERVICE_H_
#define INC_SP140_BLE_BMS_SERVICE_H_

#include "sp140/structs.h"

class NimBLEServer;

void initBmsBleService(NimBLEServer* server);

// Binary packed telemetry update (V1 protocol)
// bms_id: 0-3 for multi-BMS support
void updateBMSPackedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id = 0);
void updateBMSExtendedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id = 0);

#endif  // INC_SP140_BLE_BMS_SERVICE_H_
