#ifndef INC_SP140_BLE_BMS_SERVICE_H_
#define INC_SP140_BLE_BMS_SERVICE_H_

#include "sp140/structs.h"

class NimBLEServer;

void initBmsBleService(NimBLEServer* server);

// Legacy individual characteristic updates (can be disabled with DISABLE_LEGACY_BLE_TELEMETRY)
void updateBMSTelemetry(const STR_BMS_TELEMETRY_140& telemetry);

// Binary packed telemetry update (V1 protocol)
// bms_id: 0-3 for multi-BMS support
void updateBMSPackedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id = 0);

#endif  // INC_SP140_BLE_BMS_SERVICE_H_
