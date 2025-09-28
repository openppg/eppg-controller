#ifndef INC_SP140_BLE_BMS_SERVICE_H_
#define INC_SP140_BLE_BMS_SERVICE_H_

#include "sp140/structs.h"

class BLEServer;

void initBmsBleService(BLEServer* server);
void updateBMSTelemetry(const STR_BMS_TELEMETRY_140& telemetry);

#endif  // INC_SP140_BLE_BMS_SERVICE_H_
