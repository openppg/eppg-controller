#ifndef INC_SP140_BLE_FASTLINK_SERVICE_H_
#define INC_SP140_BLE_FASTLINK_SERVICE_H_

#include "sp140/structs.h"
#include <NimBLEServer.h>

void initFastLinkBleService(NimBLEServer *pServer);
void updateFastLinkTelemetry(const BLE_FastLink_Telemetry &data);

#endif // INC_SP140_BLE_FASTLINK_SERVICE_H_
