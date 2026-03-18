#ifndef INC_SP140_BLE_FASTLINK_SERVICE_H_
#define INC_SP140_BLE_FASTLINK_SERVICE_H_

#include "sp140/device_state.h"
#include "sp140/telemetry_hub.h"
#include <NimBLEServer.h>

void initFastLinkBleService(NimBLEServer *pServer);
void publishFastLinkTelemetry(const TelemetryHub &hub, DeviceState deviceState);

#endif  // INC_SP140_BLE_FASTLINK_SERVICE_H_
