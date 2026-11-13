#include "platform/runtime.h"
#ifndef INC_SP140_BLE_FASTLINK_SERVICE_H_
#define INC_SP140_BLE_FASTLINK_SERVICE_H_

#include "sp140/device_state.h"
#include "sp140/telemetry_hub.h"
#include "platform/ble.h"

void initFastLinkBleService(platform::BleServer *pServer);
BLE_FastLink_Telemetry buildFastLinkTelemetryPacket(
    const TelemetryHub &hub, DeviceState deviceState);
void publishFastLinkTelemetry(const TelemetryHub &hub, DeviceState deviceState);

#endif  // INC_SP140_BLE_FASTLINK_SERVICE_H_
