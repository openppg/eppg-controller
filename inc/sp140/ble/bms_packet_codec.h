#ifndef INC_SP140_BLE_BMS_PACKET_CODEC_H_
#define INC_SP140_BLE_BMS_PACKET_CODEC_H_

#include <cstdint>

#include "sp140/structs.h"

BLE_BMS_Telemetry_V1 buildBMSPackedTelemetryV1(
    const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id);

BLE_BMS_Extended_Telemetry_V1 buildBMSExtendedTelemetryV1(
    const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id);

#endif  // INC_SP140_BLE_BMS_PACKET_CODEC_H_
