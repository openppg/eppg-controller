#ifndef INC_SP140_ESPNOW_BRIDGE_H_
#define INC_SP140_ESPNOW_BRIDGE_H_

#include <cstdint>

#include "sp140/device_state.h"
#include "sp140/structs.h"
#include "sp140/wireless_protocol.h"

constexpr uint32_t ESPNOW_CONTROL_TIMEOUT_MS = 150;
constexpr uint32_t ESPNOW_REMOTE_STATUS_TIMEOUT_MS = 300;

bool beginEspNowBridge();
bool espNowBridgeActive();

// Remote-side API.
bool espNowSendControl(uint16_t throttle_raw, DeviceState requested_state,
                       uint8_t performance_mode);
bool espNowTakeAck(wireless::AckPacket* ack);
bool espNowTakeTelemetry(BLE_FastLink_Telemetry* telemetry);
void espNowApplyMirroredTelemetry(const BLE_FastLink_Telemetry& telemetry);
void espNowMarkMirroredTelemetryDisconnected();
uint32_t espNowRemoteStatusAgeMs();
uint32_t espNowRemoteTelemetryAgeMs();

// Receiver-side API.
bool espNowGetLatestControl(wireless::ControlPacket* control,
                            uint32_t* age_ms);
void espNowSetFailsafeLatched(bool latched);
bool espNowSendAck(DeviceState actual_state);
bool espNowSendTelemetry(const BLE_FastLink_Telemetry& telemetry);

#endif  // INC_SP140_ESPNOW_BRIDGE_H_
