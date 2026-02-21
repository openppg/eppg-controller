#include <Arduino.h>

#include "sp140/ble/esc_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

NimBLEService* pEscService = nullptr;

// Binary packed telemetry characteristic (V1)
NimBLECharacteristic* pESCPackedTelemetry = nullptr;

}  // namespace

void initEscBleService(NimBLEServer* server) {
  if (pEscService != nullptr) {
    return;
  }

  pEscService = server->createService(NimBLEUUID(ESC_TELEMETRY_SERVICE_UUID));

  // Binary packed telemetry characteristic (always enabled)
  pESCPackedTelemetry = pEscService->createCharacteristic(
      NimBLEUUID(ESC_PACKED_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Initialize packed telemetry with zeros
  BLE_ESC_Telemetry_V1 initialPacket = {};
  initialPacket.version = 1;
  pESCPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialPacket),
      sizeof(BLE_ESC_Telemetry_V1));

  pEscService->start();
}

// Binary packed telemetry update (V1 protocol)
void updateESCPackedTelemetry(const STR_ESC_TELEMETRY_140& telemetry) {
  if (pEscService == nullptr || pESCPackedTelemetry == nullptr) {
    return;
  }

  BLE_ESC_Telemetry_V1 packet;
  packet.version = 1;
  packet.connection_state = static_cast<uint8_t>(telemetry.escState);
  packet.volts = telemetry.volts;
  packet.amps = telemetry.amps;
  packet.mos_temp = telemetry.mos_temp;
  packet.cap_temp = telemetry.cap_temp;
  packet.mcu_temp = telemetry.mcu_temp;
  // motor_temp is NaN when sensor is disconnected/invalid.
  packet.motor_temp = telemetry.motor_temp;
  packet.eRPM = static_cast<int32_t>(telemetry.eRPM);
  packet.inPWM = static_cast<uint16_t>(telemetry.inPWM);
  packet.running_error = telemetry.running_error;
  packet.selfcheck_error = telemetry.selfcheck_error;
  packet.lastUpdateMs = static_cast<uint32_t>(telemetry.lastUpdateMs);

  pESCPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_ESC_Telemetry_V1));

  if (deviceConnected) {
    pESCPackedTelemetry->notify();
  }
}
