#include <Arduino.h>

#include "sp140/ble/bms_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/bms_packet_codec.h"

namespace {

NimBLEService* pBmsService = nullptr;

// Binary packed telemetry characteristic (V1)
NimBLECharacteristic* pBMSPackedTelemetry = nullptr;
NimBLECharacteristic* pBMSExtendedTelemetry = nullptr;

}  // namespace

void initBmsBleService(NimBLEServer* server) {
  // Lazily guard against repeated init calls.
  if (pBmsService != nullptr) {
    return;
  }

  pBmsService = server->createService(NimBLEUUID(BMS_TELEMETRY_SERVICE_UUID));

  // Binary packed telemetry characteristic (always enabled)
  pBMSPackedTelemetry = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_PACKED_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSExtendedTelemetry = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_EXTENDED_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Initialize telemetry characteristics with a disconnected baseline.
  STR_BMS_TELEMETRY_140 initialTelemetry = {};
  initialTelemetry.bmsState = TelemetryState::NOT_CONNECTED;
  BLE_BMS_Telemetry_V1 initialPacket = buildBMSPackedTelemetryV1(initialTelemetry, 0);
  pBMSPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialPacket),
      sizeof(BLE_BMS_Telemetry_V1));

  BLE_BMS_Extended_Telemetry_V1 initialExtendedPacket =
      buildBMSExtendedTelemetryV1(initialTelemetry, 0);
  pBMSExtendedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialExtendedPacket),
      sizeof(BLE_BMS_Extended_Telemetry_V1));

  pBmsService->start();
}

// Binary packed telemetry update (V1 protocol)
void updateBMSPackedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id) {
  if (pBmsService == nullptr || pBMSPackedTelemetry == nullptr) {
    return;
  }

  BLE_BMS_Telemetry_V1 packet = buildBMSPackedTelemetryV1(telemetry, bms_id);

  pBMSPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_BMS_Telemetry_V1));

  if (deviceConnected) {
    pBMSPackedTelemetry->notify();
  }
}

void updateBMSExtendedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id) {
  if (pBmsService == nullptr || pBMSExtendedTelemetry == nullptr) {
    return;
  }

  BLE_BMS_Extended_Telemetry_V1 packet =
      buildBMSExtendedTelemetryV1(telemetry, bms_id);
  pBMSExtendedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_BMS_Extended_Telemetry_V1));

  if (deviceConnected) {
    pBMSExtendedTelemetry->notify();
  }
}
