#include <Arduino.h>

#include "sp140/ble/controller_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

NimBLEService* pControllerService = nullptr;
NimBLECharacteristic* pControllerPackedTelemetry = nullptr;

}  // namespace

void initControllerBleService(NimBLEServer* server) {
  if (pControllerService != nullptr) {
    return;
  }

  pControllerService = server->createService(NimBLEUUID(CONTROLLER_SERVICE_UUID));

  // Binary packed telemetry characteristic
  pControllerPackedTelemetry = pControllerService->createCharacteristic(
      NimBLEUUID(CONTROLLER_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Initialize with zeros
  BLE_Controller_Telemetry_V1 initialPacket = {};
  initialPacket.version = 1;
  pControllerPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialPacket),
      sizeof(BLE_Controller_Telemetry_V1));

  pControllerService->start();
}

void updateControllerPackedTelemetry(float altitude, float baro_temp,
                                      float vario, float mcu_temp) {
  if (pControllerService == nullptr || pControllerPackedTelemetry == nullptr) {
    return;
  }

  BLE_Controller_Telemetry_V1 packet;
  packet.version = 1;
  packet.altitude = altitude;
  packet.baro_temp = baro_temp;
  packet.vario = vario;
  packet.mcu_temp = mcu_temp;
  packet.uptime_ms = static_cast<uint32_t>(millis());

  pControllerPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_Controller_Telemetry_V1));

  if (deviceConnected) {
    pControllerPackedTelemetry->notify();
  }
}
