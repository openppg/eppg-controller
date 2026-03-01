#include "sp140/ble/fastlink_service.h"
#include "sp140/ble.h" // For deviceConnected
#include "sp140/ble/ble_ids.h"
#include <NimBLECharacteristic.h>
#include <NimBLEDevice.h>

namespace {
NimBLECharacteristic *pFastLinkCharacteristic = nullptr;
}

void initFastLinkBleService(NimBLEServer *pServer) {
  if (pServer == nullptr)
    return;

  auto *pService = pServer->createService(FAST_LINK_TELEMETRY_SERVICE_UUID);
  pFastLinkCharacteristic = pService->createCharacteristic(
      FAST_LINK_TELEMETRY_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pService->start();
}

void updateFastLinkTelemetry(const BLE_FastLink_Telemetry &data) {
  if (pFastLinkCharacteristic != nullptr) {
    pFastLinkCharacteristic->setValue((uint8_t *)&data,
                                      sizeof(BLE_FastLink_Telemetry));
    if (deviceConnected) {
      pFastLinkCharacteristic->notify();
    }
  }
}
