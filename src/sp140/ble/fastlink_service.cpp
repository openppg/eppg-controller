#include "sp140/ble/fastlink_service.h"
#include "sp140/ble.h" // For deviceConnected
#include "sp140/ble/ble_ids.h"
#include "sp140/esc.h" // For requestEscHardwareInfo()
#include <NimBLECharacteristic.h>
#include <NimBLEDevice.h>

namespace {
NimBLECharacteristic *pFastLinkCharacteristic = nullptr;

// Handles write commands from the app on FAST_LINK_COMMAND_UUID.
// Command 0x01: request ESC hardware info (HW ID, FW version, bootloader, serial).
class FastLinkCommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    std::string val = pChar->getValue();
    if (val.empty()) return;
    switch (static_cast<uint8_t>(val[0])) {
      case 0x01:
        requestEscHardwareInfo();
        break;
      default:
        break;
    }
  }
};

FastLinkCommandCallbacks commandCallbacks;
}  // namespace

void initFastLinkBleService(NimBLEServer *pServer) {
  if (pServer == nullptr)
    return;

  auto *pService = pServer->createService(FAST_LINK_TELEMETRY_SERVICE_UUID);

  pFastLinkCharacteristic = pService->createCharacteristic(
      FAST_LINK_TELEMETRY_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Write-without-response command channel: app sends 0x01 to request ESC hw info
  auto *pCommandCharacteristic = pService->createCharacteristic(
      FAST_LINK_COMMAND_UUID,
      NIMBLE_PROPERTY::WRITE_NR);
  pCommandCharacteristic->setCallbacks(&commandCallbacks);

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
