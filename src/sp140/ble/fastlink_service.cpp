#include "sp140/ble/fastlink_service.h"
#include "sp140/ble.h" // For deviceConnected
#include "sp140/ble/ble_ids.h"
#include "sp140/esc.h" // For requestEscHardwareInfo()
#include "sp140/structs.h" // For BLE_FastLink_Telemetry
#include <Arduino.h>
#include <NimBLECharacteristic.h>
#include <NimBLEDevice.h>

namespace {
NimBLECharacteristic *pFastLinkCharacteristic = nullptr;
uint32_t gFastLinkNotifyOkCount = 0;
uint32_t gFastLinkNotifyFailCount = 0;
uint32_t gFastLinkSkippedNoConnCount = 0;
uint32_t gFastLinkLastStatsMs = 0;

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

class FastLinkTelemetryCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo,
                   uint16_t subValue) override {
    USBSerial.printf(
        "[FASTLINK] subscribe conn=%u addr=%s value=0x%02x notify=%u indicate=%u\n",
        connInfo.getConnHandle(), connInfo.getAddress().toString().c_str(),
        subValue, subValue & 0x01, (subValue >> 1) & 0x01);
  }
};

FastLinkCommandCallbacks commandCallbacks;
FastLinkTelemetryCallbacks telemetryCallbacks;
}  // namespace

void initFastLinkBleService(NimBLEServer *pServer) {
  if (pServer == nullptr)
    return;

  auto *pService = pServer->createService(FAST_LINK_TELEMETRY_SERVICE_UUID);

  pFastLinkCharacteristic = pService->createCharacteristic(
      FAST_LINK_TELEMETRY_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pFastLinkCharacteristic->setCallbacks(&telemetryCallbacks);

  // Set initial value so GATT reads before the first notify return a valid
  // version byte instead of all-zeros (which the app rejects as version 0).
  BLE_FastLink_Telemetry initialData = {};
  initialData.version = FASTLINK_PROTOCOL_VERSION;
  pFastLinkCharacteristic->setValue(
      reinterpret_cast<uint8_t *>(&initialData),
      sizeof(BLE_FastLink_Telemetry));

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
      const bool sent = pFastLinkCharacteristic->notify();
      if (sent) {
        ++gFastLinkNotifyOkCount;
      } else {
        ++gFastLinkNotifyFailCount;
      }
    } else {
      ++gFastLinkSkippedNoConnCount;
    }

    const uint32_t nowMs = millis();
    if (nowMs - gFastLinkLastStatsMs >= 2000) {
      gFastLinkLastStatsMs = nowMs;
      USBSerial.printf(
          "[FASTLINK] stats ok=%lu fail=%lu skippedNoConn=%lu v=%u packet=%lu uptime=%lu connected=%d\n",
          static_cast<unsigned long>(gFastLinkNotifyOkCount),
          static_cast<unsigned long>(gFastLinkNotifyFailCount),
          static_cast<unsigned long>(gFastLinkSkippedNoConnCount),
          static_cast<unsigned>(data.version),
          static_cast<unsigned long>(data.packet_id),
          static_cast<unsigned long>(data.uptime_ms), deviceConnected ? 1 : 0);
    }
  }
}
