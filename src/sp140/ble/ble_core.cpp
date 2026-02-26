#include "sp140/ble/ble_core.h"

#include <Arduino.h>

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/bms_service.h"
#include "sp140/ble/config_service.h"
#include "sp140/ble/controller_service.h"
#include "sp140/ble/esc_service.h"
#include "sp140/ble/log_sync_service.h"

namespace {

class BleServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    deviceConnected = true;
    USBSerial.println("Device connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)connInfo;
    (void)reason;
    deviceConnected = false;
    USBSerial.println("Device disconnected");
    NimBLEAdvertising* advertising = server->getAdvertising();
    advertising->start();
    USBSerial.println("Started advertising");
  }
};

}  // namespace

void setupBLE() {
  // Initialize NimBLE with device name
  NimBLEDevice::init("OpenPPG Controller");

  // Set MTU to a phone-friendly size (185 works on iOS; Android can go higher)
  NimBLEDevice::setMTU(185);

  pServer = NimBLEDevice::createServer();
  static BleServerConnectionCallbacks serverCallbacks;
  pServer->setCallbacks(&serverCallbacks);

  NimBLEAddress bleAddress = NimBLEDevice::getAddress();
  std::string uniqueId = bleAddress.toString();
  std::transform(uniqueId.begin(), uniqueId.end(), uniqueId.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::toupper(c));
                 });

  initConfigBleService(pServer, uniqueId);
  initBmsBleService(pServer);
  initEscBleService(pServer);
  initControllerBleService(pServer);
  initLogSyncBleService(pServer);

  NimBLEAdvertising* advertising = pServer->getAdvertising();
  advertising->setName("OpenPPG Controller");
  advertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  advertising->addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));
  advertising->addServiceUUID(NimBLEUUID(BMS_TELEMETRY_SERVICE_UUID));
  advertising->addServiceUUID(NimBLEUUID(ESC_TELEMETRY_SERVICE_UUID));
  advertising->addServiceUUID(NimBLEUUID(CONTROLLER_SERVICE_UUID));
  advertising->addServiceUUID(NimBLEUUID(LOG_SYNC_SERVICE_UUID));
  advertising->enableScanResponse(true);
  advertising->start();

  USBSerial.println("BLE device ready");
  USBSerial.println("Waiting for a client connection...");
}

void restartBLEAdvertising() {
  if (pServer == nullptr) {
    return;
  }

  NimBLEAdvertising* advertising = pServer->getAdvertising();
  advertising->start();
}
