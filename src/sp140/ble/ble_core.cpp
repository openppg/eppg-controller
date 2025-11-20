#include "sp140/ble/ble_core.h"

#include <Arduino.h>
#include "esp_gatt_common_api.h"  // For esp_ble_gatt_set_local_mtu

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/bms_service.h"
#include "sp140/ble/config_service.h"
#include "sp140/ble/esc_service.h"

namespace {

class BleServerConnectionCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    USBSerial.println("Device connected");
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    USBSerial.println("Device disconnected");
    BLEAdvertising* advertising = server->getAdvertising();
    advertising->start();
    USBSerial.println("Started advertising");
  }
};

}  // namespace

void setupBLE() {
  // Raise max MTU to a phone-friendly size (185 works on iOS; Android can go higher).
  BLEDevice::setMTU(185);
  BLEDevice::init("OpenPPG Controller");
  // Bluedroid server accepts up to 517 by default; cap the local MTU explicitly.
  esp_ble_gatt_set_local_mtu(185);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerConnectionCallbacks());

  BLEAddress bleAddress = BLEDevice::getAddress();
  std::string uniqueId = bleAddress.toString();
  std::transform(uniqueId.begin(), uniqueId.end(), uniqueId.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::toupper(c));
                 });

  initConfigBleService(pServer, uniqueId);
  initBmsBleService(pServer);
  initEscBleService(pServer);

  BLEAdvertising* advertising = pServer->getAdvertising();
  advertising->addServiceUUID(BLEUUID(CONFIG_SERVICE_UUID));
  advertising->addServiceUUID(BLEUUID(BMS_TELEMETRY_SERVICE_UUID));
  advertising->setScanResponse(false);
  advertising->setMinPreferred(0x0);
  advertising->start();

  USBSerial.println("BLE device ready");
  USBSerial.println("Waiting for a client connection...");
}

void restartBLEAdvertising() {
  if (pServer == nullptr) {
    return;
  }

  BLEAdvertising* advertising = pServer->getAdvertising();
  advertising->start();
}
