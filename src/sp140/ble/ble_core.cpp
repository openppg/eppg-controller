#include "sp140/ble/ble_core.h"

#include <Arduino.h>

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/bms_service.h"
#include "sp140/ble/config_service.h"
#include "sp140/ble/controller_service.h"
#include "sp140/ble/esc_service.h"
#include "sp140/ble/ota_service.h"

namespace {

// Store the active connection handle for conn param updates
uint16_t activeConnHandle = 0;

class BleServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    activeConnHandle = connInfo.getConnHandle();
    deviceConnected = true;
    // Request relaxed connection parameters for normal use: ~36ms interval
    // Telemetry runs at 1-10Hz so aggressive intervals waste phone battery.
    // OTA tightens to 15ms via requestFastConnParams() when it starts.
    // Units: interval = 1.25ms, timeout = 10ms
    server->updateConnParams(activeConnHandle, 24, 40, 0, 400);
    USBSerial.println("Device connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)connInfo;
    (void)reason;
    deviceConnected = false;

    if (isOtaInProgress()) {
      abortOta();
      USBSerial.println("OTA aborted due to disconnect");
    }

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

  // Set TX power to +9dBm for reliable connections at distance
  NimBLEDevice::setPower(9);

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
  initOtaBleService(pServer);

  NimBLEAdvertising* advertising = pServer->getAdvertising();
  advertising->setName("OpenPPG Controller");
  advertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  // Advertise only 1 UUID to fit within 31+31 byte advertisement payload.
  // All other services are discoverable via GATT service discovery after connection.
  advertising->addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));
  advertising->enableScanResponse(true);
  // Note: Advertising is deferred until after splash screen

  USBSerial.println("BLE device ready (advertising deferred)");
}

void requestFastConnParams() {
  if (pServer == nullptr || !deviceConnected) {
    return;
  }
  // Tighten to 15ms interval for OTA throughput
  pServer->updateConnParams(activeConnHandle, 12, 12, 0, 200);
}

void requestNormalConnParams() {
  if (pServer == nullptr || !deviceConnected) {
    return;
  }
  // Relax back to ~36ms for normal telemetry
  pServer->updateConnParams(activeConnHandle, 24, 40, 0, 400);
}

void restartBLEAdvertising() {
  if (pServer == nullptr) {
    return;
  }

  NimBLEAdvertising* advertising = pServer->getAdvertising();
  advertising->start();
}
