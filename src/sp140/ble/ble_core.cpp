#include "sp140/ble/ble_core.h"

#include <Arduino.h>
#include <algorithm>
#include <cctype>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/bms_service.h"
#include "sp140/ble/config_service.h"
#include "sp140/ble/controller_service.h"
#include "sp140/ble/esc_service.h"
#include "sp140/ble/log_sync_service.h"
#include "sp140/logging/telemetry_logger.h"
#if CONFIG_BT_NIMBLE_EXT_ADV
#include <NimBLEExtAdvertising.h>
#endif

namespace {

constexpr uint8_t kExtAdvInstance = 0;

void startAdvertising(NimBLEServer* server) {
  if (server == nullptr) {
    return;
  }

#if CONFIG_BT_NIMBLE_EXT_ADV
  NimBLEExtAdvertisement adv(BLE_HCI_LE_PHY_1M, BLE_HCI_LE_PHY_2M);
  adv.setLegacyAdvertising(false);
  adv.setConnectable(true);
  adv.setScannable(true);
  adv.setName("OpenPPG Controller");
  adv.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  adv.addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));
  adv.addServiceUUID(NimBLEUUID(BMS_TELEMETRY_SERVICE_UUID));
  adv.addServiceUUID(NimBLEUUID(ESC_TELEMETRY_SERVICE_UUID));
  adv.addServiceUUID(NimBLEUUID(CONTROLLER_SERVICE_UUID));
  adv.addServiceUUID(NimBLEUUID(LOG_SYNC_SERVICE_UUID));

  auto* advertising = server->getAdvertising();
  advertising->setInstanceData(kExtAdvInstance, adv);
  advertising->start(kExtAdvInstance);
#else
  auto* advertising = server->getAdvertising();
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
#endif
}

class BleServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    connectedHandle = connInfo.getConnHandle();
    telemetry_log::onBleReconnect();

    // iOS is more likely to accept params after the initial connection settles.
    vTaskDelay(pdMS_TO_TICKS(150));
    server->updateConnParams(connectedHandle, 12, 24, 0, 200);
    server->setDataLen(connectedHandle, 251);
    server->updatePhy(connectedHandle, BLE_GAP_LE_PHY_2M_MASK,
                      BLE_GAP_LE_PHY_2M_MASK, 0);
    USBSerial.println("Device connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo,
                    int reason) override {
    (void)connInfo;
    (void)reason;
    deviceConnected = false;
    connectedHandle = BLE_HS_CONN_HANDLE_NONE;
    telemetry_log::onBleDisconnect();
    USBSerial.println("Device disconnected");
    startAdvertising(server);
    USBSerial.println("Started advertising");
  }
};

}  // namespace

void setupBLE() {
  // Initialize NimBLE with device name
  NimBLEDevice::init("OpenPPG Controller");

  // Bonding and LE secure connections ("Just Works").
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setDefaultPhy(BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK);

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

  startAdvertising(pServer);

  USBSerial.println("BLE device ready");
  USBSerial.println("Waiting for a client connection...");
}

void restartBLEAdvertising() {
  if (pServer == nullptr) {
    return;
  }

  startAdvertising(pServer);
}
