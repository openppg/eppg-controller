#include "sp140/ble/ble_core.h"

#include <Arduino.h>
#include <algorithm>
#include <cctype>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/config_service.h"
#include "sp140/ble/fastlink_service.h"
#if CONFIG_BT_NIMBLE_EXT_ADV
#include <NimBLEExtAdvertising.h>
#endif

namespace {

constexpr const char *kAdvertisingName = "OpenPPG";
constexpr TickType_t kConnTuneDelayTicks = pdMS_TO_TICKS(1200);
constexpr TickType_t kPairingTimeoutTicks = pdMS_TO_TICKS(60000);
TimerHandle_t gConnTuneTimer = nullptr;
TimerHandle_t gPairingTimer = nullptr;
bool pairingModeActive = false;

void onPairingTimeout(TimerHandle_t timer) {
  (void)timer;
  pairingModeActive = false;
  USBSerial.println("[BLE] Pairing mode expired, re-enabling whitelist");
  restartBLEAdvertising();
}

void applyPreferredLinkParams(TimerHandle_t timer) {
  (void)timer;

  auto *server = pServer;
  const uint16_t handle = connectedHandle;
  if (server == nullptr || !deviceConnected ||
      handle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  // Keep reconnect path conservative; avoid immediate PHY / data-length
  // HCI procedures that can destabilize some phone reconnections.
  server->updateConnParams(handle, 12, 24, 0, 200);
  USBSerial.printf("[BLE] Link tune handle=%u connParams=REQUESTED\n", handle);
}

void startAdvertising(NimBLEServer *server) {
  if (server == nullptr) {
    return;
  }

#if CONFIG_BT_NIMBLE_EXT_ADV
  // Legacy connectable undirected advertising via the extended API.
  // Legacy PDUs avoid NimBLE's extended-adv EBUSY state machine bug.
  NimBLEExtAdvertisement adv(BLE_HCI_LE_PHY_1M, BLE_HCI_LE_PHY_1M);
  adv.setLegacyAdvertising(true);
  adv.setConnectable(true);
  adv.setScannable(true);
  adv.setName(kAdvertisingName);
  adv.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);

  // High-frequency advertising for "instant" connection
  adv.setMinInterval(32); // 20ms (32 * 0.625ms)
  adv.setMaxInterval(48); // 30ms (48 * 0.625ms)

  // Always allow connects during reconnect advertising.
  // Security is intentionally disabled for zero-touch reconnect behavior.
  size_t bondCount = NimBLEDevice::getNumBonds();
  adv.setScanFilter(false, false);
  size_t wlCount = NimBLEDevice::getWhiteListCount();
  for (size_t i = 0; i < wlCount; i++) {
    NimBLEDevice::whiteListRemove(NimBLEDevice::getWhiteListAddress(0));
  }

  // Flutter app's `startScan()` filters for CONFIG_SERVICE_UUID.
  adv.addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));

  auto *advertising = server->getAdvertising();
  advertising->stop();
  advertising->removeAll();
  const bool configured = advertising->setInstanceData(kExtAdvInstance, adv);
  const bool started = configured && advertising->start(kExtAdvInstance);
  USBSerial.printf("[BLE] Ext adv configure=%d start=%d (bonds: %u)\n",
                   configured, started, bondCount);
#else
  auto *advertising = server->getAdvertising();

  // Stop before reconfiguring (safe even if not running)
  advertising->stop();

  // Configure payload once — NimBLE accumulates addServiceUUID calls
  static bool payloadConfigured = false;
  if (!payloadConfigured) {
    advertising->setName(kAdvertisingName);
    advertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
    advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
    advertising->addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));
    advertising->enableScanResponse(true);
    advertising->setMinInterval(32); // 20ms (32 * 0.625ms)
    advertising->setMaxInterval(48); // 30ms (48 * 0.625ms)
    payloadConfigured = true;
  }

  // Always allow connects during reconnect advertising.
  // Security is intentionally disabled for zero-touch reconnect behavior.
  size_t bondCount = NimBLEDevice::getNumBonds();
  advertising->setScanFilter(false, false);
  size_t wlCount = NimBLEDevice::getWhiteListCount();
  for (size_t i = 0; i < wlCount; i++) {
    NimBLEDevice::whiteListRemove(NimBLEDevice::getWhiteListAddress(0));
  }

  const bool started = advertising->start();
  USBSerial.printf("[BLE] Legacy adv start=%s (bonds: %u)\n",
                   started ? "OK" : "FAIL", bondCount);
#endif
}

class BleServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    deviceConnected = true;
    connectedHandle = connInfo.getConnHandle();

    if (gConnTuneTimer != nullptr) {
      xTimerStop(gConnTuneTimer, 0);
      xTimerStart(gConnTuneTimer, 0);
    }
    USBSerial.printf("Device connected handle=%u addr=%s\n", connectedHandle,
                     connInfo.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo,
                    int reason) override {
    (void)connInfo;
    if (gConnTuneTimer != nullptr) {
      xTimerStop(gConnTuneTimer, 0);
    }
    deviceConnected = false;
    connectedHandle = BLE_HS_CONN_HANDLE_NONE;
    USBSerial.printf("Device disconnected reason=%d\n", reason);

    // Restart advertising once and keep it stable for inbound reconnects.
    startAdvertising(server);
  }
};

} // namespace

void setupBLE() {
  // Initialize NimBLE with device name
  NimBLEDevice::init(kAdvertisingName);

  // Disable BLE pairing/bond requirements for zero-touch reconnect.
  // No user interaction should be needed after install.
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setDefaultPhy(BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK);

  // Set MTU to maximum for high-performance telemetry
  NimBLEDevice::setMTU(517);

  pServer = NimBLEDevice::createServer();
  static BleServerConnectionCallbacks serverCallbacks;
  pServer->setCallbacks(&serverCallbacks);
  if (gConnTuneTimer == nullptr) {
    gConnTuneTimer = xTimerCreate("bleConnTune", kConnTuneDelayTicks, pdFALSE,
                                  nullptr, applyPreferredLinkParams);
  }

  NimBLEAddress bleAddress = NimBLEDevice::getAddress();
  std::string uniqueId = bleAddress.toString();
  std::transform(
      uniqueId.begin(), uniqueId.end(), uniqueId.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  initConfigBleService(pServer, uniqueId);
  initFastLinkBleService(pServer);

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

void enterBLEPairingMode() {
  pairingModeActive = true;

  if (gPairingTimer == nullptr) {
    gPairingTimer =
        xTimerCreate("blePair", kPairingTimeoutTicks, pdFALSE, nullptr,
                     onPairingTimeout);
  }

  if (gPairingTimer != nullptr) {
    xTimerStop(gPairingTimer, 0);
    xTimerStart(gPairingTimer, 0);
  }

  USBSerial.println("[BLE] Pairing mode active for 60s");
  restartBLEAdvertising();
}
