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
constexpr TickType_t kConnTuneDelayTicks = pdMS_TO_TICKS(180);
constexpr TickType_t kPairingTimeoutTicks = pdMS_TO_TICKS(60000);
TimerHandle_t gConnTuneTimer = nullptr;
TimerHandle_t gPairingTimer = nullptr;
bool pairingModeActive = false;
NimBLEAddress lastConnectedAddr;  // Track last connected device for directed adv
bool hasLastConnectedAddr = false;
TaskHandle_t reconnectAdvTaskHandle = nullptr;

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

  server->updateConnParams(handle, 12, 24, 0, 200);
  server->setDataLen(handle, 251);
  const bool phyOk = server->updatePhy(handle, BLE_GAP_LE_PHY_2M_MASK,
                                       BLE_GAP_LE_PHY_2M_MASK, 0);
  USBSerial.printf("[BLE] Link tune handle=%u phy2m=%s\n", handle,
                   phyOk ? "OK" : "FAIL");
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

  // Whitelisting: If we have bonds and pairing mode is not active,
  // only allow bonded devices to connect.
  size_t bondCount = NimBLEDevice::getNumBonds();
  if (bondCount > 0 && !pairingModeActive) {
    for (size_t i = 0; i < bondCount; ++i) {
      NimBLEDevice::whiteListAdd(NimBLEDevice::getBondedAddress(i));
    }
    adv.setScanFilter(false, true); // Only whitelisted devices can connect
  } else {
    // Open advertising: no bonds yet, or pairing mode is active
    adv.setScanFilter(false, false);
    size_t wlCount = NimBLEDevice::getWhiteListCount();
    for (size_t i = 0; i < wlCount; i++) {
      NimBLEDevice::whiteListRemove(NimBLEDevice::getWhiteListAddress(0));
    }
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

  // Update whitelist on every call (bonds/pairing state may change)
  size_t bondCount = NimBLEDevice::getNumBonds();
  if (bondCount > 0 && !pairingModeActive) {
    for (size_t i = 0; i < bondCount; ++i) {
      NimBLEDevice::whiteListAdd(NimBLEDevice::getBondedAddress(i));
    }
    advertising->setScanFilter(false, true);
  } else {
    advertising->setScanFilter(false, false);
    size_t wlCount = NimBLEDevice::getWhiteListCount();
    for (size_t i = 0; i < wlCount; i++) {
      NimBLEDevice::whiteListRemove(NimBLEDevice::getWhiteListAddress(0));
    }
  }

  const bool started = advertising->start();
  USBSerial.printf("[BLE] Legacy adv start=%s (bonds: %u)\n",
                   started ? "OK" : "FAIL", bondCount);
#endif
}

// Persistent reconnect advertising task.
// Refreshes advertising every 5s until reconnected.
void reconnectAdvTask(void *pvParameters) {
  (void)pvParameters;
  USBSerial.println("[BLE] Reconnect adv task started");

  // Undirected legacy advertising with service UUID.
  // Refresh every 5s — advertising can silently stop if a brief L2CAP
  // connection attempt interrupts it, so periodic restart is essential.
  startAdvertising(pServer);

  for (;;) {
    if (deviceConnected || pairingModeActive || pServer == nullptr) {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    if (deviceConnected || pairingModeActive) break;

    startAdvertising(pServer);
  }

  USBSerial.println("[BLE] Reconnect adv task ended");
  reconnectAdvTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

class BleServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    deviceConnected = true;
    connectedHandle = connInfo.getConnHandle();
    lastConnectedAddr = connInfo.getAddress();
    hasLastConnectedAddr = true;

    // Stop the reconnect advertising task if it's running
    if (reconnectAdvTaskHandle != nullptr) {
      vTaskDelete(reconnectAdvTaskHandle);
      reconnectAdvTaskHandle = nullptr;
    }

    if (gConnTuneTimer != nullptr) {
      xTimerStop(gConnTuneTimer, 0);
      xTimerStart(gConnTuneTimer, 0);
    }
    USBSerial.printf("Device connected handle=%u addr=%s\n", connectedHandle,
                     lastConnectedAddr.toString().c_str());
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

    // Start persistent reconnect advertising task.
    // Refreshes advertising every 5s until reconnected.
    if (hasLastConnectedAddr && !pairingModeActive &&
        reconnectAdvTaskHandle == nullptr) {
      xTaskCreate(reconnectAdvTask, "BLEReconAdv", 3072, nullptr, 1,
                  &reconnectAdvTaskHandle);
    } else {
      // Fallback: just start normal advertising
      startAdvertising(server);
    }
  }
};

} // namespace

void setupBLE() {
  // Initialize NimBLE with device name
  NimBLEDevice::init(kAdvertisingName);

  // Bonding and LE secure connections ("Just Works").
  NimBLEDevice::setSecurityAuth(true, false, true);
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
