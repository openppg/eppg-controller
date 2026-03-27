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
#include "sp140/ble/ota_service.h"
#if CONFIG_BT_NIMBLE_EXT_ADV
#include <NimBLEExtAdvertising.h>
#endif

namespace {

constexpr const char *kAdvertisingName = "OpenPPG";
constexpr TickType_t kConnTuneDelayTicks = pdMS_TO_TICKS(1200);
constexpr TickType_t kPairingTimeoutTicks = pdMS_TO_TICKS(60000);
constexpr TickType_t kAdvertisingWatchdogTicks = pdMS_TO_TICKS(1000);
TimerHandle_t gConnTuneTimer = nullptr;
TimerHandle_t gPairingTimer = nullptr;
TimerHandle_t gAdvertisingWatchdogTimer = nullptr;
bool pairingModeActive = false;
bool pairingModeTransitionActive = false;

// Store the active connection handle for conn param updates
uint16_t activeConnHandle = 0;

void stopPairingModeTimer() {
  if (gPairingTimer != nullptr) {
    xTimerStop(gPairingTimer, 0);
  }
}

size_t syncWhiteListFromBonds() {
  // Reconcile the whitelist to the current bond store. Advertising must be
  // stopped before calling this — the BLE controller rejects whitelist changes
  // while advertising with a filter (rc=524 / BLE_HS_EBUSY). Prune stale
  // entries first, then add any missing bonded addresses.
  for (size_t i = NimBLEDevice::getWhiteListCount(); i > 0; --i) {
    const NimBLEAddress addr = NimBLEDevice::getWhiteListAddress(i - 1);
    if (!NimBLEDevice::isBonded(addr)) {
      NimBLEDevice::whiteListRemove(addr);
    }
  }

  const int bondCount = NimBLEDevice::getNumBonds();
  for (int i = 0; i < bondCount; ++i) {
    const NimBLEAddress addr = NimBLEDevice::getBondedAddress(i);
    if (!NimBLEDevice::onWhiteList(addr)) {
      NimBLEDevice::whiteListAdd(addr);
    }
  }

  return NimBLEDevice::getWhiteListCount();
}

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

bool shouldAdvertiseWhilePowered() {
  return !pairingModeTransitionActive &&
         (pairingModeActive || NimBLEDevice::getNumBonds() > 0);
}

bool startAdvertising(NimBLEServer *server) {
  if (server == nullptr || pairingModeTransitionActive) {
    return false;
  }

  const size_t bondCount = static_cast<size_t>(NimBLEDevice::getNumBonds());
  const bool allowOpenAdvertising = pairingModeActive;

  // Stop advertising BEFORE modifying the whitelist — the BLE controller
  // rejects whitelist changes while advertising with a filter (BLE_HS_EBUSY).
  auto *advertising = server->getAdvertising();
  advertising->stop();

  const size_t whiteListCount = syncWhiteListFromBonds();

  if (!allowOpenAdvertising && bondCount == 0) {
    USBSerial.println(
        "[BLE] No bonds present and pairing mode inactive; advertising stopped");
    return false;
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
  adv.setMinInterval(32);  // 20ms (32 * 0.625ms)
  adv.setMaxInterval(48);  // 30ms (48 * 0.625ms)

  // Open advertising only during the explicit pairing window. Normal runtime
  // advertising only accepts bonded devices from the controller whitelist.
  if (allowOpenAdvertising) {
    adv.setScanFilter(false, false);
  } else {
    adv.setScanFilter(false, true);
  }

  // Flutter app's `startScan()` filters for CONFIG_SERVICE_UUID.
  adv.addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));

  // Scan response: manufacturer data with pairing-mode flag so the Flutter app
  // can hide non-pairable controllers from the connect list.
  // Format: Espressif company ID (0x02E5 LE) + 1 flag byte.
  NimBLEExtAdvertisement scanRsp(BLE_HCI_LE_PHY_1M, BLE_HCI_LE_PHY_1M);
  scanRsp.setLegacyAdvertising(true);
  scanRsp.setScannable(true);
  const uint8_t mfrData[] = {0xE5, 0x02,
                             static_cast<uint8_t>(allowOpenAdvertising ? 0x01 : 0x00)};
  scanRsp.setManufacturerData(mfrData, sizeof(mfrData));

  advertising->removeAll();
  const bool configured = advertising->setInstanceData(kExtAdvInstance, adv);
  const bool scanRspConfigured =
      configured ? advertising->setScanResponseData(kExtAdvInstance, scanRsp)
                 : false;
  const bool started =
      configured && scanRspConfigured && advertising->start(kExtAdvInstance);
  USBSerial.printf(
      "[BLE] Ext adv cfg=%d scanRsp=%d start=%d mode=%s bonds=%u wl=%u\n",
      configured, scanRspConfigured, started,
      allowOpenAdvertising ? "OPEN" : "BONDED",
      static_cast<unsigned>(bondCount), static_cast<unsigned>(whiteListCount));
  return started;
#else
  // Configure payload once — NimBLE accumulates addServiceUUID calls
  static bool payloadConfigured = false;
  if (!payloadConfigured) {
    advertising->setName(kAdvertisingName);
    advertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
    advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
    advertising->addServiceUUID(NimBLEUUID(CONFIG_SERVICE_UUID));
    advertising->enableScanResponse(true);
    advertising->setMinInterval(32);  // 20ms (32 * 0.625ms)
    advertising->setMaxInterval(48);  // 30ms (48 * 0.625ms)
    payloadConfigured = true;
  }

  // Open advertising only during the explicit pairing window. Normal runtime
  // advertising only accepts bonded devices from the controller whitelist.
  if (allowOpenAdvertising) {
    advertising->setScanFilter(false, false);
  } else {
    advertising->setScanFilter(false, true);
  }

  // Manufacturer data with pairing-mode flag (updated every restart).
  // Espressif company ID (0x02E5 LE) + 1 flag byte.
  const std::string mfrPayload = {'\xE5', '\x02',
                                  static_cast<char>(allowOpenAdvertising ? 0x01 : 0x00)};
  advertising->setManufacturerData(mfrPayload);

  const bool started = advertising->start();
  USBSerial.printf("[BLE] Legacy adv start=%s mode=%s bonds=%u whitelist=%u\n",
                   started ? "OK" : "FAIL",
                   allowOpenAdvertising ? "OPEN" : "BONDED",
                   static_cast<unsigned>(bondCount),
                   static_cast<unsigned>(whiteListCount));
  return started;
#endif
}

void onAdvertisingWatchdog(TimerHandle_t timer) {
  (void)timer;

  auto *server = pServer;
  if (server == nullptr || deviceConnected || !shouldAdvertiseWhilePowered()) {
    return;
  }

  auto *advertising = server->getAdvertising();
  if (advertising != nullptr && advertising->isAdvertising()) {
    return;
  }

  USBSerial.println(
      "[BLE] Advertising inactive while powered and disconnected; retrying");
  const bool started = startAdvertising(server);
  if (!started) {
    USBSerial.println("[BLE] Advertising retry did not start; watchdog will continue");
  }
}

class BleServerConnectionCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    activeConnHandle = connInfo.getConnHandle();
    deviceConnected = true;
    connectedHandle = connInfo.getConnHandle();

    if (gConnTuneTimer != nullptr) {
      xTimerStop(gConnTuneTimer, 0);
      xTimerStart(gConnTuneTimer, 0);
    }
    USBSerial.printf(
        "Device connected handle=%u addr=%s bonded=%d encrypted=%d pairing=%d\n",
        connectedHandle, connInfo.getAddress().toString().c_str(),
        connInfo.isBonded() ? 1 : 0, connInfo.isEncrypted() ? 1 : 0,
        pairingModeActive ? 1 : 0);

    // During pairing mode, proactively request fresh security negotiation.
    // This helps recover from stale iOS bonds where iOS tries to restore
    // encryption with keys the controller no longer has (rc=19 failures).
    if (pairingModeActive && !connInfo.isEncrypted()) {
      USBSerial.println("[BLE] Pairing mode: requesting fresh security exchange");
      ble_gap_security_initiate(connInfo.getConnHandle());
    }
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo,
                    int reason) override {
    (void)connInfo;
    if (gConnTuneTimer != nullptr) {
      xTimerStop(gConnTuneTimer, 0);
    }
    deviceConnected = false;
    connectedHandle = BLE_HS_CONN_HANDLE_NONE;

    if (isOtaInProgress()) {
      abortOta();
      USBSerial.println("OTA aborted due to disconnect");
    }

    USBSerial.printf("Device disconnected reason=%d\n", reason);

    // Suppress the immediate advertising restart during a pairing transition —
    // enterBLEPairingMode() issues its own startAdvertising after clearing bonds.
    if (!pairingModeTransitionActive) {
      startAdvertising(server);
    }
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
    const NimBLEAddress identityAddress = connInfo.getIdAddress();
    if (connInfo.isBonded()) {
      NimBLEDevice::whiteListAdd(identityAddress);
      if (pairingModeActive) {
        pairingModeActive = false;
        stopPairingModeTimer();
      }
    }

    USBSerial.printf(
        "[BLE] Auth complete addr=%s id=%s bonded=%d encrypted=%d keySize=%u\n",
        connInfo.getAddress().toString().c_str(),
        identityAddress.toString().c_str(), connInfo.isBonded() ? 1 : 0,
        connInfo.isEncrypted() ? 1 : 0, connInfo.getSecKeySize());

    if (!connInfo.isEncrypted() || !connInfo.isBonded()) {
      if (pairingModeActive) {
        // During pairing mode, a failed auth likely means the phone has a stale
        // bond (e.g. iOS cached old encryption keys).  Delete any peer data we
        // have and request fresh pairing instead of rejecting outright.
        const NimBLEAddress peerAddr = connInfo.getAddress();
        USBSerial.printf("[BLE] Pairing mode: auth failed for addr=%s id=%s, "
                         "requesting fresh pairing\n",
                         peerAddr.toString().c_str(),
                         identityAddress.toString().c_str());
        NimBLEDevice::deleteBond(identityAddress);
        ble_gap_security_initiate(connInfo.getConnHandle());
      } else {
        USBSerial.println("[BLE] Rejecting untrusted BLE session");
        if (pServer != nullptr) {
          pServer->disconnect(connInfo.getConnHandle());
        }
      }
    }
  }

  void onIdentity(NimBLEConnInfo &connInfo) override {
    const NimBLEAddress identityAddress = connInfo.getIdAddress();
    NimBLEDevice::whiteListAdd(identityAddress);
    USBSerial.printf("[BLE] Identity resolved addr=%s\n",
                     identityAddress.toString().c_str());
  }
};

}  // namespace

void setupBLE() {
  // Initialize NimBLE with device name
  NimBLEDevice::init(kAdvertisingName);

  // Require bonded LE Secure Connections. The controller has no input/output,
  // so pairing stays frictionless ("Just Works") while reconnects restore an
  // encrypted trusted link automatically.
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setDefaultPhy(BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK);

  // Set MTU to maximum for high-performance telemetry
  NimBLEDevice::setMTU(517);

  // Set TX power to +9dBm for reliable connections at distance
  NimBLEDevice::setPowerLevel(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  static BleServerConnectionCallbacks serverCallbacks;
  pServer->setCallbacks(&serverCallbacks);
  pServer->advertiseOnDisconnect(false);
  if (gConnTuneTimer == nullptr) {
    gConnTuneTimer = xTimerCreate("bleConnTune", kConnTuneDelayTicks, pdFALSE,
                                  nullptr, applyPreferredLinkParams);
  }
  if (gAdvertisingWatchdogTimer == nullptr) {
    gAdvertisingWatchdogTimer =
        xTimerCreate("bleAdvWatch", kAdvertisingWatchdogTicks, pdTRUE, nullptr,
                     onAdvertisingWatchdog);
  }
  if (gAdvertisingWatchdogTimer != nullptr) {
    xTimerStart(gAdvertisingWatchdogTimer, 0);
  }

  NimBLEAddress bleAddress = NimBLEDevice::getAddress();
  std::string uniqueId = bleAddress.toString();
  std::transform(
      uniqueId.begin(), uniqueId.end(), uniqueId.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  initConfigBleService(pServer, uniqueId);
  initFastLinkBleService(pServer);
  initOtaBleService(pServer);

  USBSerial.println("BLE device ready");
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

  startAdvertising(pServer);
}

void enterBLEPairingMode() {
  // Block advertising restarts (e.g. from onDisconnect) during this transition.
  pairingModeTransitionActive = true;

  // Single-bond model: disconnect the current peer so we can safely clear bonds.
  if (deviceConnected && pServer != nullptr &&
      connectedHandle != BLE_HS_CONN_HANDLE_NONE) {
    pServer->disconnect(connectedHandle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Stop advertising before modifying bonds — NimBLE rejects bond deletion
  // while the controller is advertising (BLE_HS_EBUSY).
  if (pServer != nullptr) {
    auto *adv = pServer->getAdvertising();
    if (adv != nullptr) {
      adv->stop();
    }
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  const int bondCount = NimBLEDevice::getNumBonds();
  bool cleared = false;
  if (bondCount > 0) {
    cleared = NimBLEDevice::deleteAllBonds();
    if (!cleared) {
      for (int i = NimBLEDevice::getNumBonds() - 1; i >= 0; --i) {
        NimBLEDevice::deleteBond(NimBLEDevice::getBondedAddress(i));
      }
      cleared = NimBLEDevice::getNumBonds() == 0;
    }
  }
  USBSerial.printf("[BLE] Cleared bonds: %s (was %d, now %d)\n",
                   cleared ? "OK" : (bondCount == 0 ? "NONE" : "FAILED"),
                   bondCount, NimBLEDevice::getNumBonds());

  pairingModeActive = true;
  pairingModeTransitionActive = false;

  if (gPairingTimer == nullptr) {
    gPairingTimer = xTimerCreate("blePair", kPairingTimeoutTicks, pdFALSE,
                                 nullptr, onPairingTimeout);
  }

  if (gPairingTimer != nullptr) {
    stopPairingModeTimer();
    xTimerStart(gPairingTimer, 0);
  }

  USBSerial.println("[BLE] Pairing mode active for 60s");
  restartBLEAdvertising();
}
