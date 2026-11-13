#include "sp140/ble/ble_core.h"

#include <atomic>
#include <cstdio>
#include <string>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "host/ble_sm.h"
#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/config_service.h"
#include "sp140/ble/fastlink_service.h"
#include "sp140/ble/ota_service.h"

namespace {
constexpr char kTag[] = "controller_ble";
std::atomic<bool> pairingActive{false};
std::atomic<bool> pairingTransition{false};
TimerHandle_t pairingTimer = nullptr;
TimerHandle_t tuningTimer = nullptr;
TimerHandle_t advertisingTimer = nullptr;

std::string advertisingName(std::string* uniqueId) {
  uint8_t mac[6]{};
  if (esp_read_mac(mac, ESP_MAC_BT) != ESP_OK) return "OpenPPG SP140";
  char name[32];
  std::snprintf(name, sizeof(name), "OpenPPG SP140 [%02X%02X]", mac[4], mac[5]);
  char id[18];
  std::snprintf(id, sizeof(id), "%02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  *uniqueId = id;
  return name;
}
bool startAdvertising() {
  if (!pServer || !pServer->ready() || pairingTransition || deviceConnected) return false;
  pServer->setPairingAllowed(pairingActive);
  return pServer->advertise(platform::BleUuid(CONFIG_SERVICE_UUID), pairingActive);
}
void pairingExpired(TimerHandle_t) {
  pairingActive = false;
  if (pServer) pServer->setPairingAllowed(false);
  startAdvertising();
}
void tuneConnection(TimerHandle_t) {
  if (pServer && deviceConnected && connectedHandle != BLE_HS_CONN_HANDLE_NONE)
    pServer->updateConnParams(connectedHandle, 12, 24, 0, 200);
}
void advertisingWatchdog(TimerHandle_t) {
  if (!deviceConnected && pServer && !pServer->isAdvertising()) startAdvertising();
}
class ConnectionCallbacks : public platform::BleServerCallbacks {
  void onConnect(platform::BleServer*, platform::BleConnection& connection) override {
    connectedHandle = connection.getConnHandle();
    deviceConnected = true;
    xTimerReset(tuningTimer, 0);
    ESP_LOGI(kTag, "Connected handle=%u bonded=%d encrypted=%d", connectedHandle,
             connection.isBonded(), connection.isEncrypted());
    // The central initiates security. A proactive peripheral request can cause
    // a second Android pairing dialog; retry only after failed authentication.
  }
  void onDisconnect(platform::BleServer*, platform::BleConnection&, int reason) override {
    xTimerStop(tuningTimer, 0);
    deviceConnected = false;
    connectedHandle = BLE_HS_CONN_HANDLE_NONE;
    if (isOtaInProgress()) abortOta();
    ESP_LOGI(kTag, "Disconnected reason=%d", reason);
    if (!pairingTransition) startAdvertising();
  }
  void onAuthenticationComplete(platform::BleConnection& connection) override {
    if (connection.isEncrypted() && connection.isBonded()) {
      pairingActive = false;
      pServer->setPairingAllowed(false);
      xTimerStop(pairingTimer, 0);
    } else if (pairingActive) {
      pServer->deleteBond(connection.getIdAddress());
      ble_gap_security_initiate(connection.getConnHandle());
    } else {
      pServer->disconnect(connection.getConnHandle());
    }
  }
  void onMTUChange(uint16_t mtu, platform::BleConnection& connection) override {
    ESP_LOGI(kTag, "MTU=%u handle=%u", mtu, connection.getConnHandle());
  }
};
}

void setupBLE() {
  std::string uniqueId;
  const auto name = advertisingName(&uniqueId);
  static platform::BleServer server;
  static ConnectionCallbacks callbacks;
  pServer = &server;
  pServer->setCallbacks(&callbacks);
  pairingTimer = xTimerCreate("blePair", pdMS_TO_TICKS(60000), pdFALSE, nullptr, pairingExpired);
  tuningTimer = xTimerCreate("bleTune", pdMS_TO_TICKS(1200), pdFALSE, nullptr, tuneConnection);
  advertisingTimer = xTimerCreate("bleAdv", pdMS_TO_TICKS(1000), pdTRUE, nullptr, advertisingWatchdog);
  if (!pairingTimer || !tuningTimer || !advertisingTimer) {
    ESP_LOGE(kTag, "Cannot allocate BLE timers");
    return;
  }
  initConfigBleService(pServer, uniqueId);
  initFastLinkBleService(pServer);
  initOtaBleService(pServer);
  if (!pServer->start(name)) {
    ESP_LOGE(kTag, "Cannot initialize native NimBLE host");
    return;
  }
  xTimerStart(advertisingTimer, 0);
#ifdef BLE_PAIR_ON_BOOT
  enterBLEPairingMode();
#endif
}
void restartBLEAdvertising() { startAdvertising(); }
void requestFastConnParams() {
  if (pServer && deviceConnected)
    pServer->updateConnParams(connectedHandle, 12, 12, 0, 800);
}
void requestNormalConnParams() {
  if (pServer && deviceConnected)
    pServer->updateConnParams(connectedHandle, 24, 40, 0, 400);
}
void enterBLEPairingMode() {
  if (!pServer || !pServer->ready()) return;
  if (pairingTransition.exchange(true)) return;
  pServer->stopAdvertising();
  if (deviceConnected) pServer->disconnect(connectedHandle);
  // Do not modify the bond store while a connection is tearing down. Retry
  // on a dedicated worker rather than blocking a FreeRTOS timer service task.
  const BaseType_t created = xTaskCreate([](void*) {
    for (unsigned i = 0; deviceConnected && i < 100; ++i)
      vTaskDelay(pdMS_TO_TICKS(10));
    if (!deviceConnected && pServer->clearBonds()) {
      pairingActive = true;
      pServer->setPairingAllowed(true);
      xTimerReset(pairingTimer, 0);
    } else {
      ESP_LOGE(kTag, "Cannot enter pairing mode: disconnect/bond erase failed");
    }
    pairingTransition = false;
    startAdvertising();
    vTaskDelete(nullptr);
  }, "blePairClear", 4096, nullptr, 1, nullptr);
  if (created != pdPASS) {
    pairingTransition = false;
    ESP_LOGE(kTag, "Cannot allocate pairing worker");
    startAdvertising();
  }
}
bool isBLEPairingModeActive() { return pairingActive.load(); }
