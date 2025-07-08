#include "BLEOTAManager.h"
#include <esp_log.h>

static const char* TAG = "BLE_OTA";

BLEOTAManager::BLEOTAManager() {
  pServer = nullptr;
  pService = nullptr;
  pControlChar = nullptr;
  pDataChar = nullptr;
  pStatusChar = nullptr;
  
  otaHandle = 0;
  otaPartition = nullptr;
  
  currentStatus = OTA_STATUS_IDLE;
  receivedBytes = 0;
  totalBytes = 0;
  otaActive = false;
  deviceConnected = false;
  
  // Initialize callback objects
  pControlCallbacks = new OTAControlCallbacks(this);
  pDataCallbacks = new OTADataCallbacks(this);
  pServerCallbacks = new OTAServerCallbacks(this);
}

BLEOTAManager::~BLEOTAManager() {
  if (otaActive) {
    abortOTA();
  }
  
  delete pControlCallbacks;
  delete pDataCallbacks;
  delete pServerCallbacks;
}

bool BLEOTAManager::init(const char* deviceName) {
  logInfo("Initializing BLE OTA Manager");
  
  // Initialize BLE
  BLEDevice::init(deviceName);
  
  // Create BLE server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(pServerCallbacks);
  
  // Create BLE service
  pService = pServer->createService(BLE_OTA_SERVICE_UUID);
  
  // Create Control characteristic
  pControlChar = pService->createCharacteristic(
    BLE_OTA_CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pControlChar->setCallbacks(pControlCallbacks);
  
  // Create Data characteristic
  pDataChar = pService->createCharacteristic(
    BLE_OTA_DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pDataChar->setCallbacks(pDataCallbacks);
  
  // Create Status characteristic
  pStatusChar = pService->createCharacteristic(
    BLE_OTA_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->addDescriptor(new BLE2902());
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_OTA_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  logInfo("BLE OTA Service started. Waiting for connections...");
  
  updateStatus(OTA_STATUS_IDLE);
  
  return true;
}

void BLEOTAManager::loop() {
  // Handle BLE events and maintain connection
  if (deviceConnected) {
    // Connection is active, handle any background tasks
    delay(10);
  } else {
    // No connection, ensure advertising is active
    if (!BLEDevice::getAdvertising()->isAdvertising()) {
      BLEDevice::startAdvertising();
    }
    delay(100);
  }
}

bool BLEOTAManager::startOTA() {
  logInfo("Starting OTA process");
  
  if (otaActive) {
    logError("OTA already active");
    return false;
  }
  
  // Get the next OTA partition
  otaPartition = esp_ota_get_next_update_partition(nullptr);
  if (!otaPartition) {
    logError("No OTA partition available");
    updateStatus(OTA_STATUS_ERROR);
    return false;
  }
  
  // Begin OTA
  esp_err_t err = esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle);
  if (err != ESP_OK) {
    logError("esp_ota_begin failed");
    updateStatus(OTA_STATUS_ERROR);
    return false;
  }
  
  otaActive = true;
  receivedBytes = 0;
  totalBytes = 0;
  
  updateStatus(OTA_STATUS_ACTIVE);
  
  logInfo("OTA started successfully");
  return true;
}

bool BLEOTAManager::writeOTAData(uint8_t* data, size_t length) {
  if (!otaActive) {
    logError("OTA not active");
    return false;
  }
  
  if (length == 0) {
    return true;
  }
  
  esp_err_t err = esp_ota_write(otaHandle, data, length);
  if (err != ESP_OK) {
    logError("esp_ota_write failed");
    abortOTA();
    return false;
  }
  
  receivedBytes += length;
  
  // Log progress every 10KB
  if (receivedBytes % 10240 == 0) {
    char msg[100];
    snprintf(msg, sizeof(msg), "OTA progress: %u bytes received", receivedBytes);
    logInfo(msg);
  }
  
  return true;
}

bool BLEOTAManager::finishOTA() {
  logInfo("Finishing OTA process");
  
  if (!otaActive) {
    logError("OTA not active");
    return false;
  }
  
  // End OTA
  esp_err_t err = esp_ota_end(otaHandle);
  if (err != ESP_OK) {
    logError("esp_ota_end failed");
    abortOTA();
    return false;
  }
  
  // Set boot partition
  err = esp_ota_set_boot_partition(otaPartition);
  if (err != ESP_OK) {
    logError("esp_ota_set_boot_partition failed");
    abortOTA();
    return false;
  }
  
  otaActive = false;
  otaHandle = 0;
  otaPartition = nullptr;
  
  updateStatus(OTA_STATUS_SUCCESS);
  
  logInfo("OTA completed successfully");
  return true;
}

void BLEOTAManager::abortOTA() {
  logInfo("Aborting OTA process");
  
  if (otaActive) {
    esp_ota_abort(otaHandle);
    otaActive = false;
    otaHandle = 0;
    otaPartition = nullptr;
    receivedBytes = 0;
    totalBytes = 0;
  }
  
  updateStatus(OTA_STATUS_ABORTED);
}

void BLEOTAManager::resetDevice() {
  logInfo("Resetting device");
  delay(1000);
  ESP.restart();
}

void BLEOTAManager::updateStatus(uint8_t status) {
  currentStatus = status;
  
  // Update BLE characteristic
  if (pStatusChar) {
    pStatusChar->setValue(&currentStatus, 1);
    pStatusChar->notify();
  }
}

void BLEOTAManager::logError(const char* message) {
  ESP_LOGE(TAG, "%s", message);
  Serial.println(String("[ERROR] ") + message);
}

void BLEOTAManager::logInfo(const char* message) {
  ESP_LOGI(TAG, "%s", message);
  Serial.println(String("[INFO] ") + message);
}

// Control Callbacks Implementation
void OTAControlCallbacks::onWrite(BLECharacteristic* pCharacteristic) {
  std::string value = pCharacteristic->getValue();
  
  if (value.length() > 0) {
    uint8_t cmd = value[0];
    
    switch (cmd) {
      case OTA_CMD_START:
        pOTAManager->logInfo("Received START command");
        pOTAManager->startOTA();
        break;
        
      case OTA_CMD_ABORT:
        pOTAManager->logInfo("Received ABORT command");
        pOTAManager->abortOTA();
        break;
        
      case OTA_CMD_FINISH:
        pOTAManager->logInfo("Received FINISH command");
        pOTAManager->finishOTA();
        break;
        
      case OTA_CMD_RESET:
        pOTAManager->logInfo("Received RESET command");
        pOTAManager->resetDevice();
        break;
        
      default:
        pOTAManager->logError("Unknown OTA command");
        break;
    }
  }
}

// Data Callbacks Implementation
void OTADataCallbacks::onWrite(BLECharacteristic* pCharacteristic) {
  std::string value = pCharacteristic->getValue();
  
  if (value.length() > 0) {
    uint8_t* data = (uint8_t*)value.c_str();
    size_t length = value.length();
    
    if (!pOTAManager->writeOTAData(data, length)) {
      pOTAManager->logError("Failed to write OTA data");
    }
  }
}

// Server Callbacks Implementation
void OTAServerCallbacks::onConnect(BLEServer* pServer) {
  pOTAManager->logInfo("Device connected");
  pOTAManager->setDeviceConnected(true);
}

void OTAServerCallbacks::onDisconnect(BLEServer* pServer) {
  pOTAManager->logInfo("Device disconnected");
  pOTAManager->setDeviceConnected(false);
  
  // Abort OTA if active
  if (pOTAManager->isOTAActive()) {
    pOTAManager->abortOTA();
  }
  
  // Restart advertising
  delay(500);
  BLEDevice::startAdvertising();
}