#ifndef BLE_OTA_MANAGER_H
#define BLE_OTA_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// BLE OTA Service and Characteristic UUIDs
#define BLE_OTA_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_OTA_CONTROL_CHAR_UUID   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_OTA_DATA_CHAR_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_OTA_STATUS_CHAR_UUID    "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

// OTA Control Commands
#define OTA_CMD_START    0x01
#define OTA_CMD_ABORT    0x02
#define OTA_CMD_FINISH   0x03
#define OTA_CMD_RESET    0x04

// OTA Status Values
#define OTA_STATUS_IDLE      0x00
#define OTA_STATUS_ACTIVE    0x01
#define OTA_STATUS_SUCCESS   0x02
#define OTA_STATUS_ERROR     0x03
#define OTA_STATUS_ABORTED   0x04

class BLEOTAManager;

// Forward declarations
class OTAControlCallbacks;
class OTADataCallbacks;
class OTAServerCallbacks;

class BLEOTAManager {
private:
  BLEServer* pServer;
  BLEService* pService;
  BLECharacteristic* pControlChar;
  BLECharacteristic* pDataChar;
  BLECharacteristic* pStatusChar;
  
  esp_ota_handle_t otaHandle;
  const esp_partition_t* otaPartition;
  
  uint8_t currentStatus;
  uint32_t receivedBytes;
  uint32_t totalBytes;
  bool otaActive;
  bool deviceConnected;
  
  // Callback objects
  OTAControlCallbacks* pControlCallbacks;
  OTADataCallbacks* pDataCallbacks;
  OTAServerCallbacks* pServerCallbacks;
  
  void updateStatus(uint8_t status);
  void logError(const char* message);
  void logInfo(const char* message);

public:
  BLEOTAManager();
  ~BLEOTAManager();
  
  bool init(const char* deviceName = "ESP32S3-OTA");
  void loop();
  
  // OTA Operations
  bool startOTA();
  bool writeOTAData(uint8_t* data, size_t length);
  bool finishOTA();
  void abortOTA();
  void resetDevice();
  
  // Status and Info
  uint8_t getStatus() const { return currentStatus; }
  uint32_t getReceivedBytes() const { return receivedBytes; }
  uint32_t getTotalBytes() const { return totalBytes; }
  bool isOTAActive() const { return otaActive; }
  bool isDeviceConnected() const { return deviceConnected; }
  
  // Callback setters
  void setDeviceConnected(bool connected) { deviceConnected = connected; }
  
  // Friends for callback access
  friend class OTAControlCallbacks;
  friend class OTADataCallbacks;
  friend class OTAServerCallbacks;
};

// Callback Classes
class OTAControlCallbacks : public BLECharacteristicCallbacks {
private:
  BLEOTAManager* pOTAManager;
  
public:
  OTAControlCallbacks(BLEOTAManager* manager) : pOTAManager(manager) {}
  void onWrite(BLECharacteristic* pCharacteristic) override;
};

class OTADataCallbacks : public BLECharacteristicCallbacks {
private:
  BLEOTAManager* pOTAManager;
  
public:
  OTADataCallbacks(BLEOTAManager* manager) : pOTAManager(manager) {}
  void onWrite(BLECharacteristic* pCharacteristic) override;
};

class OTAServerCallbacks : public BLEServerCallbacks {
private:
  BLEOTAManager* pOTAManager;
  
public:
  OTAServerCallbacks(BLEOTAManager* manager) : pOTAManager(manager) {}
  void onConnect(BLEServer* pServer) override;
  void onDisconnect(BLEServer* pServer) override;
};

#endif // BLE_OTA_MANAGER_H