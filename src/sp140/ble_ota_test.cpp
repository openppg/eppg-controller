#include <Arduino.h>

#ifdef ENABLE_BLE_OTA

#include "BLEOTAManager.h"
#include "sp140/debug.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

// Define DEBUG_PRINT macro for BLE OTA debugging
#if CORE_DEBUG_LEVEL >= 3
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

// Global BLE OTA Manager instance
BLEOTAManager* bleOTAManager = nullptr;

// Initialize BLE OTA
void setupBLEOTA() {
  DEBUG_PRINT("Setting up BLE OTA...");

  bleOTAManager = new BLEOTAManager();

  // Initialize with a unique device name
  String deviceName = "OpenPPG-SP140-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (bleOTAManager->init(deviceName.c_str())) {
    DEBUG_PRINT("BLE OTA initialized successfully");
    DEBUG_PRINT("Device name: " + deviceName);
    DEBUG_PRINT("Service UUID: " + String(BLE_OTA_SERVICE_UUID));
  } else {
    DEBUG_PRINT("Failed to initialize BLE OTA");
  }
}

// Loop function for BLE OTA
void loopBLEOTA() {
  if (bleOTAManager) {
    bleOTAManager->loop();

    // Optional: Print status updates
    static unsigned long lastStatusPrint = 0;
    static uint8_t lastStatus = 0xFF;

    if (millis() - lastStatusPrint > 5000) { // Every 5 seconds
      uint8_t currentStatus = bleOTAManager->getStatus();
      bool isConnected = bleOTAManager->isDeviceConnected();

      if (currentStatus != lastStatus || isConnected) {
        DEBUG_PRINT("BLE OTA Status: " + String(currentStatus) +
                   " | Connected: " + String(isConnected ? "Yes" : "No"));

        if (bleOTAManager->isOTAActive()) {
          DEBUG_PRINT("OTA Progress: " + String(bleOTAManager->getReceivedBytes()) + " bytes received");
        }

        lastStatus = currentStatus;
        lastStatusPrint = millis();
      }
    }
  }
}

// Get BLE OTA status for integration with main app
bool isBLEOTAActive() {
  return bleOTAManager && bleOTAManager->isOTAActive();
}

// Get BLE connection status
bool isBLEOTAConnected() {
  return bleOTAManager && bleOTAManager->isDeviceConnected();
}

// Cleanup BLE OTA
void cleanupBLEOTA() {
  if (bleOTAManager) {
    delete bleOTAManager;
    bleOTAManager = nullptr;
  }
}

// Print OTA partition information
void printOTAInfo() {
  DEBUG_PRINT("=== OTA Partition Information ===");

  // Get current running partition
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running) {
    DEBUG_PRINT("Current partition: " + String(running->label));
    DEBUG_PRINT("Address: 0x" + String(running->address, HEX));
    DEBUG_PRINT("Size: " + String(running->size / 1024) + " KB");
  }

  // Get next update partition
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  if (next) {
    DEBUG_PRINT("Next update partition: " + String(next->label));
    DEBUG_PRINT("Address: 0x" + String(next->address, HEX));
    DEBUG_PRINT("Size: " + String(next->size / 1024) + " KB");
  }

  // Get OTA data partition
  const esp_partition_t* otadata = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr);
  if (otadata) {
    DEBUG_PRINT("OTA data partition: " + String(otadata->label));
    DEBUG_PRINT("Address: 0x" + String(otadata->address, HEX));
    DEBUG_PRINT("Size: " + String(otadata->size) + " bytes");
  }

  DEBUG_PRINT("================================");
}

// Test OTA partition validity
bool testOTAPartitions() {
  DEBUG_PRINT("Testing OTA partitions...");

  // Check if we have the required partitions
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  const esp_partition_t* otadata = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr);

  if (!running) {
    DEBUG_PRINT("ERROR: No running partition found!");
    return false;
  }

  if (!next) {
    DEBUG_PRINT("ERROR: No next update partition found!");
    return false;
  }

  if (!otadata) {
    DEBUG_PRINT("ERROR: No OTA data partition found!");
    return false;
  }

  // Check partition sizes
  if (running->size < 1024 * 1024) { // Less than 1MB
    DEBUG_PRINT("WARNING: Running partition is smaller than 1MB");
  }

  if (next->size < 1024 * 1024) { // Less than 1MB
    DEBUG_PRINT("WARNING: Next partition is smaller than 1MB");
  }

  if (otadata->size < 0x2000) { // Less than 8KB
    DEBUG_PRINT("ERROR: OTA data partition is too small!");
    return false;
  }

  DEBUG_PRINT("OTA partitions test passed!");
  return true;
}

// Get current firmware version/build info
void printFirmwareInfo() {
  DEBUG_PRINT("=== Firmware Information ===");

  // Get Arduino-compatible system information
  DEBUG_PRINT("Project: OpenPPG-SP140");
  DEBUG_PRINT("Compile time: " + String(__TIME__));
  DEBUG_PRINT("Compile date: " + String(__DATE__));
  DEBUG_PRINT("SDK version: " + String(ESP.getSdkVersion()));

  // Get chip info using Arduino ESP class
  DEBUG_PRINT("Chip: ESP32-S3");
  DEBUG_PRINT("CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
  DEBUG_PRINT("Flash size: " + String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB");
  DEBUG_PRINT("Flash speed: " + String(ESP.getFlashChipSpeed() / 1000000) + " MHz");
  DEBUG_PRINT("Sketch size: " + String(ESP.getSketchSize()) + " bytes");
  DEBUG_PRINT("Free sketch space: " + String(ESP.getFreeSketchSpace()) + " bytes");

  // Memory information
  DEBUG_PRINT("Heap size: " + String(ESP.getHeapSize()) + " bytes");
  DEBUG_PRINT("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
  DEBUG_PRINT("Min free heap: " + String(ESP.getMinFreeHeap()) + " bytes");
  DEBUG_PRINT("Max alloc heap: " + String(ESP.getMaxAllocHeap()) + " bytes");

  // PSRAM information (if available)
  if (ESP.getPsramSize() > 0) {
    DEBUG_PRINT("PSRAM size: " + String(ESP.getPsramSize()) + " bytes");
    DEBUG_PRINT("Free PSRAM: " + String(ESP.getFreePsram()) + " bytes");
    DEBUG_PRINT("Min free PSRAM: " + String(ESP.getMinFreePsram()) + " bytes");
    DEBUG_PRINT("Max alloc PSRAM: " + String(ESP.getMaxAllocPsram()) + " bytes");
  }

  DEBUG_PRINT("=============================");
}

#endif // ENABLE_BLE_OTA
