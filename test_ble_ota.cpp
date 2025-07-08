/*
 * BLE OTA Test Application
 * 
 * This is a standalone test application for ESP32S3 BLE OTA functionality.
 * It demonstrates how to use the BLEOTAManager class for over-the-air updates.
 * 
 * To use this test:
 * 1. Flash this firmware to your ESP32S3
 * 2. Connect via BLE using a BLE scanner app (like nRF Connect)
 * 3. Look for service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * 4. Use the control characteristic to start OTA (write 0x01)
 * 5. Send firmware data via the data characteristic
 * 6. Finish OTA (write 0x03) and the device will reboot
 */

#include <Arduino.h>
#include "BLEOTAManager.h"

// Global BLE OTA Manager instance
BLEOTAManager bleOTAManager;

// Test application state
unsigned long lastHeartbeat = 0;
int heartbeatCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("==================================");
  Serial.println("ESP32S3 BLE OTA Test Application");
  Serial.println("==================================");
  
  // Print system information
  printSystemInfo();
  
  // Test OTA partitions
  if (!testOTAPartitions()) {
    Serial.println("ERROR: OTA partition test failed!");
    Serial.println("Check your partition table configuration.");
    while (1) {
      delay(1000);
    }
  }
  
  // Initialize BLE OTA
  Serial.println("Initializing BLE OTA...");
  String deviceName = "ESP32S3-BLE-OTA-Test";
  
  if (bleOTAManager.init(deviceName.c_str())) {
    Serial.println("BLE OTA initialized successfully!");
    Serial.println("Device name: " + deviceName);
    Serial.println("Service UUID: " + String(BLE_OTA_SERVICE_UUID));
    Serial.println("");
    Serial.println("Ready for BLE connections!");
    Serial.println("Use nRF Connect or similar app to connect.");
  } else {
    Serial.println("Failed to initialize BLE OTA!");
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  // Handle BLE OTA
  bleOTAManager.loop();
  
  // Application heartbeat
  if (millis() - lastHeartbeat > 10000) { // Every 10 seconds
    heartbeatCounter++;
    Serial.printf("Heartbeat #%d - Connected: %s, OTA Active: %s\n", 
                  heartbeatCounter,
                  bleOTAManager.isDeviceConnected() ? "Yes" : "No",
                  bleOTAManager.isOTAActive() ? "Yes" : "No");
    
    if (bleOTAManager.isOTAActive()) {
      Serial.printf("OTA Progress: %u bytes received\n", bleOTAManager.getReceivedBytes());
    }
    
    lastHeartbeat = millis();
  }
  
  // Small delay to prevent excessive CPU usage
  delay(100);
}

void printSystemInfo() {
  Serial.println("=== System Information ===");
  
  // Get application description
  const esp_app_desc_t* app_desc = esp_app_get_description();
  if (app_desc) {
    Serial.println("Project: " + String(app_desc->project_name));
    Serial.println("Version: " + String(app_desc->version));
    Serial.println("Compile time: " + String(app_desc->time));
    Serial.println("Compile date: " + String(app_desc->date));
    Serial.println("IDF version: " + String(app_desc->idf_ver));
  }
  
  // Get chip info
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  
  Serial.println("Chip: ESP32-S3");
  Serial.println("Cores: " + String(chip_info.cores));
  Serial.println("Flash size: " + String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB");
  Serial.println("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("MAC Address: " + String((uint64_t)ESP.getEfuseMac(), HEX));
  
  Serial.println("===========================");
}

bool testOTAPartitions() {
  Serial.println("=== Testing OTA Partitions ===");
  
  // Check running partition
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) {
    Serial.println("ERROR: No running partition found!");
    return false;
  }
  
  Serial.println("Current partition: " + String(running->label));
  Serial.println("Address: 0x" + String(running->address, HEX));
  Serial.println("Size: " + String(running->size / 1024) + " KB");
  
  // Check next update partition
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  if (!next) {
    Serial.println("ERROR: No next update partition found!");
    return false;
  }
  
  Serial.println("Next update partition: " + String(next->label));
  Serial.println("Address: 0x" + String(next->address, HEX));
  Serial.println("Size: " + String(next->size / 1024) + " KB");
  
  // Check OTA data partition
  const esp_partition_t* otadata = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr);
  if (!otadata) {
    Serial.println("ERROR: No OTA data partition found!");
    return false;
  }
  
  Serial.println("OTA data partition: " + String(otadata->label));
  Serial.println("Address: 0x" + String(otadata->address, HEX));
  Serial.println("Size: " + String(otadata->size) + " bytes");
  
  // Validate partition sizes
  if (running->size < 1024 * 1024) {
    Serial.println("WARNING: Running partition is smaller than 1MB");
  }
  
  if (next->size < 1024 * 1024) {
    Serial.println("WARNING: Next partition is smaller than 1MB");
  }
  
  if (otadata->size < 0x2000) {
    Serial.println("ERROR: OTA data partition is too small!");
    return false;
  }
  
  Serial.println("OTA partitions test PASSED!");
  Serial.println("===============================");
  return true;
}