// Copyright 2020 <Zach Whitehead>
// OpenPPG

// ** Logic for EEPROM **
# define EEPROM_OFFSET 0  // Address of first byte of EEPROM

// Constants for device data
const unsigned int DEFAULT_SCREEN_ROTATION = 3;
const bool DEFAULT_METRIC_TEMP = true;
const bool DEFAULT_METRIC_ALT = true;
const int DEFAULT_PERFORMANCE_MODE = 0;
const int DEFAULT_THEME = 0;  // 0=light, 1=dark
const int DEFAULT_BATT_SIZE = 4000;  // 4kw

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define CONFIG_SERVICE_UUID      "1779A55B-DEB8-4482-A5D1-A12E62146138"

#define METRIC_ALT_UUID          "DF63F19E-7295-4A44-A0DC-184D1AFEDDF7"
#define FW_VERSION_UUID          "2A26"  // Using standard BLE UUID for version
#define ARMED_TIME_UUID          "58B29259-43EF-4593-B700-250EC839A2B2"
#define SCREEN_ROTATION_UUID     "9CBAB736-3705-4ECF-8086-FB7C5FB86282"
#define SEA_PRESSURE_UUID        "DB47E20E-D8C1-405A-971A-DA0A2DF7E0F6"
#define METRIC_TEMP_UUID         "D4962473-A3FB-4754-AD6A-90B079C3FB38"
#define PERFORMANCE_MODE_UUID    "D76C2E92-3547-4F5F-AFB4-515C5C08B06B"
#define BATT_SIZE_UUID           "4D076617-DC8C-46A5-902B-3F44FA28887E"
#define THEME_UUID               "AD0E4309-1EB2-461A-B36C-697B2E1604D2"
#define HW_REVISION_UUID         "2A27"  // Using standard BLE UUID for revision

#define DEVICE_INFO_SERVICE_UUID   "180A"  // Standard BLE Device Information Service
#define MANUFACTURER_NAME_UUID     "2A29"  // Standard BLE Manufacturer Name characteristic

class MetricAltCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() == 1) {  // Ensure we only get a single byte
        USBSerial.print("New: ");
        USBSerial.println(value[0], HEX);

        // Convert the received byte to a boolean
        deviceData.metric_alt = (value[0] != 0);

        writeDeviceData();
        USBSerial.println("Metric alt setting saved to EEPROM");
      } else {
        USBSerial.println("Invalid value length - expected 1 byte");
      }
    }
};

class PerformanceModeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() == 1) {
        uint8_t mode = value[0];
        if (mode <= 1) {  // Ensure value is 0 or 1
          deviceData.performance_mode = mode;
          writeDeviceData();
          USBSerial.println("Performance mode saved to EEPROM");
        } else {
          USBSerial.println("Invalid performance mode value");
        }
      } else {
        USBSerial.println("Invalid value length - expected 1 byte");
      }
    }
};

class ScreenRotationCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() == 1) {
        uint8_t rotation = value[0];
        if (rotation == 1 || rotation == 3) {  // Only allow valid rotation values
          deviceData.screen_rotation = rotation;
          writeDeviceData();
          resetRotation(rotation);  // Update screen immediately
          USBSerial.println("Screen rotation saved to EEPROM");
        } else {
          USBSerial.println("Invalid rotation value");
        }
      } else {
        USBSerial.println("Invalid value length - expected 1 byte");
      }
    }
};

void setupBLE() {
  // Initialize BLE
  BLEDevice::init("OpenPPG Controller");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(CONFIG_SERVICE_UUID);

  BLECharacteristic *pMetricAlt = pService->createCharacteristic(
                                   METRIC_ALT_UUID,
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_WRITE
                                 );

  pMetricAlt->setCallbacks(new MetricAltCallbacks());

  int metricAlt = deviceData.metric_alt ? 1 : 0;
  pMetricAlt->setValue(metricAlt);

  BLECharacteristic *pPerformanceMode = pService->createCharacteristic(
                                         PERFORMANCE_MODE_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pPerformanceMode->setCallbacks(new PerformanceModeCallbacks());
  int performanceMode = deviceData.performance_mode ? 1 : 0;

  pPerformanceMode->setValue(performanceMode);

  BLECharacteristic *pScreenRotation = pService->createCharacteristic(
                                        SCREEN_ROTATION_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE
                                      );

  pScreenRotation->setCallbacks(new ScreenRotationCallbacks());

  // screen rotation is 1 or 3
  int screenRotation = deviceData.screen_rotation == 1 ? 1 : 3;
  pScreenRotation->setValue(screenRotation);

  // Add read-only characteristics for device info
  BLECharacteristic *pFirmwareVersion = pService->createCharacteristic(
                                         FW_VERSION_UUID,
                                         BLECharacteristic::PROPERTY_READ
                                       );
  pFirmwareVersion->setValue(VERSION_STRING);

  BLECharacteristic *pHardwareRevision = pService->createCharacteristic(
                                          HW_REVISION_UUID,
                                          BLECharacteristic::PROPERTY_READ
                                        );
  // Convert revision to string
  char revision[4];
  snprintf(revision, sizeof(revision), "%d", deviceData.revision);
  pHardwareRevision->setValue(revision);

  BLECharacteristic *pArmedTime = pService->createCharacteristic(
                                   ARMED_TIME_UUID,
                                   BLECharacteristic::PROPERTY_READ
                                 );
  pArmedTime->setValue((uint8_t*)&deviceData.armed_time, sizeof(deviceData.armed_time));

  // Create the Device Information Service
  BLEService *pDeviceInfoService = pServer->createService(DEVICE_INFO_SERVICE_UUID);

  // Add Manufacturer Name characteristic
  BLECharacteristic *pManufacturer = pDeviceInfoService->createCharacteristic(
                                      MANUFACTURER_NAME_UUID,
                                      BLECharacteristic::PROPERTY_READ
                                    );
  pManufacturer->setValue("OpenPPG");

  pDeviceInfoService->start();

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  USBSerial.println("BLE device ready");
  USBSerial.println("Waiting for a client connection...");
}

// read saved data from EEPROM
void refreshDeviceData() {
  uint16_t crc;
  #ifdef M0_PIO
    if (0 != eep.read(EEPROM_OFFSET, deviceData, sizeof(deviceData))) {
      // Serial.println(F("error reading EEPROM"));
    }
  #elif RP_PIO
    EEPROM.get(EEPROM_OFFSET, deviceData);
  #elif CAN_PIO
    if (!EEPROM.begin(sizeof(deviceData))) {
      Serial.println(F("Failed to initialise EEPROM"));
      return;
    }
    EEPROM.get(EEPROM_OFFSET, deviceData);
  #endif
  crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);

  // If the CRC does not match, reset the device data
  if (crc != deviceData.crc || deviceData.sea_pressure < 0) {
    Serial.println(F("EEPROM CRC mismatch - resetting device data"));
    resetDeviceData();
  }

  // Update the revision if required
  //updateRevisionIfRequired();
}

// Update the revision if required
void updateRevisionIfRequired() {
  #ifdef RP_PIO
    if (deviceData.revision == 0) {
      deviceData.revision = 1;
      writeDeviceData();  // Save the updated revision to EEPROM
    }
  #endif
}

// Write to EEPROM
void writeDeviceData() {
  deviceData.crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
  #ifdef RP_PIO
  printDeviceData();
  EEPROM.put(EEPROM_OFFSET, deviceData);
  if (EEPROM.commit()) {
    Serial.println("EEPROM commit successful");
  } else {
    Serial.println("EEPROM commit failed");
  }
  #elif CAN_PIO
  EEPROM.put(EEPROM_OFFSET, deviceData);
  if (EEPROM.commit()) {
    Serial.println("EEPROM commit successful");
  } else {
    Serial.println("EEPROM commit failed");
  }
  #endif
}

// Reset EEPROM and deviceData to factory defaults
void resetDeviceData() {
  deviceData = STR_DEVICE_DATA_140_V1();

  // Set the revision based on the arch and board revision
  #ifdef M0_PIO
    deviceData.revision = 0;
  #elif RP_PIO
    deviceData.revision = 2;  // Default to new 2040 board revision
  #elif CAN_PIO
    deviceData.revision = 3;  // Set appropriate revision for ESP32-S3
  #endif

  deviceData.version_major = VERSION_MAJOR;
  deviceData.version_minor = VERSION_MINOR;
  deviceData.screen_rotation = DEFAULT_SCREEN_ROTATION;
  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;
  deviceData.metric_temp = DEFAULT_METRIC_TEMP;
  deviceData.metric_alt = DEFAULT_METRIC_ALT;
  deviceData.performance_mode = DEFAULT_PERFORMANCE_MODE;
  deviceData.theme = DEFAULT_THEME;
  deviceData.batt_size = DEFAULT_BATT_SIZE;
  deviceData.armed_time = 0;
  writeDeviceData();
}

// ** Logic for WebUSB **
// Callback for when the USB connection state changes
void line_state_callback(bool connected) {
  setLEDColor(connected ? LED_BLUE : LED_GREEN);
  setLEDs(connected);

  if ( connected ) send_usb_serial();
}

// Not implemented for ESP32-S3 yet
void parse_usb_serial() {
#ifdef USE_TINYUSB
  const size_t capacity = JSON_OBJECT_SIZE(13) + 90;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
    // display.fillScreen(DEFAULT_BG_COLOR);
    //TODO: display ("BL - UF2");
    rebootBootloader();
    return;  // run only the command
  }

  if (doc["major_v"] < 5) return; // ignore old versions


  deviceData.screen_rotation = doc["screen_rot"].as<unsigned int>();  // "3/1"
  deviceData.sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  deviceData.metric_temp = doc["metric_temp"];  // true/false
  deviceData.metric_alt = doc["metric_alt"];  // true/false
  deviceData.performance_mode = doc["performance_mode"];  // 0,1
  deviceData.batt_size = doc["batt_size"];  // 4000
  deviceData.theme = doc["theme"];  // 0,1
  sanitizeDeviceData();
  writeDeviceData();
  resetRotation(deviceData.screen_rotation);  // Screen orientation may have changed
  setTheme(deviceData.theme);  // may have changed

  send_usb_serial();
#endif
}

bool sanitizeDeviceData() {
  bool changed = false;
  // Ensure screen rotation is either 1 or 3, default to 3
  if (deviceData.screen_rotation == 1 || deviceData.screen_rotation == 3) {
  } else {
    deviceData.screen_rotation = 3;
    changed = true;
  }

  // Ensure sea pressure is within acceptable limits, default to 1013.25
  // 337 is the air pressure at the top of Mt. Everest
  // 1065 is the air pressure at the dead sea. Pad both a bit
  if (deviceData.sea_pressure < 300 || deviceData.sea_pressure > 1200) {
    deviceData.sea_pressure = 1013.25;
    changed = true;
  }

  // Simply force metric_temp and metric_alt to be valid bool values
  if (deviceData.metric_temp != true && deviceData.metric_temp != false) {
    deviceData.metric_temp = true;
    changed = true;
  }
  if (deviceData.metric_alt != true && deviceData.metric_alt != false) {
    deviceData.metric_alt = true;
    changed = true;
  }
  // Ensure performance_mode is either 0 or 1, default to 0
  if (deviceData.performance_mode > 1) {
    deviceData.performance_mode = 0;
    changed = true;
  }
  // Ensure battery size is within acceptable limits, default to 4000
  if (deviceData.batt_size < 1 || deviceData.batt_size > 10000) {
    deviceData.batt_size = 4000;
    changed = true;
  }
  // Ensure theme is either 0 or 1, default to 0
  if (deviceData.theme > 1) {
    deviceData.theme = 0; // 0=light, 1=dark
    changed = true;
  }
  return changed;
}

/**
 * Prints the hardware configuration to the Serial monitor.
 *
 * @param config The HardwareConfig object containing the hardware configuration.
 */
void debugHardwareConfig(const HardwareConfig& config) {
  Serial.println("Hardware Configuration:");
  Serial.print("button_top: ");
  Serial.println(config.button_top);
  Serial.print("buzzer_pin: ");
  Serial.println(config.buzzer_pin);
  Serial.print("led_sw: ");
  Serial.println(config.led_sw);
  Serial.print("throttle_pin: ");
  Serial.println(config.throttle_pin);
  Serial.print("bmp_pin: ");
  Serial.println(config.bmp_pin);
  Serial.print("tft_rst: ");
  Serial.println(config.tft_rst);
  Serial.print("tft_cs: ");
  Serial.println(config.tft_cs);
  Serial.print("tft_dc: ");
  Serial.println(config.tft_dc);
  Serial.print("tft_lite: ");
  Serial.println(config.tft_lite);
  Serial.print("enable_vib: ");
  Serial.println(config.enable_vib ? "true" : "false");
  Serial.print("enable_neopixel: ");
  Serial.println(config.enable_neopixel ? "true" : "false");
}

void send_usb_serial() {
#ifdef USE_TINYUSB
#ifdef M0_PIO
  const size_t capacity = JSON_OBJECT_SIZE(11) + 90;
  DynamicJsonDocument doc(capacity);

  doc["major_v"] = VERSION_MAJOR;
  doc["minor_v"] = VERSION_MINOR;
  doc["arch"] = "SAMD21";
  doc["screen_rot"] = deviceData.screen_rotation;
  doc["armed_time"] = deviceData.armed_time;
  doc["metric_temp"] = deviceData.metric_temp;
  doc["metric_alt"] = deviceData.metric_alt;
  doc["performance_mode"] = deviceData.performance_mode;
  doc["sea_pressure"] = deviceData.sea_pressure;
  doc["device_id"] = chipId();

  char output[256];
  serializeJson(doc, output);
  usb_web.println(output);
#elif RP_PIO
  StaticJsonDocument<256> doc; // <- a little more than 256 bytes in the stack

  doc["mj_v"].set(VERSION_MAJOR);
  doc["mi_v"].set(VERSION_MINOR);
  doc["arch"].set("RP2040");
  doc["scr_rt"].set(deviceData.screen_rotation);
  doc["ar_tme"].set(deviceData.armed_time);
  doc["m_tmp"].set(deviceData.metric_temp);
  doc["m_alt"].set(deviceData.metric_alt);
  doc["prf"].set(deviceData.performance_mode);
  doc["sea_p"].set(deviceData.sea_pressure);
  doc["thm"].set(deviceData.theme);
  doc["rev"].set(deviceData.revision);
  //doc["id"].set(chipId()); // webusb bug prevents anything over a certain size / this extra field from being sent

  char output[256];
  serializeJson(doc, output, sizeof(output));
  usb_web.println(output);
  usb_web.flush();
  //Serial.println(chipId());
#endif // M0_PIO/RP_PIO
#endif // USE_TINYUSB
}

