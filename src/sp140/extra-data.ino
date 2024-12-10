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
#include <BLE2902.h>

#define CONFIG_SERVICE_UUID      "1779A55B-DEB8-4482-A5D1-A12E62146138"

#define METRIC_ALT_UUID          "DF63F19E-7295-4A44-A0DC-184D1AFEDDF7"
#define ARMED_TIME_UUID          "58B29259-43EF-4593-B700-250EC839A2B2"
#define SCREEN_ROTATION_UUID     "9CBAB736-3705-4ECF-8086-FB7C5FB86282"
#define SEA_PRESSURE_UUID        "DB47E20E-D8C1-405A-971A-DA0A2DF7E0F6"
#define METRIC_TEMP_UUID         "D4962473-A3FB-4754-AD6A-90B079C3FB38"
#define PERFORMANCE_MODE_UUID    "D76C2E92-3547-4F5F-AFB4-515C5C08B06B"
#define BATT_SIZE_UUID           "4D076617-DC8C-46A5-902B-3F44FA28887E"
#define THEME_UUID               "AD0E4309-1EB2-461A-B36C-697B2E1604D2"
#define HW_REVISION_UUID         "2A27"  // Using standard BLE UUID for revision
#define FW_VERSION_UUID          "2A26"  // Using standard BLE UUID for version

#define THROTTLE_VALUE_UUID      "50AB3859-9FBF-4D30-BF97-2516EE632FAD"

#define DEVICE_INFO_SERVICE_UUID   "180A"  // Standard BLE Device Information Service
#define MANUFACTURER_NAME_UUID     "2A29"  // Standard BLE Manufacturer Name characteristic

#define BMS_TELEMETRY_SERVICE_UUID "9E0F2FA3-3F2B-49C0-A6A3-3D8923062133"
#define ESC_TELEMETRY_SERVICE_UUID "C154DAE9-1984-40EA-B20F-5B23F9CBA0A9"

#define DEVICE_STATE_UUID          "8F80BCF5-B58F-4908-B079-E8AD6F5EE257"

// BMS Characteristic UUIDs
#define BMS_SOC_UUID                "ACDEB138-3BD0-4BB3-B159-19F6F70871ED"
#define BMS_VOLTAGE_UUID            "AC0768DF-2F49-43D4-B23D-1DC82C90A9E9"
#define BMS_CURRENT_UUID            "6FEEC926-BA3C-4E65-BC71-5DB481811186"
#define BMS_POWER_UUID              "9DEA1343-434F-4555-A0A1-BB43FCBC68A6"
#define BMS_HIGH_CELL_UUID          "49267B41-560F-4CFF-ADC8-90EF85D2BE20"
#define BMS_LOW_CELL_UUID           "B9D01E5C-3751-4092-8B06-6D1FFF479E77"
#define BMS_HIGH_TEMP_UUID          "0EA08B6D-C905-4D9D-93F8-51E35DA096FC"
#define BMS_LOW_TEMP_UUID           "26CD6E8A-175D-4C8E-B487-DEFF0B034F2A"
#define BMS_FAILURE_LEVEL_UUID      "396C768B-F348-44CC-9D46-92388F25A557"
#define BMS_VOLTAGE_DIFF_UUID       "1C45825B-7C81-430B-8D5F-B644FFFC71BB"


static BLECharacteristic* pBMSSOC = nullptr;
static BLECharacteristic* pBMSVoltage = nullptr;
static BLECharacteristic* pBMSCurrent = nullptr;
static BLECharacteristic* pBMSPower = nullptr;
static BLECharacteristic* pBMSHighCell = nullptr;
static BLECharacteristic* pBMSLowCell = nullptr;
static BLECharacteristic* pBMSHighTemp = nullptr;
static BLECharacteristic* pBMSLowTemp = nullptr;
static BLECharacteristic* pBMSFailureLevel = nullptr;
static BLECharacteristic* pBMSVoltageDiff = nullptr;

void updateThrottleBLE(int value) {
  // Handle disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    USBSerial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  // Handle connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Send notification if connected
  if (deviceConnected && pThrottleCharacteristic != nullptr) {
    try {
      pThrottleCharacteristic->setValue((uint8_t*)&value, sizeof(value));
      pThrottleCharacteristic->notify();
      delay(5); // prevent bluetooth stack congestion - can be as low as 3ms
    } catch (...) {
      USBSerial.println("Error sending BLE notification");
    }
  }
}

void updateBMSTelemetry(const STR_BMS_TELEMETRY_140& telemetry) {
  if (!deviceConnected) return;

  // Create temporary variables for the float values
  float soc = telemetry.soc;
  float voltage = telemetry.battery_voltage;
  float current = telemetry.battery_current;
  float power = telemetry.power;
  float highCell = telemetry.highest_cell_voltage;
  float lowCell = telemetry.lowest_cell_voltage;
  float highTemp = telemetry.highest_temperature;
  float lowTemp = telemetry.lowest_temperature;
  float voltageDiff = telemetry.voltage_differential;

  // Update each characteristic using the temporary variables
  pBMSSOC->setValue(soc);
  pBMSVoltage->setValue(voltage);
  pBMSCurrent->setValue(current);
  pBMSPower->setValue(power);
  pBMSHighCell->setValue(highCell);
  pBMSLowCell->setValue(lowCell);
  pBMSHighTemp->setValue(highTemp);
  pBMSLowTemp->setValue(lowTemp);
  pBMSFailureLevel->setValue((uint8_t*)&telemetry.battery_failure_level, sizeof(uint8_t));
  pBMSVoltageDiff->setValue(voltageDiff);
}

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

class ThrottleValueCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Return the current pot value when read
    pot->update();
    uint16_t potVal = pot->getValue();
    pCharacteristic->setValue((uint8_t*)&potVal, sizeof(potVal));
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    if (currentState != ARMED_CRUISING) {
      return;  // Only allow updates while in cruise mode
    }

    std::string value = pCharacteristic->getValue();
    if (value.length() == 2) {  // Expecting 2 bytes for PWM value
      uint16_t newPWM = (value[0] << 8) | value[1];

      // Validate PWM range
      if (newPWM >= ESC_MIN_SPIN_PWM && newPWM <= ESC_MAX_PWM) {
        if (xQueueSend(throttleUpdateQueue, &newPWM, pdMS_TO_TICKS(100)) != pdTRUE) {
          USBSerial.println("Failed to queue throttle update");
        }
      }
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    USBSerial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    USBSerial.println("Device disconnected");
    // Restart advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    USBSerial.println("Started advertising");
  }
};


void setupBLE() {
  // Initialize BLE
  BLEDevice::init("OpenPPG Controller");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(CONFIG_SERVICE_UUID);

  // Add device state characteristic
  pDeviceStateCharacteristic = pService->createCharacteristic(
    DEVICE_STATE_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  uint8_t initialState = DISARMED;  // Create a variable to reference
  pDeviceStateCharacteristic->setValue(&initialState, sizeof(initialState));
  pDeviceStateCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pMetricAlt = pService->createCharacteristic(
                                   METRIC_ALT_UUID,
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_WRITE);

  pMetricAlt->setCallbacks(new MetricAltCallbacks());

  int metricAlt = deviceData.metric_alt ? 1 : 0;
  pMetricAlt->setValue(metricAlt);

  BLECharacteristic *pPerformanceMode = pService->createCharacteristic(
                                         PERFORMANCE_MODE_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE);

  pPerformanceMode->setCallbacks(new PerformanceModeCallbacks());
  int performanceMode = deviceData.performance_mode ? 1 : 0;

  pPerformanceMode->setValue(performanceMode);

  BLECharacteristic *pScreenRotation = pService->createCharacteristic(
                                        SCREEN_ROTATION_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE);

  pScreenRotation->setCallbacks(new ScreenRotationCallbacks());

  // screen rotation is 1 or 3
  int screenRotation = deviceData.screen_rotation == 1 ? 1 : 3;
  pScreenRotation->setValue(screenRotation);

  // Add read-only characteristics for device info
  BLECharacteristic *pFirmwareVersion = pService->createCharacteristic(
                                         FW_VERSION_UUID,
                                         BLECharacteristic::PROPERTY_READ);
  pFirmwareVersion->setValue(VERSION_STRING);

  BLECharacteristic *pHardwareRevision = pService->createCharacteristic(
                                          HW_REVISION_UUID,
                                          BLECharacteristic::PROPERTY_READ);
  // Send revision directly as a byte
  pHardwareRevision->setValue(&deviceData.revision, sizeof(deviceData.revision));

  BLECharacteristic *pArmedTime = pService->createCharacteristic(
                                   ARMED_TIME_UUID,
                                   BLECharacteristic::PROPERTY_READ);
  pArmedTime->setValue((uint8_t*)&deviceData.armed_time, sizeof(deviceData.armed_time));

  // Create the Device Information Service
  BLEService *pDeviceInfoService = pServer->createService(DEVICE_INFO_SERVICE_UUID);

  // Add Manufacturer Name characteristic
  BLECharacteristic *pManufacturer = pDeviceInfoService->createCharacteristic(
                                      MANUFACTURER_NAME_UUID,
                                      BLECharacteristic::PROPERTY_READ);
  pManufacturer->setValue("OpenPPG");

  // Create throttle characteristic with notify capability
  pThrottleCharacteristic = pService->createCharacteristic(
    THROTTLE_VALUE_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |  // Add write permission
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pThrottleCharacteristic->setCallbacks(new ThrottleValueCallbacks());
  pThrottleCharacteristic->addDescriptor(new BLE2902());

  //pDeviceInfoService->start();

  // Create BMS Telemetry Service
  BLEService *pBMSService = pServer->createService(BMS_TELEMETRY_SERVICE_UUID);

  // Create read-only characteristics for BMS data
  pBMSSOC = pBMSService->createCharacteristic(
      BMS_SOC_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSVoltage = pBMSService->createCharacteristic(
      BMS_VOLTAGE_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSCurrent = pBMSService->createCharacteristic(
      BMS_CURRENT_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSPower = pBMSService->createCharacteristic(
      BMS_POWER_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSHighCell = pBMSService->createCharacteristic(
      BMS_HIGH_CELL_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSLowCell = pBMSService->createCharacteristic(
      BMS_LOW_CELL_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSHighTemp = pBMSService->createCharacteristic(
      BMS_HIGH_TEMP_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSLowTemp = pBMSService->createCharacteristic(
      BMS_LOW_TEMP_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSFailureLevel = pBMSService->createCharacteristic(
      BMS_FAILURE_LEVEL_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  pBMSVoltageDiff = pBMSService->createCharacteristic(
      BMS_VOLTAGE_DIFF_UUID,
      BLECharacteristic::PROPERTY_READ
  );

  // Start all services
  pService->start();
  pBMSService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->start();

  USBSerial.println("BLE device ready");
  USBSerial.println("Waiting for a client connection...");
}

// read saved data from EEPROM
void refreshDeviceData() {
  uint16_t crc;
  #ifdef RP_PIO
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
  #ifdef RP_PIO
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
#ifdef RP_PIO
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
#endif //RP_PIO
#endif // USE_TINYUSB
}
