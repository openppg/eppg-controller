// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include <Preferences.h>  // Add ESP32 Preferences library
#include "../../inc/sp140/throttle.h"

/**
 * WebSerial Protocol Documentation
 *
 * Commands should be sent as JSON objects with the following format:
 *
 * For simple commands:
 * { "command": "command_name" }
 *
 * For settings updates:
 * {
 *   "settings": {
 *     "screen_rot": 3,
 *     "sea_pressure": 1013.25,
 *     "metric_temp": true,
 *     "metric_alt": true,
 *     "performance_mode": 0,
 *     "theme": 0
 *   }
 * }
 *
 * Available Commands:
 * - "rbl": Reboot to bootloader for firmware updates
 * - "sync": Request current device settings and state
 *
 * Response Format for sync command:
 * {
 *   "mj_v": number,        // Major version
 *   "mi_v": number,        // Minor version
 *   "arch": string,        // Architecture ("ESP32S3")
 *   "scr_rt": number,      // Screen rotation (1 or 3)
 *   "ar_tme": number,      // Armed time in minutes
 *   "m_tmp": bool,         // Metric temperature
 *   "m_alt": bool,         // Metric altitude
 *   "prf": number,         // Performance mode (0=chill, 1=sport)
 *   "sea_p": float,        // Sea pressure (hPa/mbar)
 *   "thm": number          // Theme (0=light, 1=dark)
 * }
 */

// Constants for device data
const unsigned int DEFAULT_SCREEN_ROTATION = 3;
const bool DEFAULT_METRIC_TEMP = true;
const bool DEFAULT_METRIC_ALT = true;
const int DEFAULT_PERFORMANCE_MODE = 0;
const int DEFAULT_THEME = 0;  // 0=light, 1=dark

// Preferences namespace
const char* PREFS_NAMESPACE = "openppg";

// Preferences keys
const char* KEY_VERSION_MAJOR = "ver_major";
const char* KEY_VERSION_MINOR = "ver_minor";
const char* KEY_SCREEN_ROTATION = "scr_rot";
const char* KEY_SEA_PRESSURE = "sea_pres";
const char* KEY_METRIC_TEMP = "metric_tmp";
const char* KEY_METRIC_ALT = "metric_alt";
const char* KEY_PERFORMANCE_MODE = "perf_mode";
const char* KEY_THEME = "theme";
const char* KEY_ARMED_TIME = "armed_time";
const char* KEY_REVISION = "revision";
const char* KEY_TIMEZONE_OFFSET = "tz_offset";

// Create a Preferences instance
Preferences preferences;

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
#define THEME_UUID               "AD0E4309-1EB2-461A-B36C-697B2E1604D2"
#define HW_REVISION_UUID         "2A27"  // Using standard BLE UUID for revision
#define FW_VERSION_UUID          "2A26"  // Using standard BLE UUID for version
#define UNIX_TIME_UUID          "E09FF0B7-5D02-4FD5-889E-C4251A58D9E7"  // Our custom UUID
#define TIMEZONE_UUID           "CAE49D1A-7C21-4B0C-8520-416F3EF69DB1"

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
#define BMS_CELL_VOLTAGES_UUID      "4337e58b-8462-49b2-b061-c3bf379ef4af"

// ESC Characteristic UUIDs
#define ESC_VOLTAGE_UUID           "0528ecd8-9337-4249-95e4-9aba69f6c1f4"
#define ESC_CURRENT_UUID           "3889e01e-7d2d-4478-b5cc-a06b803e2788"
#define ESC_RPM_UUID              "24dc4a84-0be3-4eba-a8c3-ed9748daa599"
#define ESC_TEMPS_UUID            "d087f190-5450-4fea-b9ff-17133a0b6f64"

// Add near the top with other BLE characteristic declarations
static BLECharacteristic* pESCVoltage = nullptr;
static BLECharacteristic* pESCCurrent = nullptr;
static BLECharacteristic* pESCRPM = nullptr;
static BLECharacteristic* pESCTemps = nullptr;

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
static BLECharacteristic* pBMSCellVoltages = nullptr;

void updateThrottleBLE(int value) {
  // Handle disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    vTaskDelay(pdMS_TO_TICKS(500));  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
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
      vTaskDelay(pdMS_TO_TICKS(5));  // prevent bluetooth stack congestion - can be as low as 3ms
    } catch (...) {
      USBSerial.println("Error sending BLE notification");
    }
  }
}

void updateBMSTelemetry(const STR_BMS_TELEMETRY_140& telemetry) {
  // Set characteristic values regardless of connection state,
  // so they have values when a client connects and reads them.
  // Notifications will only be sent if connected.

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
  if (pBMSSOC) pBMSSOC->setValue(soc);
  if (pBMSVoltage) pBMSVoltage->setValue(voltage);
  if (pBMSCurrent) pBMSCurrent->setValue(current);
  if (pBMSPower) pBMSPower->setValue(power);
  if (pBMSHighCell) pBMSHighCell->setValue(highCell);
  if (pBMSLowCell) pBMSLowCell->setValue(lowCell);
  if (pBMSHighTemp) pBMSHighTemp->setValue(highTemp);
  if (pBMSLowTemp) pBMSLowTemp->setValue(lowTemp);
  if (pBMSFailureLevel) pBMSFailureLevel->setValue((uint8_t*)&telemetry.battery_failure_level, sizeof(uint8_t));
  if (pBMSVoltageDiff) pBMSVoltageDiff->setValue(voltageDiff);

  // Update cell voltages characteristic
  if (pBMSCellVoltages != nullptr) {
    uint16_t cell_millivolts[BMS_CELLS_NUM];
    for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
      cell_millivolts[i] = (uint16_t)(telemetry.cell_voltages[i] * 1000.0f);
    }

    // Debug print first and last cell voltage
    // USBSerial.print("Setting cell voltages: Cell 1 = ");
    // USBSerial.print(cell_millivolts[0]);
    // USBSerial.print(" mV, Cell ");
    // USBSerial.print(BMS_CELLS_NUM);
    // USBSerial.print(" = ");
    // USBSerial.print(cell_millivolts[BMS_CELLS_NUM-1]);
    // USBSerial.println(" mV");

    pBMSCellVoltages->setValue((uint8_t*)cell_millivolts, BMS_CELLS_NUM * sizeof(uint16_t));
  }

  // Only send notifications if a device is actually connected
  if (deviceConnected) {
    if (pBMSSOC) pBMSSOC->notify();
    if (pBMSVoltage) pBMSVoltage->notify();
    if (pBMSCurrent) pBMSCurrent->notify();
    if (pBMSPower) pBMSPower->notify();
    if (pBMSHighCell) pBMSHighCell->notify();
    if (pBMSLowCell) pBMSLowCell->notify();
    if (pBMSHighTemp) pBMSHighTemp->notify();
    if (pBMSLowTemp) pBMSLowTemp->notify();
    if (pBMSFailureLevel) pBMSFailureLevel->notify();
    if (pBMSVoltageDiff) pBMSVoltageDiff->notify();
  }
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
      USBSerial.println("Metric alt setting saved to Preferences");
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
        USBSerial.println("Performance mode saved to Preferences");
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
        //resetRotation(rotation);  // Update screen immediately
        USBSerial.println("Screen rotation saved to Preferences");
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
    // Return the current pot value when read (raw ADC)
    uint16_t potVal = readThrottleRaw();
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
      if (newPWM >= ESC_MIN_PWM && newPWM <= ESC_MAX_PWM) {
        if (throttleUpdateQueue == NULL) {
          USBSerial.println("throttleUpdateQueue is NULL!");
        } else if (xQueueSend(throttleUpdateQueue, &newPWM, pdMS_TO_TICKS(100)) != pdTRUE) {
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

class TimeCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() == sizeof(time_t)) {  // Expecting just a unix timestamp
      struct timeval tv;
      time_t timestamp;

      // Copy the incoming timestamp
      memcpy(&timestamp, value.data(), sizeof(timestamp));

      // Apply timezone offset
      timestamp += deviceData.timezone_offset;

      tv.tv_sec = timestamp;
      tv.tv_usec = 0;

      if (settimeofday(&tv, NULL) == 0) {
        USBSerial.println("Time set successfully");
      } else {
        USBSerial.println("Failed to set time");
      }
    } else {
      USBSerial.println("Invalid timestamp length");
    }
  }

  void onRead(BLECharacteristic *pCharacteristic) {
    time_t now;
    time(&now);
    now -= deviceData.timezone_offset;  // Remove timezone offset for UTC time
    pCharacteristic->setValue((uint8_t*)&now, sizeof(now));
  }
};

class TimezoneCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() == 4) {  // Expecting 4 bytes for timezone offset
      int32_t offset;
      memcpy(&offset, value.data(), sizeof(offset));

      deviceData.timezone_offset = offset;
      writeDeviceData();
      USBSerial.print("Timezone offset set to: ");
      USBSerial.println(offset);
    } else {
      USBSerial.println("Invalid timezone offset length");
    }
  }

  void onRead(BLECharacteristic *pCharacteristic) {
    pCharacteristic->setValue((uint8_t*)&deviceData.timezone_offset, sizeof(deviceData.timezone_offset));
  }
};

void setupBLE() {
  // Initialize BLE
  BLEDevice::init("OpenPPG Controller");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pConfigService = pServer->createService(BLEUUID(CONFIG_SERVICE_UUID), 30);

  // Add time characteristics BEFORE starting the service
  BLECharacteristic *pUnixTime = pConfigService->createCharacteristic(
                                    BLEUUID(UNIX_TIME_UUID),
                                    BLECharacteristic::PROPERTY_READ |
                                    BLECharacteristic::PROPERTY_WRITE);
  pUnixTime->setCallbacks(new TimeCallbacks());

  BLECharacteristic *pTimezone = pConfigService->createCharacteristic(
                                  BLEUUID(TIMEZONE_UUID),
                                  BLECharacteristic::PROPERTY_READ |
                                  BLECharacteristic::PROPERTY_WRITE);
  pTimezone->setCallbacks(new TimezoneCallbacks());
  pTimezone->setValue((uint8_t*)&deviceData.timezone_offset, sizeof(deviceData.timezone_offset));

  // Add device state characteristic
  pDeviceStateCharacteristic = pConfigService->createCharacteristic(
    BLEUUID(DEVICE_STATE_UUID),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY);

  uint8_t initialState = DISARMED;  // Create a variable to reference
  pDeviceStateCharacteristic->setValue(&initialState, sizeof(initialState));
  pDeviceStateCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pMetricAlt = pConfigService->createCharacteristic(
                                   BLEUUID(METRIC_ALT_UUID),
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_WRITE);

  pMetricAlt->setCallbacks(new MetricAltCallbacks());

  int metricAlt = deviceData.metric_alt ? 1 : 0;
  pMetricAlt->setValue(metricAlt);

  BLECharacteristic *pPerformanceMode = pConfigService->createCharacteristic(
                                         BLEUUID(PERFORMANCE_MODE_UUID),
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE);

  pPerformanceMode->setCallbacks(new PerformanceModeCallbacks());
  int performanceMode = deviceData.performance_mode ? 1 : 0;

  pPerformanceMode->setValue(performanceMode);

  BLECharacteristic *pScreenRotation = pConfigService->createCharacteristic(
                                        BLEUUID(SCREEN_ROTATION_UUID),
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE);

  pScreenRotation->setCallbacks(new ScreenRotationCallbacks());

  // screen rotation is 1 or 3
  int screenRotation = deviceData.screen_rotation == 1 ? 1 : 3;
  pScreenRotation->setValue(screenRotation);

  // Add read-only characteristics for device info
  BLECharacteristic *pFirmwareVersion = pConfigService->createCharacteristic(
                                        BLEUUID(FW_VERSION_UUID),
                                        BLECharacteristic::PROPERTY_READ);

  // Use 2-byte format: [major, minor] instead of string
  uint8_t versionBytes[2] = {VERSION_MAJOR, VERSION_MINOR};
  pFirmwareVersion->setValue(versionBytes, sizeof(versionBytes));

  BLECharacteristic *pHardwareRevision = pConfigService->createCharacteristic(
                                         BLEUUID(HW_REVISION_UUID),
                                         BLECharacteristic::PROPERTY_READ);
  // Send revision directly as a byte
  pHardwareRevision->setValue(&deviceData.revision, sizeof(deviceData.revision));

  BLECharacteristic *pArmedTime = pConfigService->createCharacteristic(
                                  BLEUUID(ARMED_TIME_UUID),
                                  BLECharacteristic::PROPERTY_READ);
  pArmedTime->setValue((uint8_t*)&deviceData.armed_time, sizeof(deviceData.armed_time));

  // Create the Device Information Service
  BLEService *pDeviceInfoService = pServer->createService(BLEUUID(DEVICE_INFO_SERVICE_UUID), 10);

  // Add Manufacturer Name characteristic
  BLECharacteristic *pManufacturer = pDeviceInfoService->createCharacteristic(
                                     BLEUUID(MANUFACTURER_NAME_UUID),
                                     BLECharacteristic::PROPERTY_READ);
  pManufacturer->setValue("OpenPPG");

  // Create throttle characteristic with notify capability
  pThrottleCharacteristic = pConfigService->createCharacteristic(
    BLEUUID(THROTTLE_VALUE_UUID),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |  // Add write permission
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pThrottleCharacteristic->setCallbacks(new ThrottleValueCallbacks());
  pThrottleCharacteristic->addDescriptor(new BLE2902());

  // Create BMS Telemetry Service
  BLEService *pBMSService = pServer->createService(BLEUUID(BMS_TELEMETRY_SERVICE_UUID), 30);

  // Create read-only characteristics for BMS data
  pBMSSOC = pBMSService->createCharacteristic(
      BLEUUID(BMS_SOC_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSVoltage = pBMSService->createCharacteristic(
      BLEUUID(BMS_VOLTAGE_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSCurrent = pBMSService->createCharacteristic(
      BLEUUID(BMS_CURRENT_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSPower = pBMSService->createCharacteristic(
      BLEUUID(BMS_POWER_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSHighCell = pBMSService->createCharacteristic(
      BLEUUID(BMS_HIGH_CELL_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSLowCell = pBMSService->createCharacteristic(
      BLEUUID(BMS_LOW_CELL_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSHighTemp = pBMSService->createCharacteristic(
      BLEUUID(BMS_HIGH_TEMP_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSLowTemp = pBMSService->createCharacteristic(
      BLEUUID(BMS_LOW_TEMP_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSFailureLevel = pBMSService->createCharacteristic(
      BLEUUID(BMS_FAILURE_LEVEL_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSVoltageDiff = pBMSService->createCharacteristic(
      BLEUUID(BMS_VOLTAGE_DIFF_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pBMSCellVoltages = pBMSService->createCharacteristic(
      BLEUUID(BMS_CELL_VOLTAGES_UUID),
      BLECharacteristic::PROPERTY_READ // Only READ property for debugging
  );

  // Set initial value for pBMSCellVoltages (array of zeros)
  uint16_t initial_cell_values[BMS_CELLS_NUM] = {0};
  pBMSCellVoltages->setValue((uint8_t*)initial_cell_values, BMS_CELLS_NUM * sizeof(uint16_t));

  // Create ESC Telemetry Service
  BLEService *pESCService = pServer->createService(BLEUUID(ESC_TELEMETRY_SERVICE_UUID), 20);

  // Create characteristics for ESC data
  pESCVoltage = pESCService->createCharacteristic(
      BLEUUID(ESC_VOLTAGE_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pESCCurrent = pESCService->createCharacteristic(
      BLEUUID(ESC_CURRENT_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pESCRPM = pESCService->createCharacteristic(
      BLEUUID(ESC_RPM_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  pESCTemps = pESCService->createCharacteristic(
      BLEUUID(ESC_TEMPS_UUID),
      BLECharacteristic::PROPERTY_READ
  );

  // Start all services AFTER adding all characteristics
  pConfigService->start();
  pBMSService->start();
  pESCService->start();
  pDeviceInfoService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID(CONFIG_SERVICE_UUID));
  pAdvertising->addServiceUUID(BLEUUID(BMS_TELEMETRY_SERVICE_UUID));
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->start();

  USBSerial.println("BLE device ready");
  USBSerial.println("Waiting for a client connection...");
}

// Read saved data from Preferences
void refreshDeviceData() {
  // Try to initialize preferences, with corruption recovery
  if (!preferences.begin(PREFS_NAMESPACE, false)) {
    USBSerial.println(F("Failed to initialize Preferences - may be corrupted"));

    // Try to clear corrupted preferences and start fresh
    preferences.begin(PREFS_NAMESPACE, false);
    preferences.clear();
    preferences.end();

    USBSerial.println(F("Cleared potentially corrupted preferences, using defaults"));
    resetDeviceData();
    return;
  }

  // Check if we have saved preferences before
  if (!preferences.isKey(KEY_VERSION_MAJOR)) {
    USBSerial.println(F("No saved preferences found - initializing with defaults"));
    preferences.end();
    resetDeviceData();
    return;
  }

  // Load all values from preferences with validation
  bool dataValid = true;

  deviceData.version_major = preferences.getUChar(KEY_VERSION_MAJOR, VERSION_MAJOR);
  deviceData.version_minor = preferences.getUChar(KEY_VERSION_MINOR, VERSION_MINOR);
  deviceData.screen_rotation = preferences.getUChar(KEY_SCREEN_ROTATION, DEFAULT_SCREEN_ROTATION);
  deviceData.sea_pressure = preferences.getFloat(KEY_SEA_PRESSURE, DEFAULT_SEA_PRESSURE);
  deviceData.metric_temp = preferences.getBool(KEY_METRIC_TEMP, DEFAULT_METRIC_TEMP);
  deviceData.metric_alt = preferences.getBool(KEY_METRIC_ALT, DEFAULT_METRIC_ALT);
  deviceData.performance_mode = preferences.getUChar(KEY_PERFORMANCE_MODE, DEFAULT_PERFORMANCE_MODE);
  deviceData.theme = preferences.getUChar(KEY_THEME, DEFAULT_THEME);
  deviceData.armed_time = preferences.getUShort(KEY_ARMED_TIME, 0);
  deviceData.revision = preferences.getUChar(KEY_REVISION, 3);  // Default to ESP32-S3
  deviceData.timezone_offset = preferences.getInt(KEY_TIMEZONE_OFFSET, 0);

  // Validate critical display-related settings
  if (deviceData.screen_rotation != 1 && deviceData.screen_rotation != 3) {
    USBSerial.println(F("Warning: Invalid screen rotation detected, using default"));
    deviceData.screen_rotation = DEFAULT_SCREEN_ROTATION;
    dataValid = false;
  }

  if (deviceData.theme > 1) {
    USBSerial.println(F("Warning: Invalid theme detected, using default"));
    deviceData.theme = DEFAULT_THEME;
    dataValid = false;
  }

  preferences.end();

  // Ensure values are within valid ranges
  if (sanitizeDeviceData() || !dataValid) {
    USBSerial.println(F("Sanitized corrupted preference values"));
    writeDeviceData();  // Save sanitized values
  }

  USBSerial.println(F("Device data loaded from Preferences"));
}

// Write to Preferences
void writeDeviceData() {
  if (!preferences.begin(PREFS_NAMESPACE, false)) {
    USBSerial.println(F("Failed to initialize Preferences for writing"));
    return;
  }

  // Save all values to preferences with error checking
  bool success = true;
  success &= (preferences.putUChar(KEY_VERSION_MAJOR, deviceData.version_major) > 0);
  success &= (preferences.putUChar(KEY_VERSION_MINOR, deviceData.version_minor) > 0);
  success &= (preferences.putUChar(KEY_SCREEN_ROTATION, deviceData.screen_rotation) > 0);
  success &= (preferences.putFloat(KEY_SEA_PRESSURE, deviceData.sea_pressure) > 0);
  success &= (preferences.putBool(KEY_METRIC_TEMP, deviceData.metric_temp) > 0);
  success &= (preferences.putBool(KEY_METRIC_ALT, deviceData.metric_alt) > 0);
  success &= (preferences.putUChar(KEY_PERFORMANCE_MODE, deviceData.performance_mode) > 0);
  success &= (preferences.putUChar(KEY_THEME, deviceData.theme) > 0);
  success &= (preferences.putUShort(KEY_ARMED_TIME, deviceData.armed_time) > 0);
  success &= (preferences.putUChar(KEY_REVISION, deviceData.revision) > 0);
  success &= (preferences.putInt(KEY_TIMEZONE_OFFSET, deviceData.timezone_offset) > 0);

  if (success) {
    preferences.end();
    USBSerial.println(F("Device data saved to Preferences"));
  } else {
    preferences.end();
    USBSerial.println(F("Warning: Some preferences may not have been saved correctly"));
  }
}

// Reset Preferences and deviceData to factory defaults
void resetDeviceData() {
  deviceData = STR_DEVICE_DATA_140_V1();

  // Set the revision to ESP32-S3
  deviceData.revision = 3;  // Set appropriate revision for ESP32-S3

  deviceData.version_major = VERSION_MAJOR;
  deviceData.version_minor = VERSION_MINOR;
  deviceData.screen_rotation = DEFAULT_SCREEN_ROTATION;
  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;
  deviceData.metric_temp = DEFAULT_METRIC_TEMP;
  deviceData.metric_alt = DEFAULT_METRIC_ALT;
  deviceData.performance_mode = DEFAULT_PERFORMANCE_MODE;
  deviceData.theme = DEFAULT_THEME;
  deviceData.armed_time = 0;
  deviceData.timezone_offset = 0;  // Default to UTC

  // Clear all preferences and save defaults
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.clear();
  preferences.end();

  writeDeviceData();
  USBSerial.println(F("Device data reset to defaults and saved to Preferences"));
}

/**
 * Parse commands from Serial connection
 * Handles commands like reboot to bootloader and sync device settings
 */
void parse_serial_commands() {
  if (USBSerial.available()) {
    // With ArduinoJson 7, we no longer need to specify capacity
    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, USBSerial);

    // Handle parsing results
    if (error) {
      // Silently ignore non-JSON input or parsing errors
      return;
    }

    if (doc.containsKey("command")) {
      String command = doc["command"].as<String>();

      if (command == "reboot") {
        USBSerial.println("Rebooting");
        ESP.restart();
        return;
      } else if (command == "sync") {
        send_device_data();
        return;
      }
    }

    // Handle device settings updates
    if (doc.containsKey("settings")) {
      JsonObject settings = doc["settings"];

      if (settings.containsKey("screen_rot")) {
        deviceData.screen_rotation = settings["screen_rot"].as<unsigned int>();
      }

      if (settings.containsKey("sea_pressure")) {
        deviceData.sea_pressure = settings["sea_pressure"].as<float>();
      }

      if (settings.containsKey("metric_temp")) {
        deviceData.metric_temp = settings["metric_temp"].as<bool>();
      }

      if (settings.containsKey("metric_alt")) {
        deviceData.metric_alt = settings["metric_alt"].as<bool>();
      }

      if (settings.containsKey("performance_mode")) {
        deviceData.performance_mode = settings["performance_mode"].as<int>();
      }

      if (settings.containsKey("theme")) {
        deviceData.theme = settings["theme"].as<int>();
      }

      sanitizeDeviceData();
      writeDeviceData();
      // resetRotation(deviceData.screen_rotation);
      // setTheme(deviceData.theme);

      send_device_data();
    }
  }
}

/**
 * Send device data as JSON over Serial
 * Contains current device settings and state
 */
void send_device_data() {
  // With ArduinoJson 7, we no longer need to specify capacity
  JsonDocument doc;

  doc["mj_v"] = VERSION_MAJOR;
  doc["mi_v"] = VERSION_MINOR;
  doc["arch"] = "ESP32S3";
  doc["scr_rt"] = deviceData.screen_rotation;
  doc["ar_tme"] = deviceData.armed_time;
  doc["m_tmp"] = deviceData.metric_temp;
  doc["m_alt"] = deviceData.metric_alt;
  doc["prf"] = deviceData.performance_mode;
  doc["sea_p"] = deviceData.sea_pressure;
  doc["thm"] = deviceData.theme;

  // Send the JSON document over the USBSerial connection
  serializeJson(doc, USBSerial);
  USBSerial.println();  // Add newline for better readability
}

/**
 * Validates and sanitizes device settings to ensure they are within acceptable ranges.
 * Each setting is checked against its valid range and set to a default if invalid.
 *
 * @return true if any values were changed during sanitization, false if all valid
 */
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
  // Ensure theme is either 0 or 1, default to 0
  if (deviceData.theme > 1) {
    deviceData.theme = 0;  // 0=light, 1=dark
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
  USBSerial.println("Hardware Configuration:");
  USBSerial.print("button_top: ");
  USBSerial.println(config.button_top);
  USBSerial.print("buzzer_pin: ");
  USBSerial.println(config.buzzer_pin);
  USBSerial.print("led_sw: ");
  USBSerial.println(config.led_sw);
  USBSerial.print("throttle_pin: ");
  USBSerial.println(config.throttle_pin);
  USBSerial.print("bmp_pin: ");
  USBSerial.println(config.bmp_pin);
  USBSerial.print("tft_rst: ");
  USBSerial.println(config.tft_rst);
  USBSerial.print("tft_cs: ");
  USBSerial.println(config.tft_cs);
  USBSerial.print("tft_dc: ");
  USBSerial.println(config.tft_dc);
  USBSerial.print("spi_mosi: ");
  USBSerial.println(config.spi_mosi);
  USBSerial.print("spi_miso: ");
  USBSerial.println(config.spi_miso);
  USBSerial.print("spi_sclk: ");
  USBSerial.println(config.spi_sclk);
  USBSerial.print("enable_vib: ");
  USBSerial.println(config.enable_vib ? "true" : "false");
  USBSerial.print("enable_neopixel: ");
  USBSerial.println(config.enable_neopixel ? "true" : "false");
}

void updateESCTelemetryBLE(const STR_ESC_TELEMETRY_140& telemetry) {
  if (!deviceConnected) return;

  // Update voltage characteristic
  float voltage = telemetry.volts;
  pESCVoltage->setValue((uint8_t*)&voltage, sizeof(voltage));

  // Update current characteristic
  float current = telemetry.amps;
  pESCCurrent->setValue((uint8_t*)&current, sizeof(current));

  // Update RPM characteristic
  int32_t rpm = telemetry.eRPM;
  pESCRPM->setValue((uint8_t*)&rpm, sizeof(rpm));

  // Create a struct for temperatures
  struct {
    float mos_temp;
    float cap_temp;
    float mcu_temp;
  } temps = {
    telemetry.mos_temp,
    telemetry.cap_temp,
    telemetry.mcu_temp
  };
  pESCTemps->setValue((uint8_t*)&temps, sizeof(temps));
}
