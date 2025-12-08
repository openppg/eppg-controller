#include "sp140/ble/config_service.h"

#include <Arduino.h>
#include <ctime>
#include <cstring>
#include <string>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/ble_utils.h"
#include "sp140/device_state.h"
#include "sp140/globals.h"
#include "sp140/throttle.h"
#include "version.h"

extern void writeDeviceData();
extern QueueHandle_t throttleUpdateQueue;
extern volatile DeviceState currentState;

namespace {

class MetricAltCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != 1) {
      USBSerial.println("Invalid value length - expected 1 byte");
      return;
    }

    deviceData.metric_alt = (value[0] != 0);
    ::writeDeviceData();
    USBSerial.println("Metric alt setting saved to Preferences");
  }
};

class PerformanceModeCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != 1) {
      USBSerial.println("Invalid value length - expected 1 byte");
      return;
    }

    uint8_t mode = value[0];
    if (mode > 1) {
      USBSerial.println("Invalid performance mode value");
      return;
    }

    deviceData.performance_mode = mode;
    ::writeDeviceData();
    USBSerial.println("Performance mode saved to Preferences");
  }
};

class ScreenRotationCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != 1) {
      USBSerial.println("Invalid value length - expected 1 byte");
      return;
    }

    uint8_t rotation = value[0];
    if (rotation != 1 && rotation != 3) {
      USBSerial.println("Invalid rotation value");
      return;
    }

    deviceData.screen_rotation = rotation;
    ::writeDeviceData();
    USBSerial.println("Screen rotation saved to Preferences");
  }
};

class ThrottleValueCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    uint16_t potVal = readThrottleRaw();
    characteristic->setValue(reinterpret_cast<uint8_t*>(&potVal), sizeof(potVal));
  }

  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    if (currentState != ARMED_CRUISING) {
      return;  // Only allow updates while in cruise mode
    }

    std::string value = characteristic->getValue();
    if (value.length() != 2) {
      return;  // Expecting 2 bytes for PWM value
    }

    uint16_t newPWM = (static_cast<uint16_t>(value[0]) << 8) | static_cast<uint16_t>(value[1]);
    if (newPWM < ESC_MIN_PWM || newPWM > ESC_MAX_PWM) {
      return;
    }

    if (xQueueSend(throttleUpdateQueue, &newPWM, pdMS_TO_TICKS(100)) != pdTRUE) {
      USBSerial.println("Failed to queue throttle update");
    }
  }
};

class TimeCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != sizeof(time_t)) {
      USBSerial.println("Invalid timestamp length");
      return;
    }

    time_t timestamp;
    memcpy(&timestamp, value.data(), sizeof(timestamp));
    timestamp += deviceData.timezone_offset;

    struct timeval tv;
    tv.tv_sec = timestamp;
    tv.tv_usec = 0;

    if (settimeofday(&tv, NULL) == 0) {
      USBSerial.println("Time set successfully");
    } else {
      USBSerial.println("Failed to set time");
    }
  }

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    time_t now;
    time(&now);
    now -= deviceData.timezone_offset;
    characteristic->setValue(reinterpret_cast<uint8_t*>(&now), sizeof(now));
  }
};

class TimezoneCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != sizeof(int32_t)) {
      USBSerial.println("Invalid timezone offset length");
      return;
    }

    int32_t offset;
    memcpy(&offset, value.data(), sizeof(offset));
    deviceData.timezone_offset = offset;
    ::writeDeviceData();
    USBSerial.print("Timezone offset set to: ");
    USBSerial.println(offset);
  }

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    characteristic->setValue(
        reinterpret_cast<uint8_t*>(&deviceData.timezone_offset),
        sizeof(deviceData.timezone_offset));
  }
};

class ThemeCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != 1) {
      USBSerial.println("Invalid value length - expected 1 byte");
      return;
    }

    uint8_t theme = value[0];
    if (theme > 1) {
      USBSerial.println("Invalid theme value - must be 0 or 1");
      return;
    }

    deviceData.theme = theme;
    ::writeDeviceData();
    USBSerial.println("Theme setting saved to Preferences");
  }

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    characteristic->setValue(&deviceData.theme, sizeof(deviceData.theme));
  }
};

class SeaPressureCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != sizeof(float)) {
      USBSerial.println("Invalid sea pressure length");
      return;
    }

    float pressure;
    memcpy(&pressure, value.data(), sizeof(pressure));

    // Validate range: 300.0 - 1200.0 hPa/mbar
    if (pressure < 300.0f || pressure > 1200.0f) {
      USBSerial.println("Invalid sea pressure value - must be between 300.0 and 1200.0");
      return;
    }

    deviceData.sea_pressure = pressure;
    ::writeDeviceData();
    USBSerial.print("Sea pressure set to: ");
    USBSerial.println(pressure);
  }

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    characteristic->setValue(
        reinterpret_cast<uint8_t*>(&deviceData.sea_pressure),
        sizeof(deviceData.sea_pressure));
  }
};

class MetricTempCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.length() != 1) {
      USBSerial.println("Invalid value length - expected 1 byte");
      return;
    }

    deviceData.metric_temp = (value[0] != 0);
    ::writeDeviceData();
    USBSerial.println("Metric temp setting saved to Preferences");
  }

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    uint8_t metricTempValue = deviceData.metric_temp ? 1 : 0;
    characteristic->setValue(&metricTempValue, sizeof(metricTempValue));
  }
};

}  // namespace

void initConfigBleService(NimBLEServer* server, const std::string& uniqueId) {
  NimBLEService* configService = server->createService(NimBLEUUID(CONFIG_SERVICE_UUID));

  NimBLECharacteristic* unixTime = configService->createCharacteristic(
      NimBLEUUID(UNIX_TIME_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static TimeCallbacks timeCallbacks;
  unixTime->setCallbacks(&timeCallbacks);

  NimBLECharacteristic* timezone = configService->createCharacteristic(
      NimBLEUUID(TIMEZONE_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static TimezoneCallbacks timezoneCallbacks;
  timezone->setCallbacks(&timezoneCallbacks);
  timezone->setValue(reinterpret_cast<uint8_t*>(&deviceData.timezone_offset),
                     sizeof(deviceData.timezone_offset));

  pDeviceStateCharacteristic = configService->createCharacteristic(
      NimBLEUUID(DEVICE_STATE_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t initialState = DISARMED;
  pDeviceStateCharacteristic->setValue(&initialState, sizeof(initialState));

  NimBLECharacteristic* metricAlt = configService->createCharacteristic(
      NimBLEUUID(METRIC_ALT_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static MetricAltCallbacks metricAltCallbacks;
  metricAlt->setCallbacks(&metricAltCallbacks);
  int metricAltValue = deviceData.metric_alt ? 1 : 0;
  metricAlt->setValue(metricAltValue);

  NimBLECharacteristic* performanceMode = configService->createCharacteristic(
      NimBLEUUID(PERFORMANCE_MODE_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static PerformanceModeCallbacks performanceModeCallbacks;
  performanceMode->setCallbacks(&performanceModeCallbacks);
  int performanceValue = deviceData.performance_mode ? 1 : 0;
  performanceMode->setValue(performanceValue);

  NimBLECharacteristic* screenRotation = configService->createCharacteristic(
      NimBLEUUID(SCREEN_ROTATION_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static ScreenRotationCallbacks screenRotationCallbacks;
  screenRotation->setCallbacks(&screenRotationCallbacks);
  int screenValue = (deviceData.screen_rotation == 1) ? 1 : 3;
  screenRotation->setValue(screenValue);

  NimBLECharacteristic* theme = configService->createCharacteristic(
      NimBLEUUID(THEME_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static ThemeCallbacks themeCallbacks;
  theme->setCallbacks(&themeCallbacks);
  theme->setValue(&deviceData.theme, sizeof(deviceData.theme));

  NimBLECharacteristic* seaPressure = configService->createCharacteristic(
      NimBLEUUID(SEA_PRESSURE_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static SeaPressureCallbacks seaPressureCallbacks;
  seaPressure->setCallbacks(&seaPressureCallbacks);
  seaPressure->setValue(
      reinterpret_cast<uint8_t*>(&deviceData.sea_pressure),
      sizeof(deviceData.sea_pressure));

  NimBLECharacteristic* metricTemp = configService->createCharacteristic(
      NimBLEUUID(METRIC_TEMP_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  static MetricTempCallbacks metricTempCallbacks;
  metricTemp->setCallbacks(&metricTempCallbacks);
  uint8_t metricTempValue = deviceData.metric_temp ? 1 : 0;
  metricTemp->setValue(&metricTempValue, sizeof(metricTempValue));

  NimBLECharacteristic* firmwareVersion = configService->createCharacteristic(
      NimBLEUUID(FW_VERSION_UUID), NIMBLE_PROPERTY::READ);
  uint8_t versionBytes[2] = {VERSION_MAJOR, VERSION_MINOR};
  firmwareVersion->setValue(versionBytes, sizeof(versionBytes));

  NimBLECharacteristic* hardwareRevision = configService->createCharacteristic(
      NimBLEUUID(HW_REVISION_UUID), NIMBLE_PROPERTY::READ);
  hardwareRevision->setValue(&deviceData.revision, sizeof(deviceData.revision));

  NimBLECharacteristic* armedTime = configService->createCharacteristic(
      NimBLEUUID(ARMED_TIME_UUID), NIMBLE_PROPERTY::READ);
  armedTime->setValue(reinterpret_cast<uint8_t*>(&deviceData.armed_time),
                      sizeof(deviceData.armed_time));

  pThrottleCharacteristic = configService->createCharacteristic(
      NimBLEUUID(THROTTLE_VALUE_UUID),
      NIMBLE_PROPERTY::READ |
      NIMBLE_PROPERTY::WRITE |
      NIMBLE_PROPERTY::NOTIFY |
      NIMBLE_PROPERTY::INDICATE);
  static ThrottleValueCallbacks throttleValueCallbacks;
  pThrottleCharacteristic->setCallbacks(&throttleValueCallbacks);

  NimBLEService* deviceInfoService = server->createService(NimBLEUUID(DEVICE_INFO_SERVICE_UUID));
  NimBLECharacteristic* manufacturer = deviceInfoService->createCharacteristic(
      NimBLEUUID(MANUFACTURER_NAME_UUID), NIMBLE_PROPERTY::READ);
  manufacturer->setValue("OpenPPG");

  NimBLECharacteristic* uniqueIdCharacteristic = deviceInfoService->createCharacteristic(
      NimBLEUUID(DEVICE_UNIQUE_ID_UUID), NIMBLE_PROPERTY::READ);
  uniqueIdCharacteristic->setValue(uniqueId);  // Already uppercase string

  configService->start();
  deviceInfoService->start();
}

void updateThrottleBLE(int value) {
  if (pThrottleCharacteristic == nullptr || !deviceConnected) {
    return;
  }

  try {
    pThrottleCharacteristic->setValue(reinterpret_cast<uint8_t*>(&value), sizeof(value));
    pThrottleCharacteristic->notify();
    vTaskDelay(pdMS_TO_TICKS(5));
  } catch (...) {
    USBSerial.println("Error sending BLE notification");
  }
}
