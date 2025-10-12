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
#include "sp140/device_state.h"
#include "sp140/globals.h"
#include "sp140/throttle.h"
#include "version.h"

extern void writeDeviceData();
extern QueueHandle_t throttleUpdateQueue;
extern volatile DeviceState currentState;

namespace {

class MetricAltCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
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

class PerformanceModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
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

class ScreenRotationCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
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

class ThrottleValueCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* characteristic) override {
    uint16_t potVal = readThrottleRaw();
    characteristic->setValue(reinterpret_cast<uint8_t*>(&potVal), sizeof(potVal));
  }

  void onWrite(BLECharacteristic* characteristic) override {
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

class TimeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
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

  void onRead(BLECharacteristic* characteristic) override {
    time_t now;
    time(&now);
    now -= deviceData.timezone_offset;
    characteristic->setValue(reinterpret_cast<uint8_t*>(&now), sizeof(now));
  }
};

class TimezoneCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
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

  void onRead(BLECharacteristic* characteristic) override {
    characteristic->setValue(
        reinterpret_cast<uint8_t*>(&deviceData.timezone_offset),
        sizeof(deviceData.timezone_offset));
  }
};

}  // namespace

void initConfigBleService(BLEServer* server, const std::string& uniqueId) {
  BLEService* configService = server->createService(BLEUUID(CONFIG_SERVICE_UUID), 30);

  BLECharacteristic* unixTime = configService->createCharacteristic(
      BLEUUID(UNIX_TIME_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  unixTime->setCallbacks(new TimeCallbacks());

  BLECharacteristic* timezone = configService->createCharacteristic(
      BLEUUID(TIMEZONE_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  timezone->setCallbacks(new TimezoneCallbacks());
  timezone->setValue(reinterpret_cast<uint8_t*>(&deviceData.timezone_offset),
                     sizeof(deviceData.timezone_offset));

  pDeviceStateCharacteristic = configService->createCharacteristic(
      BLEUUID(DEVICE_STATE_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  uint8_t initialState = DISARMED;
  pDeviceStateCharacteristic->setValue(&initialState, sizeof(initialState));
  pDeviceStateCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* metricAlt = configService->createCharacteristic(
      BLEUUID(METRIC_ALT_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  metricAlt->setCallbacks(new MetricAltCallbacks());
  int metricAltValue = deviceData.metric_alt ? 1 : 0;
  metricAlt->setValue(metricAltValue);

  BLECharacteristic* performanceMode = configService->createCharacteristic(
      BLEUUID(PERFORMANCE_MODE_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  performanceMode->setCallbacks(new PerformanceModeCallbacks());
  int performanceValue = deviceData.performance_mode ? 1 : 0;
  performanceMode->setValue(performanceValue);

  BLECharacteristic* screenRotation = configService->createCharacteristic(
      BLEUUID(SCREEN_ROTATION_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  screenRotation->setCallbacks(new ScreenRotationCallbacks());
  int screenValue = (deviceData.screen_rotation == 1) ? 1 : 3;
  screenRotation->setValue(screenValue);

  BLECharacteristic* firmwareVersion = configService->createCharacteristic(
      BLEUUID(FW_VERSION_UUID), BLECharacteristic::PROPERTY_READ);
  uint8_t versionBytes[2] = {VERSION_MAJOR, VERSION_MINOR};
  firmwareVersion->setValue(versionBytes, sizeof(versionBytes));

  BLECharacteristic* hardwareRevision = configService->createCharacteristic(
      BLEUUID(HW_REVISION_UUID), BLECharacteristic::PROPERTY_READ);
  hardwareRevision->setValue(&deviceData.revision, sizeof(deviceData.revision));

  BLECharacteristic* armedTime = configService->createCharacteristic(
      BLEUUID(ARMED_TIME_UUID), BLECharacteristic::PROPERTY_READ);
  armedTime->setValue(reinterpret_cast<uint8_t*>(&deviceData.armed_time),
                      sizeof(deviceData.armed_time));

  pThrottleCharacteristic = configService->createCharacteristic(
      BLEUUID(THROTTLE_VALUE_UUID),
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_INDICATE);
  pThrottleCharacteristic->setCallbacks(new ThrottleValueCallbacks());
  pThrottleCharacteristic->addDescriptor(new BLE2902());

  BLEService* deviceInfoService = server->createService(BLEUUID(DEVICE_INFO_SERVICE_UUID), 10);
  BLECharacteristic* manufacturer = deviceInfoService->createCharacteristic(
      BLEUUID(MANUFACTURER_NAME_UUID), BLECharacteristic::PROPERTY_READ);
  manufacturer->setValue("OpenPPG");

  BLECharacteristic* uniqueIdCharacteristic = deviceInfoService->createCharacteristic(
      BLEUUID(DEVICE_UNIQUE_ID_UUID), BLECharacteristic::PROPERTY_READ);
  uniqueIdCharacteristic->setValue(uniqueId);  // Already uppercase string

  configService->start();
  deviceInfoService->start();
}

void updateThrottleBLE(int value) {
  if (!deviceConnected && oldDeviceConnected) {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (pServer != nullptr) {
      pServer->startAdvertising();
    }
    USBSerial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  if (!deviceConnected || pThrottleCharacteristic == nullptr) {
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
