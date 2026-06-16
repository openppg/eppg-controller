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
#include "sp140/ble/ota_service.h"
#include "sp140/esc_config_relay.h"

extern void writeDeviceData();
extern QueueHandle_t throttleUpdateQueue;
extern volatile DeviceState currentState;

namespace {

constexpr uint32_t kReadSecure = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC;
constexpr uint32_t kReadWriteSecure =
NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC;
constexpr uint32_t kNotifyReadSecure =
NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY |
NIMBLE_PROPERTY::READ_ENC;
constexpr uint32_t kReadWriteNotifyIndicateSecure =
NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE |
NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC;

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
    uint16_t potVal = getLastThrottleRaw();
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

// ESC config relay command characteristic. The app writes an opcode-multiplexed
// payload; we parse it and hand a single-parameter request to the relay module,
// which runs the write/save/restart/verify on the throttle task (CAN owner).
//   0x10 SET_PARAM / 0x1F SET_AND_COMMIT : [op][config_id u16 LE][len u8][data...]
// The DISARMED gate is enforced inside the relay (status reports REJECTED_ARMED).
class EscRelayCmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string value = characteristic->getValue();
    if (value.size() < 1) return;
    const uint8_t op = static_cast<uint8_t>(value[0]);
    switch (op) {
      case 0x10:   // SET_PARAM   (single param, write + commit + verify)
      case 0x1F: {  // SET_AND_COMMIT
        if (value.size() < 4) return;  // op + config_id(2) + len(1)
        uint16_t configId = static_cast<uint8_t>(value[1]) |
                            (static_cast<uint16_t>(static_cast<uint8_t>(value[2])) << 8);
        uint8_t len = static_cast<uint8_t>(value[3]);
        if (len > 48 || value.size() < static_cast<size_t>(4 + len)) return;
        bool accepted = escConfigRelayRequestSetParam(
            configId, reinterpret_cast<const uint8_t*>(value.data()) + 4, len);
        USBSerial.printf("ESC relay cmd op=0x%02X id=0x%04X len=%d accepted=%d\n",
                         op, configId, len, accepted);
        break;
      }
      default:
        USBSerial.printf("ESC relay: unknown opcode 0x%02X\n", op);
        break;
    }
  }
};

// ESC config relay status characteristic (read + notify). Returns the latched
// session status so the app can poll until a terminal result (the design's
// poll-until-terminal contract, which also survives a BLE drop + reconnect).
//   [code][config_id u16 LE][phase][detail][progress u16 LE][readback...]
class EscRelayStatusCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    EscRelayStatus s = escConfigRelayGetStatus();
    uint8_t buf[16];
    buf[0] = static_cast<uint8_t>(s.code);
    buf[1] = s.config_id & 0xFF;
    buf[2] = (s.config_id >> 8) & 0xFF;
    buf[3] = static_cast<uint8_t>(s.phase);
    buf[4] = s.detail;
    buf[5] = 0;  // progress permille LSB (config = 0)
    buf[6] = 0;  // progress permille MSB
    uint8_t n = (s.readback_len > 8) ? 8 : s.readback_len;
    for (uint8_t i = 0; i < n; i++) buf[7 + i] = s.readback[i];
    characteristic->setValue(buf, 7 + n);
  }
};

}  // namespace

void initConfigBleService(NimBLEServer* server, const std::string& uniqueId) {
  NimBLEService* configService = server->createService(NimBLEUUID(CONFIG_SERVICE_UUID));

  NimBLECharacteristic* unixTime = configService->createCharacteristic(
      NimBLEUUID(UNIX_TIME_UUID),
      kReadWriteSecure);
  static TimeCallbacks timeCallbacks;
  unixTime->setCallbacks(&timeCallbacks);

  NimBLECharacteristic* timezone = configService->createCharacteristic(
      NimBLEUUID(TIMEZONE_UUID),
      kReadWriteSecure);
  static TimezoneCallbacks timezoneCallbacks;
  timezone->setCallbacks(&timezoneCallbacks);
  timezone->setValue(reinterpret_cast<uint8_t*>(&deviceData.timezone_offset),
                     sizeof(deviceData.timezone_offset));

  pDeviceStateCharacteristic = configService->createCharacteristic(
      NimBLEUUID(DEVICE_STATE_UUID),
      kNotifyReadSecure);
  uint8_t initialState = DISARMED;
  pDeviceStateCharacteristic->setValue(&initialState, sizeof(initialState));

  NimBLECharacteristic* metricAlt = configService->createCharacteristic(
      NimBLEUUID(METRIC_ALT_UUID),
      kReadWriteSecure);
  static MetricAltCallbacks metricAltCallbacks;
  metricAlt->setCallbacks(&metricAltCallbacks);
  int metricAltValue = deviceData.metric_alt ? 1 : 0;
  metricAlt->setValue(metricAltValue);

  NimBLECharacteristic* performanceMode = configService->createCharacteristic(
      NimBLEUUID(PERFORMANCE_MODE_UUID),
      kReadWriteSecure);
  static PerformanceModeCallbacks performanceModeCallbacks;
  performanceMode->setCallbacks(&performanceModeCallbacks);
  int performanceValue = deviceData.performance_mode ? 1 : 0;
  performanceMode->setValue(performanceValue);

  NimBLECharacteristic* screenRotation = configService->createCharacteristic(
      NimBLEUUID(SCREEN_ROTATION_UUID),
      kReadWriteSecure);
  static ScreenRotationCallbacks screenRotationCallbacks;
  screenRotation->setCallbacks(&screenRotationCallbacks);
  int screenValue = (deviceData.screen_rotation == 1) ? 1 : 3;
  screenRotation->setValue(screenValue);

  NimBLECharacteristic* theme = configService->createCharacteristic(
      NimBLEUUID(THEME_UUID),
      kReadWriteSecure);
  static ThemeCallbacks themeCallbacks;
  theme->setCallbacks(&themeCallbacks);
  theme->setValue(&deviceData.theme, sizeof(deviceData.theme));

  NimBLECharacteristic* seaPressure = configService->createCharacteristic(
      NimBLEUUID(SEA_PRESSURE_UUID),
      kReadWriteSecure);
  static SeaPressureCallbacks seaPressureCallbacks;
  seaPressure->setCallbacks(&seaPressureCallbacks);
  seaPressure->setValue(
      reinterpret_cast<uint8_t*>(&deviceData.sea_pressure),
      sizeof(deviceData.sea_pressure));

  NimBLECharacteristic* metricTemp = configService->createCharacteristic(
      NimBLEUUID(METRIC_TEMP_UUID),
      kReadWriteSecure);
  static MetricTempCallbacks metricTempCallbacks;
  metricTemp->setCallbacks(&metricTempCallbacks);
  uint8_t metricTempValue = deviceData.metric_temp ? 1 : 0;
  metricTemp->setValue(&metricTempValue, sizeof(metricTempValue));

  NimBLECharacteristic* firmwareVersion = configService->createCharacteristic(
      NimBLEUUID(FW_VERSION_UUID), kReadSecure);
  // Include an internal build number so the app can distinguish same-version OTA releases.
  uint8_t versionBytes[6] = {
      VERSION_MAJOR,
      VERSION_MINOR,
      static_cast<uint8_t>(VERSION_BUILD & 0xFF),
      static_cast<uint8_t>((VERSION_BUILD >> 8) & 0xFF),
      static_cast<uint8_t>((VERSION_BUILD >> 16) & 0xFF),
      static_cast<uint8_t>((VERSION_BUILD >> 24) & 0xFF),
  };
  firmwareVersion->setValue(versionBytes, sizeof(versionBytes));

  NimBLECharacteristic* hardwareRevision = configService->createCharacteristic(
      NimBLEUUID(HW_REVISION_UUID), kReadSecure);
  hardwareRevision->setValue(&deviceData.revision, sizeof(deviceData.revision));

  NimBLECharacteristic* armedTime = configService->createCharacteristic(
      NimBLEUUID(ARMED_TIME_UUID), kReadSecure);
  armedTime->setValue(reinterpret_cast<uint8_t*>(&deviceData.armed_time),
                      sizeof(deviceData.armed_time));

  pThrottleCharacteristic = configService->createCharacteristic(
      NimBLEUUID(THROTTLE_VALUE_UUID),
      kReadWriteNotifyIndicateSecure);
  static ThrottleValueCallbacks throttleValueCallbacks;
  pThrottleCharacteristic->setCallbacks(&throttleValueCallbacks);

  // ESC config relay: command (write) + status (read/notify).
  NimBLECharacteristic* escRelayCmd = configService->createCharacteristic(
      NimBLEUUID(ESC_RELAY_CMD_UUID), kReadWriteSecure);
  static EscRelayCmdCallbacks escRelayCmdCallbacks;
  escRelayCmd->setCallbacks(&escRelayCmdCallbacks);

  NimBLECharacteristic* escRelayStatus = configService->createCharacteristic(
      NimBLEUUID(ESC_RELAY_STATUS_UUID), kNotifyReadSecure);
  static EscRelayStatusCallbacks escRelayStatusCallbacks;
  escRelayStatus->setCallbacks(&escRelayStatusCallbacks);

  NimBLEService* deviceInfoService = server->createService(NimBLEUUID(DEVICE_INFO_SERVICE_UUID));
  NimBLECharacteristic* manufacturer = deviceInfoService->createCharacteristic(
      NimBLEUUID(MANUFACTURER_NAME_UUID), kReadSecure);
  manufacturer->setValue("OpenPPG");

  NimBLECharacteristic* uniqueIdCharacteristic = deviceInfoService->createCharacteristic(
      NimBLEUUID(DEVICE_UNIQUE_ID_UUID), kReadSecure);
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
    if (!isOtaInProgress()) {
      pThrottleCharacteristic->notify();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  } catch (...) {
    USBSerial.println("Error sending BLE notification");
  }
}
