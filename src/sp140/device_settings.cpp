// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include "Arduino.h"
#include <ArduinoJson.h>
#include "sp140/device_settings.h"
#include "sp140/globals.h"
#include "../../inc/sp140/esp32s3-config.h"
#include <nvs.h>  // Raw NVS API for both load and the batched (single-commit) write
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "../../inc/sp140/throttle.h"
#include "../../inc/sp140/diagnostics.h"

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

// NVS namespace (Arduino Preferences used the same name)
const char* PREFS_NAMESPACE = "openppg";

// NVS keys — names and types are byte-compatible with field devices:
//   u8:   version, rotation, bools (0/1), performance_mode, theme, revision
//   blob: sea_pressure (4-byte float, same as Preferences getFloat/putFloat)
//   u16:  armed_time
//   i32:  timezone_offset
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

// Serializes all NVS settings access. writeDeviceData() is reachable
// concurrently from the BLE host task (config-service onWrite callbacks), the
// button task (disarmSystem) and the serial task (parse_serial_command_line);
// the single shared NVS namespace was previously written with no lock, so two
// writers could interleave and silently lose/corrupt a settings write. Created
// once in refreshDeviceData() while still single-threaded in setup().
static SemaphoreHandle_t s_prefsMutex = nullptr;

static void prefsEnsureMutex() {
  if (s_prefsMutex == nullptr) {
    s_prefsMutex = xSemaphoreCreateMutex();
  }
}

namespace {

constexpr size_t kWebSerialCommandBufferSize = 256;
char gWebSerialCommandBuffer[kWebSerialCommandBufferSize] = {};
size_t gWebSerialCommandLength = 0;
bool gWebSerialCommandOverflow = false;

}  // namespace

// Read saved data from NVS (same key layout the write path and older
// Preferences-based firmware use, so field devices stay compatible).
void refreshDeviceData() {
  // Create the NVS mutex now, while still single-threaded in setup(), so every
  // later (multi-task) writeDeviceData() call takes an already-existing lock.
  prefsEnsureMutex();

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(PREFS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    USBSerial.println(F("Failed to initialize Preferences - may be corrupted"));

    // resetDeviceData() wipes the namespace (Preferences.clear()) if a retry
    // can open it, then writes factory defaults.
    USBSerial.println(F("Cleared potentially corrupted preferences, using defaults"));
    resetDeviceData();
    return;
  }

  // Check if we have saved settings before (Preferences.isKey(ver_major)).
  uint8_t versionMajorProbe = 0;
  if (nvs_get_u8(handle, KEY_VERSION_MAJOR, &versionMajorProbe) != ESP_OK) {
    USBSerial.println(F("No saved preferences found - initializing with defaults"));
    nvs_close(handle);
    resetDeviceData();
    return;
  }

  // Load all values. On miss, nvs_get_* leaves the pre-set default (same as
  // Preferences). Bools are u8 0/1; sea_pressure is a 4-byte blob.
  bool dataValid = true;

  deviceData.version_major = VERSION_MAJOR;
  nvs_get_u8(handle, KEY_VERSION_MAJOR, &deviceData.version_major);
  deviceData.version_minor = VERSION_MINOR;
  nvs_get_u8(handle, KEY_VERSION_MINOR, &deviceData.version_minor);
  deviceData.screen_rotation = DEFAULT_SCREEN_ROTATION;
  nvs_get_u8(handle, KEY_SCREEN_ROTATION, &deviceData.screen_rotation);

  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;
  size_t sea_pressure_len = sizeof(deviceData.sea_pressure);
  if (nvs_get_blob(handle, KEY_SEA_PRESSURE, &deviceData.sea_pressure,
                   &sea_pressure_len) != ESP_OK ||
      sea_pressure_len != sizeof(deviceData.sea_pressure)) {
    deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;
  }

  uint8_t metric_temp = DEFAULT_METRIC_TEMP ? 1 : 0;
  nvs_get_u8(handle, KEY_METRIC_TEMP, &metric_temp);
  deviceData.metric_temp = (metric_temp == 1);
  uint8_t metric_alt = DEFAULT_METRIC_ALT ? 1 : 0;
  nvs_get_u8(handle, KEY_METRIC_ALT, &metric_alt);
  deviceData.metric_alt = (metric_alt == 1);

  deviceData.performance_mode = DEFAULT_PERFORMANCE_MODE;
  nvs_get_u8(handle, KEY_PERFORMANCE_MODE, &deviceData.performance_mode);
  deviceData.theme = DEFAULT_THEME;
  nvs_get_u8(handle, KEY_THEME, &deviceData.theme);
  deviceData.armed_time = 0;
  nvs_get_u16(handle, KEY_ARMED_TIME, &deviceData.armed_time);
  deviceData.revision = 0;  // Default to ESP32-S3
  nvs_get_u8(handle, KEY_REVISION, &deviceData.revision);
  deviceData.timezone_offset = 0;
  nvs_get_i32(handle, KEY_TIMEZONE_OFFSET, &deviceData.timezone_offset);

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

  nvs_close(handle);

  // Ensure values are within valid ranges
  if (sanitizeDeviceData() || !dataValid) {
    USBSerial.println(F("Sanitized corrupted preference values"));
    writeDeviceData();  // Save sanitized values
  }

  USBSerial.println(F("Device data loaded from Preferences"));
}

// Write to NVS. Uses the raw NVS API so all 11 keys share ONE nvs_commit()
// instead of committing per key — a single flash transaction that, together
// with s_prefsMutex, removes any window in which a concurrent writer could
// observe or produce a torn/partial save. Key names and NVS value types MATCH
// both refreshDeviceData() and older Preferences-based firmware (u8/u16/i32,
// and a 4-byte blob for the float), so field data remains fully readable.
void writeDeviceData() {
  prefsEnsureMutex();
  if (s_prefsMutex != nullptr) {
    xSemaphoreTake(s_prefsMutex, portMAX_DELAY);
  }

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(PREFS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    USBSerial.println(F("Failed to open NVS for writing"));
    if (s_prefsMutex != nullptr) {
      xSemaphoreGive(s_prefsMutex);
    }
    return;
  }

  bool success = true;
  success &= (nvs_set_u8(handle, KEY_VERSION_MAJOR, deviceData.version_major) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_VERSION_MINOR, deviceData.version_minor) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_SCREEN_ROTATION, deviceData.screen_rotation) == ESP_OK);
  success &= (nvs_set_blob(handle, KEY_SEA_PRESSURE, &deviceData.sea_pressure,
                           sizeof(deviceData.sea_pressure)) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_METRIC_TEMP, deviceData.metric_temp ? 1 : 0) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_METRIC_ALT, deviceData.metric_alt ? 1 : 0) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_PERFORMANCE_MODE, deviceData.performance_mode) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_THEME, deviceData.theme) == ESP_OK);
  success &= (nvs_set_u16(handle, KEY_ARMED_TIME, deviceData.armed_time) == ESP_OK);
  success &= (nvs_set_u8(handle, KEY_REVISION, deviceData.revision) == ESP_OK);
  success &= (nvs_set_i32(handle, KEY_TIMEZONE_OFFSET, deviceData.timezone_offset) == ESP_OK);

  // One commit for the whole settings blob.
  success &= (nvs_commit(handle) == ESP_OK);
  nvs_close(handle);

  if (s_prefsMutex != nullptr) {
    xSemaphoreGive(s_prefsMutex);
  }

  if (success) {
    USBSerial.println(F("Device data saved to Preferences"));
  } else {
    USBSerial.println(F("Warning: Some preferences may not have been saved correctly"));
  }
}

// Reset Preferences and deviceData to factory defaults
void resetDeviceData() {
  deviceData = STR_DEVICE_DATA_140_V1();

  // Set the revision to ESP32-S3
  deviceData.revision = 0;  // Set appropriate revision for ESP32-S3

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

  // Clear all keys in this namespace (Preferences.clear()) and save defaults
  nvs_handle_t handle = 0;
  if (nvs_open(PREFS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
    nvs_erase_all(handle);
    nvs_commit(handle);
    nvs_close(handle);
  }

  writeDeviceData();
  USBSerial.println(F("Device data reset to defaults and saved to Preferences"));
}

/**
 * Parse commands from Serial connection
 * Handles commands like reboot to bootloader and sync device settings
 */
void parse_serial_command_line(const char* json_line) {
  if (json_line == nullptr || json_line[0] == '\0') {
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json_line);
  if (error) {
    return;
  }

  if (!doc["command"].isNull()) {
    String command = doc["command"].as<String>();

    if (command == "reboot") {
      USBSerial.println("Rebooting");
      diagnosticsMarkPlannedRestart(
          PlannedRestartReason::USB_COMMAND_REBOOT);
      ESP.restart();
      return;
    } else if (command == "sync") {
      send_device_data();
      return;
    } else if (command == "diag_sync") {
      diagnosticsSendJson(USBSerial);
      USBSerial.println();
      return;
    } else if (command == "diag_clear") {
      diagnosticsClearPersistentData();
      diagnosticsSendJson(USBSerial);
      USBSerial.println();
      return;
    }
  }

  if (!doc["settings"].isNull()) {
    JsonObject settings = doc["settings"].as<JsonObject>();

    if (!settings.isNull()) {
      if (!settings["screen_rot"].isNull()) {
        deviceData.screen_rotation = settings["screen_rot"].as<unsigned int>();
      }

      if (!settings["sea_pressure"].isNull()) {
        deviceData.sea_pressure = settings["sea_pressure"].as<float>();
      }

      if (!settings["metric_temp"].isNull()) {
        deviceData.metric_temp = settings["metric_temp"].as<bool>();
      }

      if (!settings["metric_alt"].isNull()) {
        deviceData.metric_alt = settings["metric_alt"].as<bool>();
      }

      if (!settings["performance_mode"].isNull()) {
        deviceData.performance_mode = settings["performance_mode"].as<int>();
      }

      if (!settings["theme"].isNull()) {
        deviceData.theme = settings["theme"].as<int>();
      }
    }

    sanitizeDeviceData();
    writeDeviceData();
    send_device_data();
  }
}

void poll_serial_commands() {
  while (USBSerial.available()) {
    const int byte_read = USBSerial.read();
    if (byte_read < 0) {
      return;
    }

    const char ch = static_cast<char>(byte_read);
    if (ch == '\r') {
      continue;
    }

    if (gWebSerialCommandOverflow) {
      if (ch == '\n') {
        gWebSerialCommandOverflow = false;
        gWebSerialCommandLength = 0;
      }
      continue;
    }

    if (ch == '\n') {
      if (gWebSerialCommandLength == 0) {
        continue;
      }

      gWebSerialCommandBuffer[gWebSerialCommandLength] = '\0';
      parse_serial_command_line(gWebSerialCommandBuffer);
      gWebSerialCommandLength = 0;
      return;
    }

    if (gWebSerialCommandLength + 1 >= kWebSerialCommandBufferSize) {
      gWebSerialCommandOverflow = true;
      gWebSerialCommandLength = 0;
      continue;
    }

    gWebSerialCommandBuffer[gWebSerialCommandLength++] = ch;
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
