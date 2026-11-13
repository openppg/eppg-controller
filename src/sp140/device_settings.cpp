#include "esp_system.h"
// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include "platform/runtime.h"
#include "platform/json.h"
#include "sp140/device_settings.h"
#include "sp140/globals.h"
#include "../../inc/sp140/esp32s3-config.h"
#include "platform/nvs_store.h"
#include <nvs.h>           // Raw NVS API for the batched (single-commit) write
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "../../inc/sp140/throttle.h"
#include "../../inc/sp140/diagnostics.h"
#include "sp140/wireless_settings.h"

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

// platform::NvsStore namespace
const char* PREFS_NAMESPACE = "openppg";

// platform::NvsStore keys
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

// Read saved data from platform::NvsStore
void refreshDeviceData() {
  // Create the NVS mutex now, while still single-threaded in setup(), so every
  // later (multi-task) writeDeviceData() call takes an already-existing lock.
  prefsEnsureMutex();

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(PREFS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    platform::console.line("Failed to initialize Preferences - may be corrupted");

    // resetDeviceData() wipes the namespace (Preferences.clear()) if a retry
    // can open it, then writes factory defaults.
    platform::console.line("Cleared potentially corrupted preferences, using defaults");
    resetDeviceData();
    return;
  }

  // Check if we have saved settings before (Preferences.isKey(ver_major)).
  uint8_t versionMajorProbe = 0;
  if (nvs_get_u8(handle, KEY_VERSION_MAJOR, &versionMajorProbe) != ESP_OK) {
    platform::console.line("No saved preferences found - initializing with defaults");
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
    platform::console.line("Warning: Invalid screen rotation detected, using default");
    deviceData.screen_rotation = DEFAULT_SCREEN_ROTATION;
    dataValid = false;
  }

  if (deviceData.theme > 1) {
    platform::console.line("Warning: Invalid theme detected, using default");
    deviceData.theme = DEFAULT_THEME;
    dataValid = false;
  }

  nvs_close(handle);

  // Ensure values are within valid ranges
  if (sanitizeDeviceData() || !dataValid) {
    platform::console.line("Sanitized corrupted preference values");
    writeDeviceData();  // Save sanitized values
  }

  platform::console.line("Device data loaded from Preferences");
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
    platform::console.line("Failed to open NVS for writing");
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
    platform::console.line("Device data saved to NVS");
  } else {
    platform::console.line("Warning: Some preferences may not have been saved correctly");
  }
}

// Reset platform::NvsStore and deviceData to factory defaults
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
  platform::console.line("Device data reset to defaults and saved to platform::NvsStore");
}

/**
 * Parse commands from Serial connection
 * Handles commands like reboot to bootloader and sync device settings
 */
void parse_serial_command_line(const char* json_line) {
  if (json_line == nullptr || json_line[0] == '\0') {
    return;
  }

  using platform::json::Document;
  using platform::json::field;
  using platform::json::get;
  using platform::json::has;
  using platform::json::set;
  using platform::json::string;
  Document doc(cJSON_Parse(json_line));
  if (!doc || !cJSON_IsObject(doc.get())) return;
  const char* command = string(doc.get(), "command");
  if (strcmp(command, "reboot") == 0) {
    platform::console.line("Rebooting");
    diagnosticsMarkPlannedRestart(PlannedRestartReason::USB_COMMAND_REBOOT);
    esp_restart();
    return;
  } else if (strcmp(command, "sync") == 0) {
    send_device_data();
    return;
  } else if (strcmp(command, "diag_sync") == 0 || strcmp(command, "diag_clear") == 0) {
    if (strcmp(command, "diag_clear") == 0) diagnosticsClearPersistentData();
    diagnosticsSendJson(platform::console);
    platform::console.line();
    return;
  }
  const cJSON* settings = field(doc.get(), "settings");
  if (cJSON_IsObject(settings)) {
    if (has(settings, "screen_rot")) deviceData.screen_rotation = get<unsigned int>(settings, "screen_rot");
    if (has(settings, "sea_pressure")) deviceData.sea_pressure = get<float>(settings, "sea_pressure");
    if (has(settings, "metric_temp")) deviceData.metric_temp = get<bool>(settings, "metric_temp");
    if (has(settings, "metric_alt")) deviceData.metric_alt = get<bool>(settings, "metric_alt");
    if (has(settings, "performance_mode")) deviceData.performance_mode = get<int>(settings, "performance_mode");
    if (has(settings, "theme")) deviceData.theme = get<int>(settings, "theme");
    sanitizeDeviceData();
    writeDeviceData();
    send_device_data();
  }
  const cJSON* wireless = field(doc.get(), "wireless");
  if (wireless && !cJSON_IsNull(wireless)) {
    if (cJSON_IsObject(wireless) && applyWirelessSettingsJson(wireless)) {
      platform::console.line("Wireless settings saved; reboot required to apply role/radio changes");
      send_device_data();
    } else {
      platform::console.line("Invalid wireless settings (role, peer, 32-hex key, channel 1..13 required)");
    }
  }
}

void poll_serial_commands() {
  while (platform::console.hasInput()) {
    const int byte_read = platform::console.readByte();
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
  using platform::json::Document;
  using platform::json::field;
  using platform::json::get;
  using platform::json::has;
  using platform::json::set;
  using platform::json::string;
  Document doc(cJSON_CreateObject());
  if (!doc) return;

  set(doc.get(), "mj_v", VERSION_MAJOR);
  set(doc.get(), "mi_v", VERSION_MINOR);
  set(doc.get(), "arch", "ESP32S3");
  set(doc.get(), "scr_rt", deviceData.screen_rotation);
  set(doc.get(), "ar_tme", deviceData.armed_time);
  set(doc.get(), "m_tmp", deviceData.metric_temp);
  set(doc.get(), "m_alt", deviceData.metric_alt);
  set(doc.get(), "prf", deviceData.performance_mode);
  set(doc.get(), "sea_p", deviceData.sea_pressure);
  set(doc.get(), "thm", deviceData.theme);
  char peer_mac[18] = {};
  char local_mac[18] = {};
  formatWirelessPeerMac(peer_mac, sizeof(peer_mac));
  formatWirelessLocalMac(local_mac, sizeof(local_mac));
  set(doc.get(), "wireless_role", wirelessRoleName(wirelessSettings.role));
  set(doc.get(), "wireless_mac", local_mac);
  set(doc.get(), "wireless_peer", peer_mac);
  set(doc.get(), "wireless_channel", wirelessSettings.channel);
  set(doc.get(), "wireless_key_set", wirelessSettings.secret_configured);

  // Send the JSON document over the USBSerial connection
  char* output = cJSON_PrintUnformatted(doc.get());
  if (output) { platform::console.text(output); cJSON_free(output); }
  platform::console.line();  // Add newline for better readability
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
  platform::console.line("Hardware Configuration:");
  platform::console.text("button_top: ");
  platform::console.line(config.button_top);
  platform::console.text("buzzer_pin: ");
  platform::console.line(config.buzzer_pin);
  platform::console.text("led_sw: ");
  platform::console.line(config.led_sw);
  platform::console.text("throttle_pin: ");
  platform::console.line(config.throttle_pin);
  platform::console.text("bmp_pin: ");
  platform::console.line(config.bmp_pin);
  platform::console.text("tft_rst: ");
  platform::console.line(config.tft_rst);
  platform::console.text("tft_cs: ");
  platform::console.line(config.tft_cs);
  platform::console.text("tft_dc: ");
  platform::console.line(config.tft_dc);
  platform::console.text("spi_mosi: ");
  platform::console.line(config.spi_mosi);
  platform::console.text("spi_miso: ");
  platform::console.line(config.spi_miso);
  platform::console.text("spi_sclk: ");
  platform::console.line(config.spi_sclk);
  platform::console.text("enable_vib: ");
  platform::console.line(config.enable_vib ? "true" : "false");
  platform::console.text("enable_neopixel: ");
  platform::console.line(config.enable_neopixel ? "true" : "false");
}
