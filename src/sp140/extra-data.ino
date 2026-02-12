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
  deviceData.revision = preferences.getUChar(KEY_REVISION, 0);  // Default to ESP32-S3
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

    if (!doc["command"].isNull()) {
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
