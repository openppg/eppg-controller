#include "sp140/crash_log.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <rom/rtc.h>

#include "version.h"
#include "sp140/device_state.h"

#define DEBUG_SERIAL USBSerial

// NVS keys (max 15 chars, all under namespace "openppg")
static const char* RST_REASON = "rst_reason";
static const char* RST_COUNT  = "rst_count";
static const char* RST_ARMED  = "rst_armed";
static const char* RST_UPTIME = "rst_uptime";
static const char* RST_VER_MAJ = "rst_ver_maj";
static const char* RST_VER_MIN = "rst_ver_min";
static const char* RST_BOOTS  = "rst_boots";

extern DeviceState currentState;

static const char* resetReasonToString(int reason) {
  // ESP-IDF reset reasons (0-15)
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_SW:        return "SW_RESET";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_EXT:       return "EXTERNAL";
    default: break;
  }
  // RTC reset reasons (stored as value + 100)
  if (reason >= 100) {
    switch (reason - 100) {
      case 1:  return "RTC_POWERON";
      case 3:  return "RTC_SW_RESET";
      case 12: return "RTC_SW_CPU_RST";
      case 15: return "RTC_BROWNOUT";
      case 16: return "RTC_SDIO_RST";
      case 9:  return "RTC_DEEPSLEEP";
      case 7:  return "RTC_TG0WDT";
      case 8:  return "RTC_TG1WDT";
      case 11: return "RTC_INT_WDT";
      default: return "RTC_OTHER";
    }
  }
  return "UNKNOWN";
}

static const char* stateToString(uint8_t state) {
  switch (state) {
    case DISARMED:       return "DISARMED";
    case ARMED:          return "ARMED";
    case ARMED_CRUISING: return "ARMED_CRUISING";
    default:             return "UNKNOWN";
  }
}

void crashLogReadAndReport() {
  esp_reset_reason_t currentReason = esp_reset_reason();
  RESET_REASON rtcReason = rtc_get_reset_reason(0);  // Core 0

  // Read previous crash data from NVS
  Preferences prefs;
  prefs.begin("crashlog", true);  // read-only

  uint8_t prevReason   = prefs.getUChar(RST_REASON, 0);
  uint16_t prevCount   = prefs.getUShort(RST_COUNT, 0);
  uint8_t prevArmed    = prefs.getUChar(RST_ARMED, 0);
  uint32_t prevUptime  = prefs.getULong(RST_UPTIME, 0);
  uint8_t prevVerMaj   = prefs.getUChar(RST_VER_MAJ, 0);
  uint8_t prevVerMin   = prefs.getUChar(RST_VER_MIN, 0);
  uint8_t prevBoots    = prefs.getUChar(RST_BOOTS, 0);

  prefs.end();

  // Print previous reset info
  DEBUG_SERIAL.println("=== Previous Reset Info ===");

  if (prevCount == 0 && prevReason == 0) {
    DEBUG_SERIAL.println("No previous crash data stored.");
  } else {
    DEBUG_SERIAL.print("Reset reason: ");
    DEBUG_SERIAL.print(resetReasonToString(prevReason));
    DEBUG_SERIAL.print(" (");
    DEBUG_SERIAL.print(prevReason);
    DEBUG_SERIAL.println(")");

    DEBUG_SERIAL.print("Crash count: ");
    DEBUG_SERIAL.println(prevCount);

    DEBUG_SERIAL.print("Last state: ");
    DEBUG_SERIAL.println(stateToString(prevArmed));

    DEBUG_SERIAL.print("Last uptime: ");
    DEBUG_SERIAL.print(prevUptime);
    DEBUG_SERIAL.print(" ms (");
    DEBUG_SERIAL.print(prevUptime / 60000.0, 1);
    DEBUG_SERIAL.println(" min)");

    DEBUG_SERIAL.print("Last firmware: ");
    DEBUG_SERIAL.print(prevVerMaj);
    DEBUG_SERIAL.print(".");
    DEBUG_SERIAL.println(prevVerMin);
  }

  // Boot loop detection
  uint8_t newBoots = 0;
  if (prevUptime > 0 && prevUptime < BOOT_LOOP_UPTIME_MS) {
    newBoots = prevBoots + 1;
  }

  if (newBoots >= BOOT_LOOP_THRESHOLD) {
    DEBUG_SERIAL.println("!!! BOOT LOOP DETECTED !!!");
    DEBUG_SERIAL.print("Device has crashed ");
    DEBUG_SERIAL.print(newBoots);
    DEBUG_SERIAL.println(" times with <30s uptime each.");
    DEBUG_SERIAL.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  DEBUG_SERIAL.print("Current reset: ");
  DEBUG_SERIAL.print(resetReasonToString(currentReason));
  DEBUG_SERIAL.print(" (esp:");
  DEBUG_SERIAL.print((int)currentReason);
  DEBUG_SERIAL.print(" rtc:");
  DEBUG_SERIAL.print((int)rtcReason);
  DEBUG_SERIAL.println(")");
  DEBUG_SERIAL.println("=== End Reset Info ===");

  // Increment crash count for non-poweron resets
  uint16_t newCount = prevCount;
  if (currentReason != ESP_RST_POWERON && rtcReason != POWERON_RESET) {
    newCount++;
  }

  // Store RTC reason if esp_reset_reason returns UNKNOWN
  uint8_t reasonToStore = (uint8_t)currentReason;
  if (currentReason == ESP_RST_UNKNOWN && rtcReason != NO_MEAN) {
    reasonToStore = (uint8_t)rtcReason + 100;  // Offset to distinguish from esp_reset_reason values
  }

  // Write current boot info to NVS
  prefs.begin("crashlog", false);  // read-write

  prefs.putUChar(RST_REASON, reasonToStore);
  prefs.putUShort(RST_COUNT, newCount);
  prefs.putUChar(RST_ARMED, (uint8_t)DISARMED);
  prefs.putULong(RST_UPTIME, 0);
  prefs.putUChar(RST_VER_MAJ, VERSION_MAJOR);
  prefs.putUChar(RST_VER_MIN, VERSION_MINOR);
  prefs.putUChar(RST_BOOTS, newBoots);

  prefs.end();
}

void crashLogHeartbeat() {
  Preferences prefs;
  prefs.begin("crashlog", false);  // read-write

  prefs.putULong(RST_UPTIME, millis());
  prefs.putUChar(RST_ARMED, (uint8_t)currentState);

  prefs.end();
}

void crashLogUpdateArmedState(DeviceState state) {
  Preferences prefs;
  prefs.begin("crashlog", false);  // read-write

  prefs.putUChar(RST_ARMED, (uint8_t)state);

  prefs.end();
}

void sendCrashLogData() {
  Preferences prefs;
  prefs.begin("crashlog", true);  // read-only

  uint8_t reason    = prefs.getUChar(RST_REASON, 0);
  uint16_t count    = prefs.getUShort(RST_COUNT, 0);
  uint8_t armed     = prefs.getUChar(RST_ARMED, 0);
  uint32_t uptime   = prefs.getULong(RST_UPTIME, 0);
  uint8_t verMaj    = prefs.getUChar(RST_VER_MAJ, 0);
  uint8_t verMin    = prefs.getUChar(RST_VER_MIN, 0);
  uint8_t boots     = prefs.getUChar(RST_BOOTS, 0);

  prefs.end();

  // Manual JSON to avoid ArduinoJson stack usage on WebSerial task
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"crash_log\":{\"reason\":\"%s\",\"code\":%d,\"count\":%d,"
    "\"state\":\"%s\",\"uptime_ms\":%lu,\"fw\":\"%d.%d\","
    "\"rapid_boots\":%d,\"boot_loop\":%s}}",
    resetReasonToString(reason),
    reason, count, stateToString(armed),
    (unsigned long)uptime, verMaj, verMin,
    boots, boots >= BOOT_LOOP_THRESHOLD ? "true" : "false");

  DEBUG_SERIAL.println(buf);
}
