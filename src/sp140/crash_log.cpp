#include "sp140/crash_log.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <rom/rtc.h>

#include "version.h"
#include "sp140/device_state.h"

#define DEBUG_SERIAL USBSerial
#define CRASH_NAMESPACE "crashlog"
#define MAX_HISTORY 5

extern DeviceState currentState;

// Packed reset entry: 8 bytes per entry
// Stored as raw bytes in NVS blob
struct __attribute__((packed)) ResetEntry {
  uint8_t espReason;    // esp_reset_reason_t
  uint8_t rtcReason;    // RTC reset reason (raw)
  uint8_t armedState;   // DeviceState at time of reset
  uint8_t verMajor;     // Firmware major version
  uint32_t uptimeMs;    // Uptime at last heartbeat
};

// NVS keys
static const char* KEY_HISTORY = "rst_hist";     // Blob: array of ResetEntry
static const char* KEY_COUNT   = "rst_count";    // uint16_t: total reset count
static const char* KEY_BOOTS   = "rst_boots";    // uint8_t: rapid boot counter
static const char* KEY_ARMED   = "rst_armed";    // uint8_t: current armed state (heartbeat)
static const char* KEY_UPTIME  = "rst_uptime";   // uint32_t: current uptime (heartbeat)

static const char* espReasonStr(int reason) {
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
    default:                return "UNKNOWN";
  }
}

static const char* rtcReasonStr(int reason) {
  switch (reason) {
    case 1:  return "POWERON";
    case 3:  return "SW_SYS";
    case 5:  return "DEEPSLEEP";
    case 7:  return "TG0WDT";
    case 8:  return "TG1WDT";
    case 9:  return "RTCWDT";
    case 11: return "INT_WDT";
    case 12: return "SW_CPU";
    case 14: return "EFUSE";
    case 15: return "USB_UART";
    case 16: return "USB_JTAG";
    case 17: return "PWR_GLITCH";
    case 18: return "JTAG";
    default: return "OTHER";
  }
}

static const char* stateToString(uint8_t state) {
  switch (state) {
    case DISARMED:       return "DISARMED";
    case ARMED:          return "ARMED";
    case ARMED_CRUISING: return "CRUISING";
    default:             return "?";
  }
}

void crashLogReadAndReport() {
  esp_reset_reason_t espReason = esp_reset_reason();
  RESET_REASON rtcReason = rtc_get_reset_reason(0);

  // Read existing history from NVS
  Preferences prefs;
  prefs.begin(CRASH_NAMESPACE, true);  // read-only

  ResetEntry history[MAX_HISTORY] = {};
  size_t histLen = prefs.getBytesLength(KEY_HISTORY);
  int entryCount = 0;
  if (histLen > 0 && histLen <= sizeof(history)) {
    prefs.getBytes(KEY_HISTORY, history, histLen);
    entryCount = histLen / sizeof(ResetEntry);
  }

  uint16_t totalCount  = prefs.getUShort(KEY_COUNT, 0);
  uint8_t rapidBoots   = prefs.getUChar(KEY_BOOTS, 0);
  uint8_t lastArmed    = prefs.getUChar(KEY_ARMED, 0);
  uint32_t lastUptime  = prefs.getULong(KEY_UPTIME, 0);

  prefs.end();

  // Print reset history
  DEBUG_SERIAL.println("=== Reset History ===");
  DEBUG_SERIAL.print("Total resets: ");
  DEBUG_SERIAL.println(totalCount);

  if (entryCount > 0) {
    // Update the most recent entry with heartbeat data (armed state + uptime)
    // since those are written separately from the history blob
    history[0].armedState = lastArmed;
    history[0].uptimeMs = lastUptime;

    for (int i = 0; i < entryCount; i++) {
      ResetEntry& e = history[i];
      DEBUG_SERIAL.print(i == 0 ? " [PREV] " : "   [");
      if (i > 0) { DEBUG_SERIAL.print(i); DEBUG_SERIAL.print("]   "); }
      DEBUG_SERIAL.print("esp:");
      DEBUG_SERIAL.print(espReasonStr(e.espReason));
      DEBUG_SERIAL.print("(");
      DEBUG_SERIAL.print(e.espReason);
      DEBUG_SERIAL.print(") rtc:");
      DEBUG_SERIAL.print(rtcReasonStr(e.rtcReason));
      DEBUG_SERIAL.print("(");
      DEBUG_SERIAL.print(e.rtcReason);
      DEBUG_SERIAL.print(") state:");
      DEBUG_SERIAL.print(stateToString(e.armedState));
      DEBUG_SERIAL.print(" up:");
      DEBUG_SERIAL.print(e.uptimeMs / 1000);
      DEBUG_SERIAL.print("s fw:");
      DEBUG_SERIAL.print(e.verMajor / 10);
      DEBUG_SERIAL.print(".");
      DEBUG_SERIAL.println(e.verMajor % 10);
    }
  } else {
    DEBUG_SERIAL.println("No previous reset data.");
  }

  // Boot loop detection
  uint8_t newBoots = 0;
  if (lastUptime > 0 && lastUptime < BOOT_LOOP_UPTIME_MS) {
    newBoots = rapidBoots + 1;
  }
  if (newBoots >= BOOT_LOOP_THRESHOLD) {
    DEBUG_SERIAL.println("!!! BOOT LOOP DETECTED !!!");
  }

  DEBUG_SERIAL.print("Current: esp:");
  DEBUG_SERIAL.print(espReasonStr(espReason));
  DEBUG_SERIAL.print("(");
  DEBUG_SERIAL.print((int)espReason);
  DEBUG_SERIAL.print(") rtc:");
  DEBUG_SERIAL.print(rtcReasonStr(rtcReason));
  DEBUG_SERIAL.print("(");
  DEBUG_SERIAL.print((int)rtcReason);
  DEBUG_SERIAL.println(")");
  DEBUG_SERIAL.println("=== End Reset History ===");

  // Shift history down and insert new entry at position 0
  ResetEntry newHistory[MAX_HISTORY] = {};
  newHistory[0].espReason = (uint8_t)espReason;
  newHistory[0].rtcReason = (uint8_t)rtcReason;
  newHistory[0].armedState = (uint8_t)DISARMED;
  // Pack version: major*10 + minor (fits in uint8_t for versions like 7.6 = 76)
  newHistory[0].verMajor = VERSION_MAJOR * 10 + VERSION_MINOR;
  newHistory[0].uptimeMs = 0;

  // Copy previous entries, shifting by 1 (drop oldest if full)
  int copyCount = (entryCount < MAX_HISTORY - 1) ? entryCount : MAX_HISTORY - 1;
  for (int i = 0; i < copyCount; i++) {
    newHistory[i + 1] = history[i];
    // Patch the entry we're shifting: update its armed/uptime from heartbeat
    if (i == 0) {
      newHistory[1].armedState = lastArmed;
      newHistory[1].uptimeMs = lastUptime;
    }
  }
  int newEntryCount = copyCount + 1;

  // Increment total count for non-poweron resets
  uint16_t newCount = totalCount;
  if (espReason != ESP_RST_POWERON) {
    newCount++;
  }

  // Write updated history
  prefs.begin(CRASH_NAMESPACE, false);  // read-write

  prefs.putBytes(KEY_HISTORY, newHistory, newEntryCount * sizeof(ResetEntry));
  prefs.putUShort(KEY_COUNT, newCount);
  prefs.putUChar(KEY_BOOTS, newBoots);
  prefs.putUChar(KEY_ARMED, (uint8_t)DISARMED);
  prefs.putULong(KEY_UPTIME, 0);

  prefs.end();
}

void crashLogHeartbeat() {
  Preferences prefs;
  prefs.begin(CRASH_NAMESPACE, false);

  prefs.putULong(KEY_UPTIME, millis());
  prefs.putUChar(KEY_ARMED, (uint8_t)currentState);

  prefs.end();
}

void crashLogUpdateArmedState(DeviceState state) {
  Preferences prefs;
  prefs.begin(CRASH_NAMESPACE, false);

  prefs.putUChar(KEY_ARMED, (uint8_t)state);

  prefs.end();
}

void sendCrashLogData() {
  Preferences prefs;
  prefs.begin(CRASH_NAMESPACE, true);

  ResetEntry history[MAX_HISTORY] = {};
  size_t histLen = prefs.getBytesLength(KEY_HISTORY);
  int entryCount = 0;
  if (histLen > 0 && histLen <= sizeof(history)) {
    prefs.getBytes(KEY_HISTORY, history, histLen);
    entryCount = histLen / sizeof(ResetEntry);
  }

  uint16_t totalCount = prefs.getUShort(KEY_COUNT, 0);
  uint8_t rapidBoots  = prefs.getUChar(KEY_BOOTS, 0);
  uint8_t lastArmed   = prefs.getUChar(KEY_ARMED, 0);
  uint32_t lastUptime = prefs.getULong(KEY_UPTIME, 0);

  prefs.end();

  // Patch most recent entry with heartbeat data
  if (entryCount > 0) {
    history[0].armedState = lastArmed;
    history[0].uptimeMs = lastUptime;
  }

  // Build JSON manually (stack-safe for WebSerial task)
  DEBUG_SERIAL.print("{\"crash_log\":{\"total\":");
  DEBUG_SERIAL.print(totalCount);
  DEBUG_SERIAL.print(",\"rapid_boots\":");
  DEBUG_SERIAL.print(rapidBoots);
  DEBUG_SERIAL.print(",\"history\":[");

  for (int i = 0; i < entryCount; i++) {
    if (i > 0) DEBUG_SERIAL.print(",");
    ResetEntry& e = history[i];
    char buf[160];
    snprintf(buf, sizeof(buf),
      "{\"esp\":\"%s\",\"esp_code\":%d,\"rtc\":\"%s\",\"rtc_code\":%d,"
      "\"state\":\"%s\",\"uptime_ms\":%lu,\"fw\":\"%d.%d\"}",
      espReasonStr(e.espReason), e.espReason,
      rtcReasonStr(e.rtcReason), e.rtcReason,
      stateToString(e.armedState),
      (unsigned long)e.uptimeMs,
      e.verMajor / 10, e.verMajor % 10);
    DEBUG_SERIAL.print(buf);
  }

  DEBUG_SERIAL.println("]}}");
}
