// Shared state for the host emulation of the BLE OTA service.
//
// The stub headers in this directory (Arduino.h, esp_ota_ops.h, FreeRTOS,
// NimBLE) replace the ESP-IDF / Arduino / NimBLE include boundary so the real
// src/sp140/ble/ota_service.cpp compiles unmodified on the host. Every fake
// records what the firmware did in emu::events, in call order, and reads its
// fault-injection settings from emu::faults.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "esp_err.h"

namespace emu {

// Virtual clock behind millis() / timeMillis(). Only tests advance it.
inline unsigned long nowMs = 0;
inline void advanceMs(unsigned long ms) { nowMs += ms; }

// Ordered log of side effects, e.g. "conn_fast", "ota_begin", "notify 8022".
inline std::vector<std::string> events;
inline void recordEvent(const std::string& e) { events.push_back(e); }

// USBSerial output, one entry per print/printf call.
inline std::vector<std::string> serialLog;

inline int restartCount = 0;
inline unsigned long restartAtMs = 0;

// Fault injection for the esp_ota_* fakes.
struct OtaFaults {
  bool noUpdatePartition = false;
  uint32_t partitionSize = 0x330000;  // app0/app1 size in default_8MB.csv
  esp_err_t beginErr = ESP_OK;
  int failWriteOnCall = 0;  // 1-based esp_ota_write call to fail; 0 = never
  esp_err_t writeErr = ESP_FAIL;
  esp_err_t endErr = ESP_OK;
  esp_err_t setBootErr = ESP_OK;
  unsigned long eraseMs = 0;  // virtual time esp_ota_begin() spends erasing
};
inline OtaFaults faults;

// Hooks run inside the fakes, i.e. while the firmware is "inside" the call.
inline std::function<void()> onBegin;
inline std::function<void()> onWrite;

// Flash-side record.
inline uint32_t nextHandle = 1;  // ESP-IDF handles start at 1, never 0
inline std::map<uint32_t, size_t> liveHandles;  // handle -> bytes written
inline uint32_t beginImageSize = 0;
inline int beginCalls = 0;
inline int writeCalls = 0;
inline int endCalls = 0;
inline int abortCalls = 0;
inline int abortNotFound = 0;  // esp_ota_abort() on a handle already released
inline int setBootCalls = 0;
inline std::vector<uint8_t> flashImage;  // bytes accepted by esp_ota_write
inline const void* bootPartition = nullptr;

// FreeRTOS mutex fake. Single-threaded emulation: a blocking take on a mutex
// that is already held would deadlock on the device, so it is counted.
struct FakeMutex {
  bool held = false;
  int takes = 0;
  int gives = 0;
  int busyTryTakes = 0;
  int timedOutTakes = 0;
};
inline std::vector<std::unique_ptr<FakeMutex>> mutexes;
inline int deadlocks = 0;
// esp_ota_* calls made while the OTA state mutex was not held.
inline int unlockedFlashCalls = 0;

inline bool otaLockHeld() { return !mutexes.empty() && mutexes.back()->held; }
inline void checkLockedFlashCall() {
  if (!otaLockHeld()) ++unlockedFlashCalls;
}

inline void reset(unsigned long startMs) {
  nowMs = startMs;
  events.clear();
  serialLog.clear();
  restartCount = 0;
  restartAtMs = 0;
  faults = OtaFaults();
  onBegin = nullptr;
  onWrite = nullptr;
  nextHandle = 1;
  liveHandles.clear();
  beginImageSize = 0;
  beginCalls = writeCalls = endCalls = abortCalls = abortNotFound = 0;
  setBootCalls = 0;
  flashImage.clear();
  bootPartition = nullptr;
  mutexes.clear();
  deadlocks = 0;
  unlockedFlashCalls = 0;
}

inline int indexOf(const std::string& e) {
  for (size_t i = 0; i < events.size(); ++i) {
    if (events[i] == e) return static_cast<int>(i);
  }
  return -1;
}

inline int lastIndexOf(const std::string& e) {
  for (size_t i = events.size(); i > 0; --i) {
    if (events[i - 1] == e) return static_cast<int>(i - 1);
  }
  return -1;
}

inline int countOf(const std::string& e) {
  int n = 0;
  for (const auto& x : events) n += (x == e) ? 1 : 0;
  return n;
}

}  // namespace emu
