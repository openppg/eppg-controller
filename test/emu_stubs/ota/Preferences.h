// Arduino Preferences fake for the OTA emulation: the update-screen boot
// receipt only (ota_service.cpp's "ota-ui" namespace). Values live in
// emu::nvsU32 and every write is recorded in emu::events, so tests can check
// which task context (callback vs checkOtaTimeout) wrote NVS.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

#include "emu_ota_harness.h"

namespace emu {
inline std::map<std::string, uint32_t> nvsU32;  // "namespace/key" -> value
}  // namespace emu

class Preferences {
 public:
  bool begin(const char* name, bool readOnly = false) {
    ns_ = name ? name : "";
    readOnly_ = readOnly;
    return true;
  }
  void end() {}
  size_t putUInt(const char* key, uint32_t value) {
    if (readOnly_) return 0;
    emu::nvsU32[ns_ + "/" + key] = value;
    emu::recordEvent("nvs_put " + ns_ + "/" + key);
    return sizeof(value);
  }
  uint32_t getUInt(const char* key, uint32_t defaultValue = 0) {
    auto it = emu::nvsU32.find(ns_ + "/" + key);
    return it == emu::nvsU32.end() ? defaultValue : it->second;
  }
  bool remove(const char* key) {
    if (readOnly_) return false;
    emu::recordEvent("nvs_remove " + ns_ + "/" + key);
    return emu::nvsU32.erase(ns_ + "/" + key) > 0;
  }

 private:
  std::string ns_;
  bool readOnly_ = false;
};
