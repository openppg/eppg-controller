#pragma once
#include <cstdint>
#include <cstddef>
inline uint32_t bootReceipt = 0;
class Preferences {
 public:
  bool begin(const char*, bool) { return true; }
  size_t putUInt(const char*, uint32_t value) { bootReceipt = value; return sizeof(value); }
  uint32_t getUInt(const char*, uint32_t fallback) { return bootReceipt ? bootReceipt : fallback; }
  bool remove(const char*) { bootReceipt = 0; return true; }
  void end() {}
};
