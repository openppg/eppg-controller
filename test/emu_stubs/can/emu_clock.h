// Virtual clock shared by the Arduino and FreeRTOS stubs of the CAN emulation
// tests. Nothing advances it except the test.
#pragma once

#include <stdint.h>

inline uint64_t& emuMicrosRef() {
  static uint64_t us = 0;
  return us;
}

inline void emuAdvanceMicros(uint64_t us) { emuMicrosRef() += us; }
inline void emuAdvanceMillis(uint32_t ms) { emuMicrosRef() += 1000ULL * ms; }
