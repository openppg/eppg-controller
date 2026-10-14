// Virtual millisecond clock shared by millis()/delay() and vTaskDelay(), so
// firmware loops with long timeouts run instantly and deterministically.
// CONFIG_FREERTOS_HZ=1000 on the SP140, so one tick is one millisecond.
#pragma once

#include <stdint.h>

inline uint32_t& emuMillisRef() {
  static uint32_t ms = 0;
  return ms;
}

inline void emuAdvanceMillis(uint32_t ms) { emuMillisRef() += ms; }
inline void emuSetMillis(uint32_t ms) { emuMillisRef() = ms; }
