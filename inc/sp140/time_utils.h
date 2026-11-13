#ifndef INC_SP140_TIME_UTILS_H_
#define INC_SP140_TIME_UTILS_H_
#include <stdint.h>
#include "platform/runtime.h"
// Compatibility names retained from master; all targets share the native clock.
inline unsigned long timeMillis() { return platform::millis32(); }
inline void timeDelay(uint32_t ms) {
#ifdef ESP_PLATFORM
  vTaskDelay(pdMS_TO_TICKS(ms));
#else
  platform::delayMs(ms);
#endif
}
#endif
