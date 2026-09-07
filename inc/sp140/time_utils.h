#ifndef INC_SP140_TIME_UTILS_H_
#define INC_SP140_TIME_UTILS_H_

#include <stdint.h>

// Small Arduino-millis/delay off-ramp. Semantics match Arduino-ESP32 2.0.17
// (espressif32@6.13.0):
//   timeMillis() == esp_timer_get_time() / 1000ULL  (32-bit wrap, ms since boot)
//   timeDelay()  == vTaskDelay(pdMS_TO_TICKS(ms))
// On this firmware CONFIG_FREERTOS_HZ=1000, pdMS_TO_TICKS(ms) == ms and this
// matches Arduino delay() which calls vTaskDelay(ms / portTICK_PERIOD_MS).

#if defined(ESP_PLATFORM)

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

inline uint32_t timeMillis() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

inline void timeDelay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

#else

// Native unit / screenshot tests: share the Arduino stub clock so timestamps
// passed as millis() from tests stay coherent with production timeMillis().
#include <Arduino.h>

inline uint32_t timeMillis() {
  return static_cast<uint32_t>(millis());
}

inline void timeDelay(uint32_t ms) {
  delay(ms);
}

#endif

#endif  // INC_SP140_TIME_UTILS_H_
