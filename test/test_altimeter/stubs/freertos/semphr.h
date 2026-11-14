#pragma once
#include <mutex>
#include <chrono>
using SemaphoreHandle_t = std::timed_mutex*;
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
#define pdMS_TO_TICKS(ms) (ms)
#define pdTRUE 1
inline auto xSemaphoreCreateMutex() { return new std::timed_mutex; }
inline int xSemaphoreTake(SemaphoreHandle_t m, unsigned ms) {
  return m->try_lock_for(std::chrono::milliseconds(ms));
}
inline void xSemaphoreGive(SemaphoreHandle_t m) { m->unlock(); }
