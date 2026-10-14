#pragma once

#include "FreeRTOS.h"

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
  emuRtosStats().mutexesCreated++;
  return new EmuMutex{false};
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t m, TickType_t ticks) {
  (void)ticks;
  if (m == nullptr) return pdFALSE;
  emuRtosStats().takes++;
  if (m->held) {
    // Same task taking a held non-recursive mutex: blocks forever on target.
    emuRtosStats().selfDeadlocks++;
    return pdFALSE;
  }
  m->held = true;
  return pdTRUE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t m) {
  if (m == nullptr) return pdFALSE;
  emuRtosStats().gives++;
  if (!m->held) {
    emuRtosStats().unbalancedGives++;
    return pdFALSE;
  }
  m->held = false;
  return pdTRUE;
}

inline TaskHandle_t xSemaphoreGetMutexHolder(SemaphoreHandle_t m) {
  return (m != nullptr && m->held) ? emuCurrentTask() : nullptr;
}

inline void vSemaphoreDelete(SemaphoreHandle_t m) { delete m; }
