#pragma once
#include "FreeRTOS.h"
// Single-threaded fake refuses re-entry, catching accidental nested acquisition.
struct FakeMutex { bool held = false; };
using SemaphoreHandle_t = FakeMutex*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new FakeMutex; }
inline int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t) {
  if (!mutex || mutex->held) return pdFALSE;
  mutex->held = true;
  return pdTRUE;
}
inline void xSemaphoreGive(SemaphoreHandle_t mutex) { mutex->held = false; }
