// Fake FreeRTOS mutex. The emulation is single-threaded, so "another task holds
// the lock" is modelled by calling into the firmware from inside a fake that
// runs while the lock is held (see emu::onBegin / emu::onWrite).
#pragma once

#include <memory>

#include "emu_ota_harness.h"
#include "freertos/FreeRTOS.h"

typedef emu::FakeMutex* SemaphoreHandle_t;

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
  emu::mutexes.push_back(std::make_unique<emu::FakeMutex>());
  return emu::mutexes.back().get();
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t m, TickType_t timeout) {
  if (m == nullptr) return pdFALSE;
  if (m->held) {
    if (timeout == 0) {
      ++m->busyTryTakes;
    } else {
      ++emu::deadlocks;  // a blocking take on a held, non-recursive mutex
    }
    return pdFALSE;
  }
  m->held = true;
  ++m->takes;
  return pdTRUE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t m) {
  if (m == nullptr || !m->held) return pdFALSE;
  m->held = false;
  ++m->gives;
  return pdTRUE;
}
