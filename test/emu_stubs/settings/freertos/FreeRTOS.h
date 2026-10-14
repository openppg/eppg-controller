// Host stub of the FreeRTOS pieces the settings/QC code uses. Single "task";
// mutexes are tracked so tests can assert every take is paired with a give
// and that no code path re-takes a non-recursive mutex it already holds
// (which would deadlock on the real target).
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "../emu_clock.h"

typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;
typedef void* TaskHandle_t;
typedef void* QueueHandle_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS pdTRUE
#define pdFAIL pdFALSE
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define portTICK_PERIOD_MS ((TickType_t)1)
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(xTimeInMs) ((TickType_t)(((TickType_t)(xTimeInMs) * configTICK_RATE_HZ) / 1000U))

struct EmuMutex {
  bool held;
};

typedef EmuMutex* SemaphoreHandle_t;

struct EmuRtosStats {
  int mutexesCreated = 0;
  int takes = 0;
  int gives = 0;
  int selfDeadlocks = 0;    // take of a mutex this task already holds
  int unbalancedGives = 0;  // give of a mutex nobody holds
};

inline EmuRtosStats& emuRtosStats() {
  static EmuRtosStats s;
  return s;
}

inline TaskHandle_t emuCurrentTask() { return reinterpret_cast<TaskHandle_t>(0x1); }
