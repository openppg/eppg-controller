// The FreeRTOS names esc.cpp and the TWAI driver header use. Deliberately does
// not define FREERTOS_H: BMS_CAN only takes its mutex when that is defined.
#pragma once

#include <stdint.h>

#include "emu_clock.h"

typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))  // CONFIG_FREERTOS_HZ=1000
#define portMAX_DELAY ((TickType_t)0xffffffffUL)

inline void vTaskDelay(TickType_t ticks) { emuAdvanceMillis(ticks); }
