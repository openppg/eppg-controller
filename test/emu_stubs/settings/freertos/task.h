#pragma once

#include "FreeRTOS.h"

inline void vTaskDelay(TickType_t ticks) { emuAdvanceMillis(ticks); }
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return emuCurrentTask(); }
inline TickType_t xTaskGetTickCount() { return emuMillisRef(); }
