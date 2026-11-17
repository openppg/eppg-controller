#pragma once

#include <cstdint>

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS pdTRUE
#define pdFAIL pdFALSE
#define portMAX_DELAY static_cast<TickType_t>(0xffffffffUL)
#define pdMS_TO_TICKS(ms) static_cast<TickType_t>(ms)
