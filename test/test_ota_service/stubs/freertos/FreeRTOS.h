#pragma once
#include <cstdint>
using TickType_t = uint32_t;
constexpr int pdTRUE = 1;
constexpr int pdFALSE = 0;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
#define pdMS_TO_TICKS(x) (x)
