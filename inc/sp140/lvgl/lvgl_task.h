#pragma once

#include <Arduino.h>

// LVGL task handler function
void lvglTask(void* parameter);

// Handle to the LVGL task
extern TaskHandle_t lvglTaskHandle;
