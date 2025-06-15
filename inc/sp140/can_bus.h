#pragma once

#include <Arduino.h>
#include "driver/twai.h"
#include "sp140/structs.h"

// Unified CAN bus management
void initUnifiedCANBus();
void unifiedCANBusTask(void* parameter);
void processCANMessage(const twai_message_t& message);

// External references to ESC functions
extern void processESCMessage(const twai_message_t& message);
extern void processESCTasks();

// Task handle
extern TaskHandle_t canBusTaskHandle;
