#pragma once

#include <stdint.h>

// FreeRTOS type stubs for native builds
typedef void* SemaphoreHandle_t;
typedef void* QueueHandle_t;
typedef void* TaskHandle_t;
typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS pdTRUE
#define portMAX_DELAY 0xFFFFFFFF

inline TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }

// Semaphore stubs
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return (void*)1; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }

// Task stubs
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelete(TaskHandle_t) {}

// Queue stubs
inline QueueHandle_t xQueueCreate(UBaseType_t, UBaseType_t) { return (void*)1; }
inline BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t) { return pdTRUE; }
inline BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t) { return pdFALSE; }
inline BaseType_t xQueueOverwrite(QueueHandle_t, const void*) { return pdTRUE; }
inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t) { return 0; }
inline void xQueueReset(QueueHandle_t) {}
