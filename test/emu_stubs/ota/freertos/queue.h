// Fake FreeRTOS queue: just enough for the one-slot update-screen status queue
// (xQueueCreate / xQueueOverwrite / xQueuePeek). Never blocks.
#pragma once

#include <cstring>
#include <vector>

#include "freertos/FreeRTOS.h"

struct EmuQueue {
  std::vector<unsigned char> item;
  bool full = false;
};
typedef EmuQueue* QueueHandle_t;

inline QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t itemSize) {
  (void)length;
  QueueHandle_t q = new EmuQueue;
  q->item.resize(itemSize);
  return q;
}

inline BaseType_t xQueueOverwrite(QueueHandle_t q, const void* item) {
  if (q == nullptr) return pdFALSE;
  memcpy(q->item.data(), item, q->item.size());
  q->full = true;
  return pdPASS;
}

inline BaseType_t xQueuePeek(QueueHandle_t q, void* out, TickType_t timeout) {
  (void)timeout;
  if (q == nullptr || !q->full) return pdFALSE;
  memcpy(out, q->item.data(), q->item.size());
  return pdTRUE;
}
