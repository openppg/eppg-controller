#pragma once
#include <vector>
#include <cstring>
#include "FreeRTOS.h"
struct FakeQueue { std::vector<unsigned char> value; bool populated = false; };
using QueueHandle_t = FakeQueue*;
inline QueueHandle_t xQueueCreate(unsigned, unsigned size) {
  auto* queue = new FakeQueue;
  queue->value.resize(size);
  return queue;
}
inline int xQueueOverwrite(QueueHandle_t queue, const void* value) {
  memcpy(queue->value.data(), value, queue->value.size());
  queue->populated = true;
  return pdTRUE;
}
inline int xQueuePeek(QueueHandle_t queue, void* value, TickType_t) {
  if (!queue->populated) return pdFALSE;
  memcpy(value, queue->value.data(), queue->value.size());
  return pdTRUE;
}
