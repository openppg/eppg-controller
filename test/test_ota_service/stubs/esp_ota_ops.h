#pragma once
#include <cstddef>
#include <functional>
#include "esp_partition.h"
using esp_err_t = int;
using esp_ota_handle_t = unsigned;
constexpr esp_err_t ESP_OK = 0;
inline esp_partition_t targetPartition{8192, 0x200000};
inline esp_partition_t runningPartition{8192, 0x10000};
inline int beginResult = 0, writeResult = 0, endResult = 0, bootResult = 0;
inline unsigned writes = 0, aborts = 0;
inline bool partitionAvailable = true;
inline std::function<void()> onBegin, onWrite, onEnd;
inline const esp_partition_t* esp_ota_get_next_update_partition(const void*) {
  return partitionAvailable ? &targetPartition : nullptr;
}
inline const esp_partition_t* esp_ota_get_running_partition() { return &runningPartition; }
inline esp_err_t esp_ota_begin(const esp_partition_t*, size_t, esp_ota_handle_t* handle) {
  if (onBegin) onBegin();
  if (!beginResult) *handle = 1;
  return beginResult;
}
inline esp_err_t esp_ota_write(esp_ota_handle_t, const void*, size_t) {
  ++writes;
  if (onWrite) onWrite();
  return writeResult;
}
inline esp_err_t esp_ota_end(esp_ota_handle_t) { if (onEnd) onEnd(); return endResult; }
inline esp_err_t esp_ota_set_boot_partition(const esp_partition_t*) { return bootResult; }
inline esp_err_t esp_ota_abort(esp_ota_handle_t) { ++aborts; return 0; }
