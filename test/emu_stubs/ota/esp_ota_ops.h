// Fake ESP-IDF OTA API. Return codes and handle lifetime follow ESP-IDF 4.4
// (espressif32@6.13.0): handles start at 1, esp_ota_end() releases the handle
// whether or not it succeeds, esp_ota_end() refuses an image with no data, and
// esp_ota_abort() on a released handle returns ESP_ERR_NOT_FOUND.
#pragma once

#include <cstddef>
#include <cstdint>

#include "emu_ota_harness.h"
#include "esp_err.h"
#include "esp_partition.h"

typedef uint32_t esp_ota_handle_t;

#define OTA_SIZE_UNKNOWN 0xffffffff
#define OTA_WITH_SEQUENTIAL_WRITES 0xfffffffe

namespace emu {
inline esp_partition_t updatePartition = {
    nullptr, ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1,
    0x340000, 0x330000, 0x1000, "app1", false};
}  // namespace emu

inline const esp_partition_t* esp_ota_get_next_update_partition(const esp_partition_t* start_from) {
  (void)start_from;
  emu::recordEvent("ota_get_next_update_partition");
  if (emu::faults.noUpdatePartition) return nullptr;
  emu::updatePartition.size = emu::faults.partitionSize;
  return &emu::updatePartition;
}

inline esp_err_t esp_ota_begin(const esp_partition_t* partition, size_t image_size,
                               esp_ota_handle_t* out_handle) {
  emu::checkLockedFlashCall();
  emu::recordEvent("ota_begin");
  ++emu::beginCalls;
  if (partition == nullptr || out_handle == nullptr) return ESP_ERR_INVALID_ARG;
  if (image_size != OTA_SIZE_UNKNOWN && image_size != OTA_WITH_SEQUENTIAL_WRITES &&
      image_size > partition->size) {
    return ESP_ERR_INVALID_SIZE;
  }
  emu::advanceMs(emu::faults.eraseMs);
  if (emu::onBegin) emu::onBegin();
  if (emu::faults.beginErr != ESP_OK) return emu::faults.beginErr;
  const esp_ota_handle_t h = emu::nextHandle++;
  emu::liveHandles[h] = 0;
  emu::beginImageSize = static_cast<uint32_t>(image_size);
  emu::flashImage.clear();
  *out_handle = h;
  return ESP_OK;
}

inline esp_err_t esp_ota_write(esp_ota_handle_t handle, const void* data, size_t size) {
  emu::checkLockedFlashCall();
  emu::recordEvent("ota_write");
  ++emu::writeCalls;
  if (emu::onWrite) emu::onWrite();
  if (emu::faults.failWriteOnCall == emu::writeCalls) return emu::faults.writeErr;
  auto it = emu::liveHandles.find(handle);
  if (it == emu::liveHandles.end()) return ESP_ERR_INVALID_ARG;
  if (it->second + size > emu::updatePartition.size) return ESP_ERR_INVALID_SIZE;
  const uint8_t* p = static_cast<const uint8_t*>(data);
  emu::flashImage.insert(emu::flashImage.end(), p, p + size);
  it->second += size;
  return ESP_OK;
}

inline esp_err_t esp_ota_end(esp_ota_handle_t handle) {
  emu::checkLockedFlashCall();
  emu::recordEvent("ota_end");
  ++emu::endCalls;
  auto it = emu::liveHandles.find(handle);
  if (it == emu::liveHandles.end()) return ESP_ERR_NOT_FOUND;
  const size_t written = it->second;
  emu::liveHandles.erase(it);
  if (written == 0) return ESP_ERR_INVALID_ARG;
  return emu::faults.endErr;
}

inline esp_err_t esp_ota_abort(esp_ota_handle_t handle) {
  emu::checkLockedFlashCall();
  emu::recordEvent("ota_abort");
  ++emu::abortCalls;
  auto it = emu::liveHandles.find(handle);
  if (it == emu::liveHandles.end()) {
    ++emu::abortNotFound;
    return ESP_ERR_NOT_FOUND;
  }
  emu::liveHandles.erase(it);
  return ESP_OK;
}

inline esp_err_t esp_ota_set_boot_partition(const esp_partition_t* partition) {
  emu::checkLockedFlashCall();
  emu::recordEvent("set_boot_partition");
  ++emu::setBootCalls;
  if (partition == nullptr) return ESP_ERR_INVALID_ARG;
  if (emu::faults.setBootErr != ESP_OK) return emu::faults.setBootErr;
  emu::bootPartition = partition;
  return ESP_OK;
}
