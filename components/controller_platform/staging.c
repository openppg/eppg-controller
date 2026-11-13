#include "platform/staging.h"
#include "image_staging.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static eppg_image_store store;
static StaticSemaphore_t lock_storage;
static SemaphoreHandle_t lock;
static const esp_partition_t* partition;

static int read_partition(void* context, uint32_t offset, void* data, size_t length) {
  return esp_partition_read((const esp_partition_t*)context, offset, data, length);
}
static int write_partition(void* context, uint32_t offset, const void* data, size_t length) {
  return esp_partition_write((const esp_partition_t*)context, offset, data, length);
}
static int erase_partition(void* context, uint32_t offset, size_t length) {
  return esp_partition_erase_range((const esp_partition_t*)context, offset, length);
}
int eppg_stage_begin(uint32_t size) {
  // Initialization is performed by the first accepted disarmed maintenance
  // request. The ESC relay serializes begin with its own session guard.
  if (!lock) lock = xSemaphoreCreateMutexStatic(&lock_storage);
  if (!lock || xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) return -1;
  if (!partition) {
    partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "spiffs");
    if (partition) {
      const eppg_stage_io io = {(void*)partition, partition->size, read_partition, write_partition, erase_partition};
      eppg_image_store_init(&store, &io);
    }
  }
  const bool ok = partition && eppg_image_store_begin(&store, size);
  xSemaphoreGive(lock);
  if (!ok) ESP_LOGE("esc_stage", "Cannot claim unused/owned staging partition or erase image area");
  return ok ? 0 : -1;
}
int32_t eppg_stage_write(uint32_t offset, const uint8_t* bytes, size_t length) {
  if (!lock || xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) return -1;
  const int32_t result = eppg_image_store_write(&store, offset, bytes, length);
  xSemaphoreGive(lock);
  return result;
}
int eppg_stage_read(uint32_t offset, uint8_t* bytes, size_t length) {
  if (!lock || xSemaphoreTake(lock, 0) != pdTRUE) return -2;
  const bool result = eppg_image_store_read(&store, offset, bytes, length);
  xSemaphoreGive(lock);
  return result ? 0 : -1;
}
int eppg_stage_finish(void) {
  if (!lock || xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) return -1;
  const bool result = eppg_image_store_finish(&store);
  xSemaphoreGive(lock);
  return result ? 0 : -1;
}
int eppg_stage_abort(void) {
  if (!lock) return 0;
  if (xSemaphoreTake(lock, 0) != pdTRUE) return -2;
  eppg_image_store_abort(&store);
  xSemaphoreGive(lock);
  return 0;
}
