// Host stub of the ESP-IDF 4.4 NVS C API (components/nvs_flash/include/nvs.h)
// backed by the in-memory model in emu_nvs.h. Semantics are documented there.
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "emu_nvs.h"

#define NVS_DEFAULT_PART_NAME "nvs"
#define NVS_PART_NAME_MAX_SIZE 16
#define NVS_KEY_NAME_MAX_SIZE 16

typedef uint32_t nvs_handle_t;

typedef enum {
  NVS_READONLY,
  NVS_READWRITE,
} nvs_open_mode_t;

typedef struct {
  size_t used_entries;
  size_t free_entries;
  size_t total_entries;
  size_t namespace_count;
} nvs_stats_t;

inline esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle) {
  return emu_nvs::open(name, open_mode == NVS_READONLY, out_handle);
}

inline esp_err_t nvs_open_from_partition(const char* part_name, const char* name,
                                         nvs_open_mode_t open_mode, nvs_handle_t* out_handle) {
  if (part_name == nullptr || strcmp(part_name, NVS_DEFAULT_PART_NAME) != 0) {
    return ESP_ERR_NVS_PART_NOT_FOUND;
  }
  return nvs_open(name, open_mode, out_handle);
}

inline void nvs_close(nvs_handle_t handle) { emu_nvs::close(handle); }
inline esp_err_t nvs_commit(nvs_handle_t handle) { return emu_nvs::commit(handle); }
inline esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key) {
  return emu_nvs::eraseKey(handle, key);
}
inline esp_err_t nvs_erase_all(nvs_handle_t handle) { return emu_nvs::eraseAll(handle); }

#define EMU_NVS_TYPED(suffix, ctype, etype)                                          \
  inline esp_err_t nvs_set_##suffix(nvs_handle_t handle, const char* key, ctype value) { \
    return emu_nvs::set(handle, key, emu_nvs::Type::etype, &value, sizeof(value));   \
  }                                                                                  \
  inline esp_err_t nvs_get_##suffix(nvs_handle_t handle, const char* key, ctype* out) { \
    return emu_nvs::getFixed(handle, key, emu_nvs::Type::etype, out, sizeof(ctype)); \
  }

EMU_NVS_TYPED(i8, int8_t, I8)
EMU_NVS_TYPED(u8, uint8_t, U8)
EMU_NVS_TYPED(i16, int16_t, I16)
EMU_NVS_TYPED(u16, uint16_t, U16)
EMU_NVS_TYPED(i32, int32_t, I32)
EMU_NVS_TYPED(u32, uint32_t, U32)
EMU_NVS_TYPED(i64, int64_t, I64)
EMU_NVS_TYPED(u64, uint64_t, U64)

#undef EMU_NVS_TYPED

inline esp_err_t nvs_set_str(nvs_handle_t handle, const char* key, const char* value) {
  if (value == nullptr) return ESP_ERR_INVALID_ARG;
  return emu_nvs::set(handle, key, emu_nvs::Type::STR, value, strlen(value) + 1);
}

inline esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value,
                              size_t length) {
  if (value == nullptr && length > 0) return ESP_ERR_INVALID_ARG;
  return emu_nvs::set(handle, key, emu_nvs::Type::BLOB, value, length);
}

inline esp_err_t nvs_get_str(nvs_handle_t handle, const char* key, char* out_value,
                             size_t* length) {
  return emu_nvs::getVar(handle, key, emu_nvs::Type::STR, out_value, length);
}

inline esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value,
                              size_t* length) {
  return emu_nvs::getVar(handle, key, emu_nvs::Type::BLOB, out_value, length);
}

inline esp_err_t nvs_get_stats(const char* part_name, nvs_stats_t* nvs_stats) {
  (void)part_name;
  if (nvs_stats == nullptr) return ESP_ERR_INVALID_ARG;
  const size_t total = 126 * 4;  // 4 usable 4 KiB pages
  const size_t used = emu_nvs::state().entries.size() + emu_nvs::state().namespaces.size();
  nvs_stats->used_entries = used;
  nvs_stats->free_entries = used < total ? total - used : 0;
  nvs_stats->total_entries = total;
  nvs_stats->namespace_count = emu_nvs::state().namespaces.size();
  return ESP_OK;
}
