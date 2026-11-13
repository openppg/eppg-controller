#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "nvs.h"

namespace platform {
// Typed access deliberately keeps the field types used by installed firmware:
// booleans/u8, armed minutes/u16, timezone/i32, floats/raw records as blobs.
class NvsStore {
 public:
  NvsStore() = default;
  ~NvsStore() { close(); }
  NvsStore(const NvsStore&) = delete;
  NvsStore& operator=(const NvsStore&) = delete;
  bool open(const char* name, bool readonly = false) {
    close();
    return nvs_open(name, readonly ? NVS_READONLY : NVS_READWRITE, &handle_) == ESP_OK;
  }
  void close() {
    if (handle_) {
      nvs_close(handle_);
      handle_ = 0;
    }
  }
  bool contains(const char* key) const {
    nvs_type_t type;
    return handle_ && nvs_find_key(handle_, key, &type) == ESP_OK;
  }
  template <class T>
  T get(const char* key, T fallback = {}) const {
    T value = fallback;
    esp_err_t err;
    if constexpr (std::is_same_v<T, uint8_t>)
      err = nvs_get_u8(handle_, key, &value);
    else if constexpr (std::is_same_v<T, uint16_t>)
      err = nvs_get_u16(handle_, key, &value);
    else if constexpr (std::is_same_v<T, int32_t>)
      err = nvs_get_i32(handle_, key, &value);
    else if constexpr (std::is_same_v<T, bool>) {
      uint8_t byte = fallback ? 1 : 0;
      err = nvs_get_u8(handle_, key, &byte);
      value = byte != 0;
    } else {
      size_t length = sizeof(value);
      err = nvs_get_blob(handle_, key, &value, &length);
      if (length != sizeof(value)) err = ESP_ERR_INVALID_SIZE;
    }
    return err == ESP_OK ? value : fallback;
  }
  template <class T>
  bool set(const char* key, T value) {
    esp_err_t err;
    if constexpr (std::is_same_v<T, uint8_t>)
      err = nvs_set_u8(handle_, key, value);
    else if constexpr (std::is_same_v<T, uint16_t>)
      err = nvs_set_u16(handle_, key, value);
    else if constexpr (std::is_same_v<T, int32_t>)
      err = nvs_set_i32(handle_, key, value);
    else
      err = nvs_set_blob(handle_, key, &value, sizeof(value));
    return err == ESP_OK && nvs_commit(handle_) == ESP_OK;
  }
  size_t blobSize(const char* key) const {
    size_t length = 0;
    return nvs_get_blob(handle_, key, nullptr, &length) == ESP_OK ? length : 0;
  }
  size_t readBlob(const char* key, void* value, size_t capacity) const {
    size_t length = capacity;
    return nvs_get_blob(handle_, key, value, &length) == ESP_OK ? length : 0;
  }
  size_t writeBlob(const char* key, const void* value, size_t length) {
    return nvs_set_blob(handle_, key, value, length) == ESP_OK && nvs_commit(handle_) == ESP_OK ? length : 0;
  }
  void erase(const char* key) {
    if (nvs_erase_key(handle_, key) == ESP_OK) nvs_commit(handle_);
  }
  void eraseAll() {
    if (nvs_erase_all(handle_) == ESP_OK) nvs_commit(handle_);
  }

 private:
  nvs_handle_t handle_ = 0;
};
}  // namespace platform
