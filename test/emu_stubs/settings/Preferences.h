// Host stub of Arduino-ESP32 2.0.17 Preferences (libraries/Preferences) on top
// of the fake NVS. Each method follows the 2.0.17 Preferences.cpp control
// flow: every put* is nvs_set_* + nvs_commit, getFloat/getDouble are 4/8-byte
// blobs via getBytes, getBool is getUChar(..) == 1, isKey probes every type,
// clear is nvs_erase_all + nvs_commit. This is how 7.x/8.0 stored settings.
#pragma once

#include <math.h>
#include <string.h>

#include "Arduino.h"
#include "nvs.h"
#include "nvs_flash.h"

typedef enum {
  PT_I8, PT_U8, PT_I16, PT_U16, PT_I32, PT_U32, PT_I64, PT_U64, PT_STR, PT_BLOB, PT_INVALID
} PreferenceType;

class Preferences {
 protected:
  uint32_t _handle;
  bool _started;
  bool _readOnly;

 public:
  Preferences() : _handle(0), _started(false), _readOnly(false) {}
  ~Preferences() { end(); }

  bool begin(const char* name, bool readOnly = false, const char* partition_label = NULL) {
    if (_started) {
      return false;
    }
    _readOnly = readOnly;
    esp_err_t err = ESP_OK;
    if (partition_label != NULL) {
      err = nvs_flash_init_partition(partition_label);
      if (err) {
        return false;
      }
      err = nvs_open_from_partition(partition_label, name,
                                    readOnly ? NVS_READONLY : NVS_READWRITE, &_handle);
    } else {
      err = nvs_open(name, readOnly ? NVS_READONLY : NVS_READWRITE, &_handle);
    }
    if (err) {
      log_e("nvs_open failed: %d", err);
      return false;
    }
    _started = true;
    return true;
  }

  void end() {
    if (!_started) {
      return;
    }
    nvs_close(_handle);
    _started = false;
  }

  bool clear() {
    if (!_started || _readOnly) {
      return false;
    }
    if (nvs_erase_all(_handle)) {
      return false;
    }
    if (nvs_commit(_handle)) {
      return false;
    }
    return true;
  }

  bool remove(const char* key) {
    if (!_started || !key || _readOnly) {
      return false;
    }
    if (nvs_erase_key(_handle, key)) {
      return false;
    }
    if (nvs_commit(_handle)) {
      return false;
    }
    return true;
  }

 private:
  template <typename T, typename SetFn>
  size_t putTyped(const char* key, T value, SetFn setFn, size_t okLen) {
    if (!_started || !key || _readOnly) {
      return 0;
    }
    if (setFn(_handle, key, value)) {
      return 0;
    }
    if (nvs_commit(_handle)) {
      return 0;
    }
    return okLen;
  }

  template <typename T, typename GetFn>
  T getTyped(const char* key, T defaultValue, GetFn getFn) {
    T value = defaultValue;
    if (!_started || !key) {
      return value;
    }
    getFn(_handle, key, &value);
    return value;
  }

 public:
  size_t putChar(const char* key, int8_t value) { return putTyped(key, value, nvs_set_i8, 1); }
  size_t putUChar(const char* key, uint8_t value) { return putTyped(key, value, nvs_set_u8, 1); }
  size_t putShort(const char* key, int16_t value) { return putTyped(key, value, nvs_set_i16, 2); }
  size_t putUShort(const char* key, uint16_t value) { return putTyped(key, value, nvs_set_u16, 2); }
  size_t putInt(const char* key, int32_t value) { return putTyped(key, value, nvs_set_i32, 4); }
  size_t putUInt(const char* key, uint32_t value) { return putTyped(key, value, nvs_set_u32, 4); }
  size_t putLong(const char* key, int32_t value) { return putInt(key, value); }
  size_t putULong(const char* key, uint32_t value) { return putUInt(key, value); }
  size_t putLong64(const char* key, int64_t value) { return putTyped(key, value, nvs_set_i64, 8); }
  size_t putULong64(const char* key, uint64_t value) { return putTyped(key, value, nvs_set_u64, 8); }
  size_t putFloat(const char* key, const float_t value) {
    return putBytes(key, (void*)&value, sizeof(float_t));
  }
  size_t putDouble(const char* key, const double_t value) {
    return putBytes(key, (void*)&value, sizeof(double_t));
  }
  size_t putBool(const char* key, const bool value) {
    return putUChar(key, (uint8_t)(value ? 1 : 0));
  }

  size_t putString(const char* key, const char* value) {
    if (!_started || !key || !value || _readOnly) {
      return 0;
    }
    if (nvs_set_str(_handle, key, value)) {
      return 0;
    }
    if (nvs_commit(_handle)) {
      return 0;
    }
    return strlen(value);
  }
  size_t putString(const char* key, const String value) { return putString(key, value.c_str()); }

  size_t putBytes(const char* key, const void* value, size_t len) {
    if (!_started || !key || !value || !len || _readOnly) {
      return 0;
    }
    if (nvs_set_blob(_handle, key, value, len)) {
      return 0;
    }
    if (nvs_commit(_handle)) {
      return 0;
    }
    return len;
  }

  PreferenceType getType(const char* key) {
    if (!_started || !key || strlen(key) > 15) {
      return PT_INVALID;
    }
    int8_t mt1; uint8_t mt2; int16_t mt3; uint16_t mt4;
    int32_t mt5; uint32_t mt6; int64_t mt7; uint64_t mt8;
    size_t len = 0;
    if (nvs_get_i8(_handle, key, &mt1) == ESP_OK) return PT_I8;
    if (nvs_get_u8(_handle, key, &mt2) == ESP_OK) return PT_U8;
    if (nvs_get_i16(_handle, key, &mt3) == ESP_OK) return PT_I16;
    if (nvs_get_u16(_handle, key, &mt4) == ESP_OK) return PT_U16;
    if (nvs_get_i32(_handle, key, &mt5) == ESP_OK) return PT_I32;
    if (nvs_get_u32(_handle, key, &mt6) == ESP_OK) return PT_U32;
    if (nvs_get_i64(_handle, key, &mt7) == ESP_OK) return PT_I64;
    if (nvs_get_u64(_handle, key, &mt8) == ESP_OK) return PT_U64;
    if (nvs_get_str(_handle, key, NULL, &len) == ESP_OK) return PT_STR;
    if (nvs_get_blob(_handle, key, NULL, &len) == ESP_OK) return PT_BLOB;
    return PT_INVALID;
  }

  bool isKey(const char* key) { return getType(key) != PT_INVALID; }

  int8_t getChar(const char* key, const int8_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_i8);
  }
  uint8_t getUChar(const char* key, const uint8_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_u8);
  }
  int16_t getShort(const char* key, const int16_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_i16);
  }
  uint16_t getUShort(const char* key, const uint16_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_u16);
  }
  int32_t getInt(const char* key, const int32_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_i32);
  }
  uint32_t getUInt(const char* key, const uint32_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_u32);
  }
  int32_t getLong(const char* key, const int32_t defaultValue = 0) {
    return getInt(key, defaultValue);
  }
  uint32_t getULong(const char* key, const uint32_t defaultValue = 0) {
    return getUInt(key, defaultValue);
  }
  int64_t getLong64(const char* key, const int64_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_i64);
  }
  uint64_t getULong64(const char* key, const uint64_t defaultValue = 0) {
    return getTyped(key, defaultValue, nvs_get_u64);
  }

  float_t getFloat(const char* key, const float_t defaultValue = NAN) {
    float_t value = defaultValue;
    getBytes(key, (void*)&value, sizeof(float_t));
    return value;
  }
  double_t getDouble(const char* key, const double_t defaultValue = NAN) {
    double_t value = defaultValue;
    getBytes(key, (void*)&value, sizeof(double_t));
    return value;
  }
  bool getBool(const char* key, const bool defaultValue = false) {
    return getUChar(key, defaultValue ? 1 : 0) == 1;
  }

  size_t getString(const char* key, char* value, const size_t maxLen) {
    size_t len = 0;
    if (!_started || !key || !value || !maxLen) {
      return 0;
    }
    if (nvs_get_str(_handle, key, NULL, &len)) {
      return 0;
    }
    if (len > maxLen) {
      return 0;
    }
    if (nvs_get_str(_handle, key, value, &len)) {
      return 0;
    }
    return len;
  }

  String getString(const char* key, const String defaultValue = String()) {
    size_t len = 0;
    if (!_started || !key) {
      return String(defaultValue);
    }
    if (nvs_get_str(_handle, key, NULL, &len)) {
      return String(defaultValue);
    }
    std::vector<char> buf(len);
    if (nvs_get_str(_handle, key, buf.data(), &len)) {
      return String(defaultValue);
    }
    return String(buf.data());
  }

  size_t getBytesLength(const char* key) {
    size_t len = 0;
    if (!_started || !key) {
      return 0;
    }
    if (nvs_get_blob(_handle, key, NULL, &len)) {
      return 0;
    }
    return len;
  }

  size_t getBytes(const char* key, void* buf, size_t maxLen) {
    size_t len = getBytesLength(key);
    if (!len || !buf || !maxLen) {
      return len;
    }
    if (len > maxLen) {
      return 0;
    }
    if (nvs_get_blob(_handle, key, buf, &len)) {
      return 0;
    }
    return len;
  }

  size_t freeEntries() {
    nvs_stats_t nvs_stats;
    if (nvs_get_stats(NULL, &nvs_stats)) {
      return 0;
    }
    return nvs_stats.free_entries;
  }
};
