// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// Factory NVS namespace ("openppg-factory"). Mirrors the device_settings.cpp
// pattern: a module mutex serializes writers, and multi-key writes go through
// the raw NVS API so each logical update is ONE commit (no torn saves).

#include "sp140/factory_settings.h"

#include "Arduino.h"
#include <Preferences.h>
#include <nvs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sp140/globals.h"

// 15 chars — the NVS namespace limit. House rule: never abbreviate "openppg".
static const char* FACTORY_NAMESPACE = "openppg-factory";

// Factory keys
static const char* KEY_QC_PASSED = "qc_passed";        // u8
static const char* KEY_QC_ATTEMPTED = "qc_attempted";  // u8 (fail/abort retry)
static const char* KEY_QC_FW = "qc_fw";                // u16 (major<<8 | minor)
static const char* KEY_POT_CALIBRATED = "pot_cal";     // u8
static const char* KEY_POT_MIN = "pot_min";            // u16
static const char* KEY_POT_MAX = "pot_max";            // u16
static const char* KEY_QC_RERUN = "qc_rerun";          // u8 (consumed at boot)
static const char* KEY_QC_RECORD = "qc_record";        // blob (JSON line)
static const char* KEY_NVS_SCRATCH = "nvs_scratch";    // u16 (POST round-trip)

static SemaphoreHandle_t s_factoryMutex = nullptr;

static void factoryEnsureMutex() {
  if (s_factoryMutex == nullptr) {
    s_factoryMutex = xSemaphoreCreateMutex();
  }
}

static void factoryLock() {
  factoryEnsureMutex();
  if (s_factoryMutex != nullptr) {
    xSemaphoreTake(s_factoryMutex, portMAX_DELAY);
  }
}

static void factoryUnlock() {
  if (s_factoryMutex != nullptr) {
    xSemaphoreGive(s_factoryMutex);
  }
}

void factorySettingsInit() {
  // Create the mutex while still single-threaded in setup(), like
  // prefsEnsureMutex() in device_settings.cpp.
  factoryEnsureMutex();
}

// --- reads (Preferences wrapper, read-only) --------------------------------

static Preferences& factoryPrefs() {
  static Preferences prefs;
  return prefs;
}

static bool factoryGetFlag(const char* key) {
  factoryLock();
  bool value = false;
  Preferences& p = factoryPrefs();
  if (p.begin(FACTORY_NAMESPACE, true)) {
    value = p.getUChar(key, 0) == 1;
    p.end();
  }
  factoryUnlock();
  return value;
}

bool factoryQcPassed() { return factoryGetFlag(KEY_QC_PASSED); }
bool factoryQcAttempted() { return factoryGetFlag(KEY_QC_ATTEMPTED); }
bool factoryRerunRequested() { return factoryGetFlag(KEY_QC_RERUN); }

FactoryCal factoryGetCal() {
  FactoryCal cal = {false, 0, 4095};
  factoryLock();
  Preferences& p = factoryPrefs();
  if (p.begin(FACTORY_NAMESPACE, true)) {
    cal.calibrated = p.getUChar(KEY_POT_CALIBRATED, 0) == 1;
    cal.potMin = p.getUShort(KEY_POT_MIN, 0);
    cal.potMax = p.getUShort(KEY_POT_MAX, 4095);
    p.end();
  }
  factoryUnlock();
  return cal;
}

// --- writes (raw NVS, single commit per logical update) --------------------

// Open + apply `fn` + commit + close, under the module mutex.
template <typename Fn>
static bool factoryBatchedWrite(Fn fn) {
  factoryLock();
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(FACTORY_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    factoryUnlock();
    USBSerial.println(F("factory_settings: failed to open NVS for writing"));
    return false;
  }
  bool success = fn(handle);
  success &= (nvs_commit(handle) == ESP_OK);
  nvs_close(handle);
  factoryUnlock();
  if (!success) {
    USBSerial.println(F("factory_settings: write may not have been saved"));
  }
  return success;
}

void factoryMarkQcAttempted() {
  factoryBatchedWrite([](nvs_handle_t h) {
    return nvs_set_u8(h, KEY_QC_ATTEMPTED, 1) == ESP_OK;
  });
}

void factoryWriteQcResult(bool passed, uint16_t fwEncoded) {
  // qc_attempted is already sticky from factoryMarkQcAttempted() at gate entry.
  factoryBatchedWrite([&](nvs_handle_t h) {
    bool ok = (nvs_set_u8(h, KEY_QC_PASSED, passed ? 1 : 0) == ESP_OK);
    ok &= (nvs_set_u16(h, KEY_QC_FW, fwEncoded) == ESP_OK);
    return ok;
  });
}

void factoryWriteCal(uint16_t potMin, uint16_t potMax) {
  factoryBatchedWrite([&](nvs_handle_t h) {
    bool ok = (nvs_set_u16(h, KEY_POT_MIN, potMin) == ESP_OK);
    ok &= (nvs_set_u16(h, KEY_POT_MAX, potMax) == ESP_OK);
    ok &= (nvs_set_u8(h, KEY_POT_CALIBRATED, 1) == ESP_OK);
    return ok;
  });
}

void factoryMarkLegacyUnit(uint16_t fwEncoded) {
  // Existing unit detected at boot: back-fill qc_passed WITHOUT calibration.
  factoryBatchedWrite([&](nvs_handle_t h) {
    bool ok = (nvs_set_u8(h, KEY_QC_PASSED, 1) == ESP_OK);
    ok &= (nvs_set_u16(h, KEY_QC_FW, fwEncoded) == ESP_OK);
    return ok;
  });
}

void factorySetRerunFlag() {
  factoryBatchedWrite([](nvs_handle_t h) {
    return nvs_set_u8(h, KEY_QC_RERUN, 1) == ESP_OK;
  });
}

void factoryClearRerunFlag() {
  factoryBatchedWrite([](nvs_handle_t h) {
    return nvs_set_u8(h, KEY_QC_RERUN, 0) == ESP_OK;
  });
}

bool factoryNvsRoundTrip() {
  const uint16_t magic = 0xA5C3;
  bool wrote = factoryBatchedWrite([&](nvs_handle_t h) {
    return nvs_set_u16(h, KEY_NVS_SCRATCH, magic) == ESP_OK;
  });
  if (!wrote) {
    return false;
  }
  factoryLock();
  Preferences& p = factoryPrefs();
  uint16_t readBack = 0;
  if (p.begin(FACTORY_NAMESPACE, true)) {
    readBack = p.getUShort(KEY_NVS_SCRATCH, 0);
    p.end();
  }
  factoryUnlock();
  return readBack == magic;
}

bool factoryWriteQcRecordBlob(const void* data, size_t len) {
  if (data == nullptr || len == 0) {
    return false;
  }
  return factoryBatchedWrite([&](nvs_handle_t h) {
    return nvs_set_blob(h, KEY_QC_RECORD, data, len) == ESP_OK;
  });
}

size_t factoryReadQcRecordBlob(void* out, size_t maxLen) {
  if (out == nullptr || maxLen == 0) {
    return 0;
  }
  factoryLock();
  size_t readLen = 0;
  nvs_handle_t handle = 0;
  if (nvs_open(FACTORY_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
    size_t required = 0;
    if (nvs_get_blob(handle, KEY_QC_RECORD, nullptr, &required) == ESP_OK &&
        required > 0 && required <= maxLen) {
      if (nvs_get_blob(handle, KEY_QC_RECORD, out, &required) == ESP_OK) {
        readLen = required;
      }
    }
    nvs_close(handle);
  }
  factoryUnlock();
  return readLen;
}
