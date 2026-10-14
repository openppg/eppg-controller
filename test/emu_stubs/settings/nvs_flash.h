// Host stub of ESP-IDF 4.4 nvs_flash.h on top of emu_nvs.h.
#pragma once

#include "esp_err.h"
#include "nvs.h"

inline esp_err_t nvs_flash_init() {
  emu_nvs::state().initialized = true;
  return ESP_OK;
}

inline esp_err_t nvs_flash_init_partition(const char* part_name) {
  if (part_name == nullptr || strcmp(part_name, NVS_DEFAULT_PART_NAME) != 0) {
    return ESP_ERR_NOT_FOUND;
  }
  return nvs_flash_init();
}

inline esp_err_t nvs_flash_deinit() {
  emu_nvs::state().handles.clear();
  emu_nvs::state().initialized = false;
  return ESP_OK;
}

inline esp_err_t nvs_flash_erase() {
  emuNvsEraseFlash();
  return ESP_OK;
}
