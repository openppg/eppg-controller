#pragma once

#include <cstdint>

#include "esp_err.h"

typedef enum {
  ESP_PARTITION_TYPE_APP = 0x00,
  ESP_PARTITION_TYPE_DATA = 0x01,
} esp_partition_type_t;

typedef enum {
  ESP_PARTITION_SUBTYPE_APP_OTA_0 = 0x10,
  ESP_PARTITION_SUBTYPE_APP_OTA_1 = 0x11,
} esp_partition_subtype_t;

typedef struct {
  void* flash_chip;
  esp_partition_type_t type;
  esp_partition_subtype_t subtype;
  uint32_t address;
  uint32_t size;
  uint32_t erase_size;
  char label[17];
  bool encrypted;
} esp_partition_t;
