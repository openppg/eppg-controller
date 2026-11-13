#include "esp_mac.h"
// Copyright 2021 <Zach Whitehead>
#include "sp140/utilities.h"
#include "platform/runtime.h"

/**
 * Gets a unique ID string for the ESP32 chip
 *
 * @return std::string representation of the chip ID
 */
std::string chipId() {
  uint8_t mac[6]{};
  esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);
  const uint32_t value = (static_cast<uint32_t>(mac[5]) << 0) |
      (static_cast<uint32_t>(mac[4]) << 8) | (static_cast<uint32_t>(mac[3]) << 16);
  return std::to_string(value);
}
