// Copyright 2021 <Zach Whitehead>
#include "sp140/utilities.h"
#include "Arduino.h"

/**
 * Gets a unique ID string for the ESP32 chip
 *
 * @return String representation of the chip ID
 */
String chipId() {
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  return String(chipId);
}
