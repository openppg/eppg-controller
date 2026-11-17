// Declaration-only NimBLE stub (globals.h -> ble.h names these types).
#pragma once

#include <stddef.h>
#include <stdint.h>

class NimBLECharacteristic {
 public:
  void setValue(const uint8_t*, size_t) {}
  void notify() {}
};

class NimBLEServer {
 public:
  uint16_t getConnectedCount() { return 0; }
};
