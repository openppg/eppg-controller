#pragma once

#include <stdint.h>

// Minimal NimBLE stubs for native builds
class NimBLECharacteristic {
 public:
  void setValue(const uint8_t*, size_t) {}
  void notify() {}
};

class NimBLEServer {
 public:
  uint16_t getConnectedCount() { return 0; }
};

class NimBLEService {};
class NimBLEAdvertising {};

class NimBLEDevice {
 public:
  static void init(const char*) {}
  static NimBLEServer* createServer() { return nullptr; }
};
