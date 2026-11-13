#pragma once
#include "platform/peripheral.h"
#include <cstdint>

namespace platform {
class St7735 {
 public:
  bool initialize(int cs, int dc, int reset, uint8_t rotation);
  bool writeRegion(int x, int y, int width, int height, const uint16_t* pixels);
 private:
  bool command(uint8_t code, const uint8_t* data = nullptr, size_t size = 0);
  void* device_ = nullptr;
  int dc_ = -1;
};
}
