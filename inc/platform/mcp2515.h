#pragma once
#include "platform/peripheral.h"
#include <array>
#include <cstring>

namespace platform {
struct CanFrame {
  uint32_t id = 0;
  uint8_t length = 0;
  uint8_t data[8]{};
  bool extended = true;
};
class Mcp2515 {
 public:
  explicit Mcp2515(int cs) : cs_(cs) {}
  bool initialize();
  bool receive(CanFrame* frame);

 private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  void modify(uint8_t reg, uint8_t mask, uint8_t value);
  int cs_;
  void* device_ = nullptr;
};
}  // namespace platform
