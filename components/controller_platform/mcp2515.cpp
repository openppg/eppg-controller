#include <algorithm>
#include "platform/mcp2515.h"

namespace platform {
uint8_t Mcp2515::readRegister(uint8_t reg) {
  uint8_t tx[] = {0x03, reg, 0}, rx[3]{};
  return eppg_spi_transfer(device_, tx, rx, sizeof(tx)) == 0 ? rx[2] : 0;
}
void Mcp2515::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t tx[] = {0x02, reg, value};
  eppg_spi_transfer(device_, tx, nullptr, sizeof(tx));
}
void Mcp2515::modify(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t tx[] = {0x05, reg, mask, value};
  eppg_spi_transfer(device_, tx, nullptr, sizeof(tx));
}
bool Mcp2515::initialize() {
  device_ = eppg_spi_add_device(cs_, 10000000, 0);
  if (!device_) return false;
  const uint8_t reset = 0xc0;
  if (eppg_spi_transfer(device_, &reset, nullptr, 1) != 0) return false;
  eppg_delay_ms(10);
  writeRegister(0x0f, 0x80);
  if ((readRegister(0x0e) & 0xe0) != 0x80) return false;
  // 16 MHz oscillator / 250 kbit/s, identical timing to installed hardware.
  writeRegister(0x2a, 0x41);
  writeRegister(0x29, 0xf1);
  writeRegister(0x28, 0x85);
  writeRegister(0x2b, 0x03);
  writeRegister(0x0c, 0);
  writeRegister(0x0d, 0);
  writeRegister(0x60, 0x60);
  writeRegister(0x70, 0x60);
  writeRegister(0x0f, 0);
  return (readRegister(0x0e) & 0xe0) == 0;
}
bool Mcp2515::receive(CanFrame* frame) {
  const uint8_t pending = readRegister(0x2c) & 3;
  if (!pending || !frame) return false;
  const uint8_t buffer = pending & 1 ? 0 : 1;
  const uint8_t base = buffer ? 0x71 : 0x61;
  uint8_t tx[15]{0x03, base}, rx[15]{};
  if (eppg_spi_transfer(device_, tx, rx, sizeof(tx)) != 0) return false;
  const uint8_t* data = rx + 2;
  frame->extended = data[1] & 8;
  frame->id = static_cast<uint32_t>(data[0]) << 3 | data[1] >> 5;
  if (frame->extended) frame->id = (frame->id << 18) |
      (static_cast<uint32_t>(data[1] & 3) << 16) |
      (static_cast<uint32_t>(data[2]) << 8) | data[3];
  frame->length = std::min<uint8_t>(data[4] & 0x0f, 8);
  memcpy(frame->data, data + 5, frame->length);
  modify(0x2c, 1 << buffer, 0);
  return true;
}
}
