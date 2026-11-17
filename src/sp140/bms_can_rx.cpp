#include "sp140/bms_can_rx.h"

#include <Adafruit_SPIDevice.h>

namespace {

// MCP2515 SPI instructions and registers (data sheet DS21801G, sections 11, 12).
constexpr uint8_t kInstrWrite = 0x02;
constexpr uint8_t kInstrRead = 0x03;
constexpr uint8_t kInstrBitModify = 0x05;
constexpr uint8_t kRegEflg = 0x2D;
constexpr uint8_t kRegRxb0Ctrl = 0x60;

constexpr uint8_t kRxb0CtrlRxmAny = 0x60;  // RXM<1:0> = 11, as BMS_CAN::begin() sets
constexpr uint8_t kRxb0CtrlBukt = 0x04;
constexpr uint8_t kEflgRx0Ovr = 0x40;
constexpr uint8_t kEflgRx1Ovr = 0x80;

uint8_t readRegister(Adafruit_SPIDevice& mcp, uint8_t address) {
  const uint8_t cmd[2] = {kInstrRead, address};
  uint8_t value = 0;
  mcp.write_then_read(cmd, sizeof(cmd), &value, 1);
  return value;
}

}  // namespace

bool bmsCanRxEnableRollover(Adafruit_SPIDevice& mcp) {
  // Plain write of the whole register (the read-only bits ignore it).
  const uint8_t value = kRxb0CtrlRxmAny | kRxb0CtrlBukt;
  const uint8_t cmd[3] = {kInstrWrite, kRegRxb0Ctrl, value};
  mcp.write(cmd, sizeof(cmd));
  return (readRegister(mcp, kRegRxb0Ctrl) & value) == value;
}

uint8_t bmsCanRxTakeOverflows(Adafruit_SPIDevice& mcp) {
  const uint8_t flags = readRegister(mcp, kRegEflg) & (kEflgRx0Ovr | kEflgRx1Ovr);
  if (flags == 0) { return 0; }  // NOLINT(whitespace/newline)
  // Clear only the flags seen; one that sets in between is counted next time.
  const uint8_t cmd[4] = {kInstrBitModify, kRegEflg, flags, 0x00};
  mcp.write(cmd, sizeof(cmd));
  return ((flags & kEflgRx0Ovr) ? 1 : 0) + ((flags & kEflgRx1Ovr) ? 1 : 0);
}
