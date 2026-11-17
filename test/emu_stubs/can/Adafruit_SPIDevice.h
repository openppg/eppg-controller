// Adafruit BusIO SPI device whose far end is a model of the MCP2515 CAN
// controller, so the real Adafruit MCP2515 driver, BMS_CAN and the firmware's
// own register access run against it.
//
// The receive path follows the MCP2515 data sheet (DS21801G, section 4.2 and
// the receive flowchart, Figure 4-3): a message meeting RXB0's acceptance
// criteria goes to RXB0 if RX0IF is clear; if RXB0 is full it rolls over to
// RXB1 only when RXB0CTRL.BUKT is set, otherwise it is dropped and
// EFLG.RX0OVR is set. A rolled-over message that finds RXB1 full sets
// EFLG.RX1OVR. Only RXM=11 on RXB0 ("turn mask/filters off; receive any
// message", the mode Adafruit_MCP2515::begin() sets) is modelled.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include "SPI.h"

enum { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 };
typedef enum { SPI_BITORDER_LSBFIRST, SPI_BITORDER_MSBFIRST } BusIOBitOrder;

struct EmuMcp2515 {
  static constexpr uint8_t kCanintf = 0x2C;
  static constexpr uint8_t kEflg = 0x2D;
  static constexpr uint8_t kCanstat = 0x0E;
  static constexpr uint8_t kCanctrl = 0x0F;
  static constexpr uint8_t kRxb0Ctrl = 0x60;
  static constexpr uint8_t kRx0if = 0x01, kRx1if = 0x02;
  static constexpr uint8_t kRx0ovr = 0x40, kRx1ovr = 0x80;
  static constexpr uint8_t kBukt = 0x04;

  uint8_t regs[0x80] = {};
  uint32_t framesOnBus = 0;
  uint32_t framesStored = 0;
  uint32_t framesLost = 0;

  void reset() {
    memset(regs, 0, sizeof(regs));
    regs[kCanstat] = 0x80;
    regs[kCanctrl] = 0x87;
  }

  void writeReg(uint8_t addr, uint8_t value) {
    addr &= 0x7F;
    if (addr == kEflg) {
      // Only the receive overflow flags are writable, and only to clear them.
      regs[addr] = static_cast<uint8_t>((regs[addr] & 0x3F) | (value & 0xC0));
    } else if (addr == kRxb0Ctrl) {
      // RXRTR and FILHIT0 are read-only; BUKT1 is a read-only copy of BUKT.
      regs[addr] = static_cast<uint8_t>((value & 0x64) | ((value & kBukt) ? 0x02 : 0) |
                                        (regs[addr] & 0x09));
    } else {
      regs[addr] = value;
    }
  }

  // A complete extended frame arriving from the bus.
  void receive(uint32_t id, const uint8_t* data, uint8_t len) {
    framesOnBus++;
    const bool rxb0AcceptsAll = (regs[kRxb0Ctrl] & 0x60) == 0x60;
    if (!rxb0AcceptsAll) {
      framesLost++;  // acceptance filters are not modelled
      return;
    }
    if ((regs[kCanintf] & kRx0if) == 0) {
      load(0, id, data, len);
      regs[kCanintf] |= kRx0if;
    } else if (regs[kRxb0Ctrl] & kBukt) {
      if ((regs[kCanintf] & kRx1if) == 0) {
        load(1, id, data, len);
        regs[kCanintf] |= kRx1if;
      } else {
        regs[kEflg] |= kRx1ovr;
        framesLost++;
      }
    } else {
      regs[kEflg] |= kRx0ovr;
      framesLost++;
    }
  }

  void load(int n, uint32_t id, const uint8_t* data, uint8_t len) {
    const uint8_t base = static_cast<uint8_t>(0x61 + 0x10 * n);
    regs[base + 0] = static_cast<uint8_t>(id >> 21);
    regs[base + 1] = static_cast<uint8_t>((((id >> 18) & 0x07) << 5) | 0x08 | ((id >> 16) & 0x03));
    regs[base + 2] = static_cast<uint8_t>(id >> 8);
    regs[base + 3] = static_cast<uint8_t>(id);
    regs[base + 4] = len;
    for (uint8_t i = 0; i < len && i < 8; i++) regs[base + 5 + i] = data[i];
    framesStored++;
  }

  // One SPI transaction (CS low ... CS high).
  void transfer(const uint8_t* out, size_t outLen, uint8_t* in, size_t inLen) {
    if (outLen == 0) return;
    switch (out[0]) {
      case 0xC0:  // RESET
        reset();
        break;
      case 0x02:  // WRITE addr, data...
        for (size_t i = 2; i < outLen; i++) writeReg(static_cast<uint8_t>(out[1] + (i - 2)), out[i]);
        break;
      case 0x03:  // READ addr
        for (size_t i = 0; i < inLen; i++) in[i] = regs[(out[1] + i) & 0x7F];
        break;
      case 0x05:  // BIT MODIFY addr, mask, data
        if (outLen >= 4) {
          const uint8_t addr = out[1] & 0x7F;
          writeReg(addr, static_cast<uint8_t>((regs[addr] & ~out[2]) | (out[3] & out[2])));
        }
        break;
      default:
        break;
    }
  }
};

inline EmuMcp2515& emuMcp2515() {
  static EmuMcp2515 chip;
  return chip;
}

class Adafruit_SPIDevice {
 public:
  Adafruit_SPIDevice(int8_t, uint32_t = 1000000, BusIOBitOrder = SPI_BITORDER_MSBFIRST, uint8_t = SPI_MODE0,
                     SPIClass* = &SPI) {}
  Adafruit_SPIDevice(int8_t, int8_t, int8_t, int8_t, uint32_t = 1000000, BusIOBitOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t = SPI_MODE0) {}

  bool begin() { return true; }

  bool write(const uint8_t* buffer, size_t len, const uint8_t* prefix = nullptr, size_t prefixLen = 0) {
    std::vector<uint8_t> out;
    if (prefix) out.insert(out.end(), prefix, prefix + prefixLen);
    out.insert(out.end(), buffer, buffer + len);
    emuMcp2515().transfer(out.data(), out.size(), nullptr, 0);
    return true;
  }

  bool write_then_read(const uint8_t* out, size_t outLen, uint8_t* in, size_t inLen, uint8_t = 0xFF) {
    emuMcp2515().transfer(out, outLen, in, inLen);
    return true;
  }
};
