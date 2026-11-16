#ifndef INC_SP140_BMS_CAN_RX_H_
#define INC_SP140_BMS_CAN_RX_H_

#include <stdint.h>

class Adafruit_SPIDevice;

// MCP2515 receive settings the BMS_CAN library does not expose. The caller
// owns the shared SPI bus for the duration of each call.
//
// BMS_CAN::begin() leaves RXB0 accepting every message with rollover off, so
// a frame arriving while RXB0 is still full is dropped and RXB1 is never used:
// one frame of buffering between 100 ms polls (MCP2515 data sheet, 4.2.1 and
// Figure 4-3). Enabling rollover keeps a second frame in RXB1.

// Sets RXB0CTRL.BUKT, keeping receive-any mode. Returns true if it reads back.
bool bmsCanRxEnableRollover(Adafruit_SPIDevice& mcp);

// Returns how many receive buffers overflowed (0..2) since the last call and
// clears those flags. Each one means at least one BMS frame was dropped.
uint8_t bmsCanRxTakeOverflows(Adafruit_SPIDevice& mcp);

#endif  // INC_SP140_BMS_CAN_RX_H_
