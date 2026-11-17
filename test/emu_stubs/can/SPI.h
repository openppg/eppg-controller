// SPIClass surface the Adafruit MCP2515 driver names. Nothing is clocked: the
// MCP2515 behind it is modelled in Adafruit_SPIDevice.h.
#pragma once

class SPIClass {
 public:
  void usingInterrupt(int) {}
};

inline SPIClass SPI;
