#pragma once

#include <stdint.h>
#include "SPI.h"

#define INITR_BLACKTAB 0
#define ST77XX_BLACK 0x0000

class Adafruit_ST7735 {
 public:
  Adafruit_ST7735(SPIClass* spi, int8_t cs, int8_t dc, int8_t rst) {
    (void)spi; (void)cs; (void)dc; (void)rst;
  }
  void initR(uint8_t) {}
  void setRotation(uint8_t) {}
  void fillScreen(uint16_t) {}
  void setAddrWindow(uint16_t, uint16_t, uint16_t, uint16_t) {}
  void writePixels(uint16_t*, uint32_t) {}
  void startWrite() {}
  void endWrite() {}
};
