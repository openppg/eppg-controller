#pragma once

#include <stdint.h>

#define HSPI 2

class SPIClass {
 public:
  SPIClass(int bus = 0) { (void)bus; }
  void begin(int sck = -1, int miso = -1, int mosi = -1, int ss = -1) {
    (void)sck; (void)miso; (void)mosi; (void)ss;
  }
  void end() {}
};
