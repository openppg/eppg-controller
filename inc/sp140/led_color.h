#ifndef INC_SP140_LED_COLOR_H_
#define INC_SP140_LED_COLOR_H_

#include <stdint.h>

struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Unpack a 0x00RRGGBB status color (see STATUS_LED_* in utilities.h).
// The top byte is ignored. No Arduino dependency so it is host-testable.
inline RgbColor unpackRgb(uint32_t color) {
  return RgbColor{
    static_cast<uint8_t>((color >> 16) & 0xFF),
    static_cast<uint8_t>((color >> 8) & 0xFF),
    static_cast<uint8_t>(color & 0xFF),
  };
}

#endif  // INC_SP140_LED_COLOR_H_
