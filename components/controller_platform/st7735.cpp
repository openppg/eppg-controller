#include "platform/st7735.h"
#include <algorithm>
#include <array>

namespace platform {
bool St7735::command(uint8_t code, const uint8_t* data, size_t size) {
  eppg_gpio_write(dc_, 0);
  if (eppg_spi_transfer(device_, &code, nullptr, 1) != 0) return false;
  if (!size) return true;
  eppg_gpio_write(dc_, 1);
  return eppg_spi_transfer(device_, data, nullptr, size) == 0;
}
bool St7735::initialize(int cs, int dc, int reset, uint8_t rotation) {
  dc_ = dc;
  device_ = eppg_spi_add_device(cs, 16000000, 0);
  if (!device_) return false;
  eppg_gpio_config(dc, true, false);
  eppg_gpio_config(reset, true, false);
  eppg_gpio_write(reset, 1); eppg_delay_ms(100);
  eppg_gpio_write(reset, 0); eppg_delay_ms(100);
  eppg_gpio_write(reset, 1); eppg_delay_ms(200);
  if (!command(0x01)) return false;
  eppg_delay_ms(150);
  command(0x11); eppg_delay_ms(500);
  const uint8_t frame[] = {0x01, 0x2c, 0x2d};
  command(0xb1, frame, 3); command(0xb2, frame, 3);
  const uint8_t frame3[] = {0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d};
  command(0xb3, frame3, 6);
  const uint8_t inversion[] = {7}; command(0xb4, inversion, 1);
  const uint8_t p1[] = {0xa2, 0x02, 0x84}; command(0xc0, p1, 3);
  const uint8_t p2[] = {0xc5}; command(0xc1, p2, 1);
  const uint8_t p3[] = {0x0a, 0x00}; command(0xc2, p3, 2);
  const uint8_t p4[] = {0x8a, 0x2a}; command(0xc3, p4, 2);
  const uint8_t p5[] = {0x8a, 0xee}; command(0xc4, p5, 2);
  const uint8_t vcom[] = {0x0e}; command(0xc5, vcom, 1);
  command(0x20);
  const uint8_t color[] = {5}; command(0x3a, color, 1);
  const uint8_t gamma1[] = {0x02,0x1c,0x07,0x12,0x37,0x32,0x29,0x2d,0x29,0x25,0x2b,0x39,0x00,0x01,0x03,0x10};
  const uint8_t gamma2[] = {0x03,0x1d,0x07,0x06,0x2e,0x2c,0x29,0x2d,0x2e,0x2e,0x37,0x3f,0x00,0x00,0x02,0x10};
  command(0xe0, gamma1, 16); command(0xe1, gamma2, 16);
  // BLACKTAB uses RGB order, and no row/column offset in landscape.
  const uint8_t madctl[] = {static_cast<uint8_t>(rotation == 1 ? 0xa0 : 0x60)};
  command(0x36, madctl, 1);
  command(0x13); eppg_delay_ms(10);
  command(0x29); eppg_delay_ms(100);
  return true;
}
bool St7735::writeRegion(int x, int y, int width, int height, const uint16_t* pixels) {
  const uint8_t columns[] = {static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(x), static_cast<uint8_t>((x + width - 1) >> 8), static_cast<uint8_t>(x + width - 1)};
  const uint8_t rows[] = {static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(y), static_cast<uint8_t>((y + height - 1) >> 8), static_cast<uint8_t>(y + height - 1)};
  if (!command(0x2a, columns, 4) || !command(0x2b, rows, 4) || !command(0x2c)) return false;
  eppg_gpio_write(dc_, 1);
  std::array<uint8_t, 1024> chunk;
  size_t remaining = static_cast<size_t>(width) * height;
  while (remaining) {
    const size_t count = std::min<size_t>(remaining, chunk.size() / 2);
    for (size_t i = 0; i < count; ++i) { chunk[2*i] = pixels[i] >> 8; chunk[2*i+1] = pixels[i] & 0xff; }
    if (eppg_spi_transfer(device_, chunk.data(), nullptr, count * 2) != 0) return false;
    pixels += count; remaining -= count;
  }
  return true;
}
}
