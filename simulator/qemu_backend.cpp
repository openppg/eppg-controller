// Application-level peripheral models for Espressif ESP32-S3 QEMU.
// CPU, interrupts, FreeRTOS and flash execute in QEMU. Unsupported SPI/I2C/
// GPIO/ADC devices are intercepted at the native platform driver boundary.
#include "devices/peripherals.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
using namespace eppg::sim::devices;
St7735 display;
Mcp2515 mcp;
Bmp3xx barometer;
std::array<int, 49> gpio{};
SemaphoreHandle_t mutex = nullptr;
uint32_t bms_sequence = 0;
uint32_t previous_bms_poll = 0;
uint32_t previous_report = 0;
uint32_t now_ms() { return uint32_t(esp_timer_get_time() / 1000); }

class Lock {
 public:
  Lock() {
    // Peripheral initialization is single-threaded before application tasks.
    if (!mutex) mutex = xSemaphoreCreateMutex();
    if (mutex) xSemaphoreTake(mutex, portMAX_DELAY);
  }
  ~Lock() { if (mutex) xSemaphoreGive(mutex); }
};
CanFrame battery_frame(unsigned sequence) {
  CanFrame f; f.length = 8;
  switch (sequence % 9) {
    case 0: f.id = 0x18ff28f4; f.data = {0x18,90,0x88,0x13,0xd2,3,0,0}; break;
    case 1: f.id = 0x18fe28f4; f.data = {0x08,0x10,0xf4,0x0f,65,65,0,0}; break;
    case 2: f.id = 0x18b428f4; f.data = {65,65,65,65,65,65,0,0}; break;
    default:
      f.id = 0x18c828f4 + ((sequence % 9 - 3) << 16);
      f.data = {0,0x10,0,0x10,0,0x10,0,0x10};
      break;
  }
  return f;
}
void report() {
  const uint32_t now = now_ms();
  if (now - previous_report < 1000) return;
  previous_report = now;
  // Stable evidence marker consumed by qemu_smoke.py. Screen checksum is of
  // bytes actually written by production display/SPI code, not a fake screen.
  uint32_t hash = 2166136261u;
  for (uint16_t pixel : display.pixels()) {
    hash = (hash ^ uint8_t(pixel)) * 16777619u;
    hash = (hash ^ uint8_t(pixel >> 8)) * 16777619u;
  }
  std::printf("EPPG_QEMU {\"time_ms\":%lu,\"pixels\":%llu,\"screen_fnv1a\":%lu,\"mcp_overruns\":%llu}\n",
      static_cast<unsigned long>(now), static_cast<unsigned long long>(display.pixel_writes()),
      static_cast<unsigned long>(hash), static_cast<unsigned long long>(mcp.overruns()));
}
}

extern "C" int eppg_qemu_gpio_read(int pin) {
  Lock lock;
  if (pin < 0 || pin >= int(gpio.size())) return 0;
  // Automatic smoke scenario exercises the real button debounce/gesture task.
  // First click 4.0-4.1s, second hold 4.4-6.7s, after the 2s boot splash.
  if (pin == 1) {
    const uint32_t now = now_ms();
    return ((now >= 4000 && now < 4100) || (now >= 4400 && now < 6700)) ? 0 : 1;
  }
  return gpio[size_t(pin)];
}
extern "C" void eppg_qemu_gpio_write(int pin, int value) {
  Lock lock;
  if (pin >= 0 && pin < int(gpio.size())) gpio[size_t(pin)] = value != 0;
}
extern "C" int eppg_qemu_adc_read(int) {
  Lock lock;
  const uint32_t now = now_ms();
  // Raw 12-bit input stays released until the arm gesture completes.
  return now < 7200 ? 0 : int(std::min<uint32_t>(4095, (now - 7200) * 2));
}
extern "C" int eppg_qemu_spi_transfer(void* handle, const uint8_t* tx, uint8_t* rx, size_t length) {
  Lock lock;
  const int cs = int(reinterpret_cast<uintptr_t>(handle)) - 1;
  if (rx) std::fill_n(rx, length, 0);
  if (cs == 10) {
    display.transfer(gpio[14] != 0, tx, length);
  } else if (cs == 5) {
    const uint32_t now = now_ms();
    if (now - previous_bms_poll >= 20) {
      previous_bms_poll = now;
      mcp.receive(battery_frame(bms_sequence++));
    }
    mcp.transfer(tx, rx, length);
  } else return -1;
  report();
  return 0;
}
extern "C" int eppg_qemu_i2c_read(uint8_t address, uint8_t reg, uint8_t* data, size_t length) {
  Lock lock;
  if (address != 0x77 && address != 0x76) return -1;
  return barometer.read(reg, data, length) ? 0 : -1;
}
extern "C" int eppg_qemu_i2c_write(uint8_t address, uint8_t reg, const uint8_t* data, size_t length) {
  Lock lock;
  if (address != 0x77 && address != 0x76) return -1;
  return barometer.write(reg, data, length) ? 0 : -1;
}
