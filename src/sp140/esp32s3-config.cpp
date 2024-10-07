#include <Arduino.h>

#include "../../inc/sp140/esp32s3-config.h"

#define LED_BUILTIN  21  // m5 stack
// V1 configuration
// Some SPI pins are defined in platformio.ini
HardwareConfig v3_config = {
  .button_top = 1,
  .buzzer_pin = 8,
  .led_sw = LED_BUILTIN,
  .throttle_pin = 7,
  .alt_wire = false,
  .tft_rst = 15,
  .tft_cs = 13,
  .tft_dc = 14,
  .tft_lite = 25,  // NA
  .vibe_pwm = 27,
  .sda_pin = 44,
  .scl_pin = 41,
  .enable_vib = true,
  .enable_neopixel = true
};
