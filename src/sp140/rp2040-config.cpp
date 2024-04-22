#include <Arduino.h>

#include "../../inc/sp140/rp2040-config.h"

// V2 configuration
HardwareConfig v2_config = {
  .button_top = 7,
  .buzzer_pin = 2,
  .led_sw = LED_BUILTIN,
  .throttle_pin = A2,
  .bmp_pin = 9,
  .serial_esc = &Serial1,
  .alt_wire = true,
  .tft_rst = 6,
  .tft_cs = 4,
  .tft_dc = 5,
  .tft_lite = A3,
  .esc_pin = 3,
  .enable_vib = true,
  .enable_neopixel = true
};

// V1 configuration
HardwareConfig v1_config = {
  .button_top = 15,
  .buzzer_pin = 10,
  .led_sw = 12,
  .throttle_pin = A0,
  .serial_esc = &Serial1,
  .alt_wire = false,
  .tft_rst = 5,
  .tft_cs = 13,
  .tft_dc = 11,
  .tft_lite = 25,
  .esc_pin = 14,
  .enable_vib = false,
  .enable_neopixel = false
};
