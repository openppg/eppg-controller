#include <Arduino.h>

#include "../../inc/sp140/esp32s3-config.h"
#define LED_BUILTIN 18

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
  .enable_vib = false,
  .enable_neopixel = false
};
