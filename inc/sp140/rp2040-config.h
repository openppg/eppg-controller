// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_RP2040_CONFIG_H_
#define INC_SP140_RP2040_CONFIG_H_

#include "shared-config.h"

#define SerialESC  Serial1  // ESC UART connection

struct HardwareConfig {
  int button_top;
  int buzzer_pin;
  int led_sw;
  int throttle_pin;
  int bmp_pin;
  HardwareSerial* serial_esc;
  int tft_rst;
  int tft_cs;
  int tft_dc;
  int tft_lite;
  int esc_pin;
  bool enable_vib;
};

// V1 configuration
HardwareConfig v1_config = {
  .button_top = 7,
  .buzzer_pin = 2,
  .led_sw = LED_BUILTIN,
  .throttle_pin = A2,
  .serial_esc = &Serial1,
  .tft_rst = 6,
  .tft_cs = 4,
  .tft_dc = 5,
  .tft_lite = A3,
  .esc_pin = 3,
  .enable_vib = true
};

// V2 configuration
HardwareConfig v2_config = {
  .button_top = 15,
  .buzzer_pin = 10,
  .led_sw = 12,
  .throttle_pin = A0,
  .bmp_pin = 9,
  .serial_esc = &Serial1,
  .tft_rst = 5,
  .tft_cs = 13,
  .tft_dc = 11,
  .tft_lite = 25,
  .esc_pin = 14,
  .enable_vib = false
};
#endif  // INC_SP140_RP2040_CONFIG_H_
