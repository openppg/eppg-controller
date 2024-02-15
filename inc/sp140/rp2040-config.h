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

#endif  // INC_SP140_RP2040_CONFIG_H_
