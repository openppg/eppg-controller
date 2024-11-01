// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_ESP32S3_CONFIG_H_
#define INC_SP140_ESP32S3_CONFIG_H_

#include "shared-config.h"

struct HardwareConfig {
  int button_top;
  int buzzer_pin;
  int led_sw;
  int throttle_pin;
  int bmp_pin;
  bool alt_wire;
  int tft_rst;
  int tft_cs;
  int tft_dc;
  int tft_lite;
  int tft_mosi;
  int tft_sclk;
  int vibe_pwm;
  int sda_pin;
  int scl_pin;
  bool enable_vib;
  bool enable_neopixel;
};

extern HardwareConfig s3_config;
#endif  // INC_SP140_ESP32S3_CONFIG_H_
