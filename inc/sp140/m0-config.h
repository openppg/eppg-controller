// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_M0_CONFIG_H_
#define INC_SP140_M0_CONFIG_H_

#include "shared-config.h"

// Arduino Pins
#define BUTTON_TOP    6   // arm/disarm button_top
#define BUZZER_PIN    5   // output for buzzer speaker
#define LED_SW        LED_BUILTIN   // output for LED
#define LED_2         0   // output for LED 2
#define LED_3         38  // output for LED 3
#define THROTTLE_PIN  A0  // throttle pot input

#define SerialESC  Serial5  // ESC UART connection

// SP140
#define POT_PIN A0
#define TFT_RST 9
#define TFT_CS 10
#define TFT_DC 11
#define TFT_LITE A1
#define ESC_PIN 12
#define ENABLE_VIB            true    // enable vibration

struct HardwareConfig {
  int button_top;
  int buzzer_pin;
  int led_sw;
  int led_2;
  int led_3;
  int throttle_pin;
  HardwareSerial* serial_esc;
  int pot_pin;
  int tft_rst;
  int tft_cs;
  int tft_dc;
  int tft_lite;
  int esc_pin;
  bool enable_vib;
};

HardwareConfig m0_config = {
  .button_top = 6,
  .buzzer_pin = 5,
  .led_sw = LED_BUILTIN,
  .led_2 = 0,
  .led_3 = 38,
  .throttle_pin = A0,
  .serial_esc = &Serial5,
  .pot_pin = A0,
  .tft_rst = 9,
  .tft_cs = 10,
  .tft_dc = 11,
  .tft_lite = A1,
  .esc_pin = 12,
  .enable_vib = true
};

#endif  // INC_SP140_M0_CONFIG_H_
