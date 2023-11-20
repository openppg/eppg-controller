// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_RP2040_CONFIG_H_
#define INC_SP140_RP2040_CONFIG_H_

#include "shared-config.h"

// Arduino Pins
#define BUTTON_TOP    6  // arm/disarm button_top
#define BUTTON_SIDE   7   // secondary button_top
#define BUZZER_PIN    10  // output for buzzer speaker
#define LED_SW        LED_BUILTIN  // output for LED
#define THROTTLE_PIN  A0  // throttle pot input

#define SerialESC  Serial1  // ESC UART connection

// SP140
#define POT_PIN A0
#define TFT_RST 2
#define TFT_CS 4
#define TFT_DC 3
#define TFT_LITE 5
#define ESC_PIN 8
#define ENABLE_VIB            false    // enable vibration

#endif  // INC_SP140_RP2040_CONFIG_H_
