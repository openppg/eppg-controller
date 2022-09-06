// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_RP2040_CONFIG_H_
#define INC_SP140_RP2040_CONFIG_H_

#include "shared-config.h"

// Arduino Pins
#define BUTTON_TOP    15  // arm/disarm button_top
#define BUTTON_SIDE   7   // secondary button_top
#define BUZZER_PIN    10  // output for buzzer speaker
#define LED_SW        12  // output for LED
#define THROTTLE_PIN  A0  // throttle pot input

#define SerialESC  Serial1  // ESC UART connection

// SP140
#define POT_PIN A0
#define TFT_RST 5
#define TFT_CS 13
#define TFT_DC 11
#define TFT_LITE 25
#define ESC_PIN 14

#endif  // INC_SP140_RP2040_CONFIG_H_
