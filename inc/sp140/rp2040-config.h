// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_RP2040_CONFIG_H_
#define INC_SP140_RP2040_CONFIG_H_

#include "shared-config.h"

// Arduino Pins
#define BUTTON_TOP    7  // arm/disarm button_top
#define BUTTON_SIDE   7   // secondary button_top
#define BUZZER_PIN    2  // output for buzzer speaker
#define LED_SW        LED_BUILTIN  // output for LED
#define THROTTLE_PIN  A2  // throttle pot input

#define SerialESC  Serial1  // ESC UART connection

// UART pins
// RX is GPIO 1

// SP140
#define TFT_RST 6
#define TFT_CS 4
#define TFT_DC 5
#define TFT_LITE A3
#define ESC_PIN 3
#define ENABLE_VIB            true    // enable vibration

#endif  // INC_SP140_RP2040_CONFIG_H_
