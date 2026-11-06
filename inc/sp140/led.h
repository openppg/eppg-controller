#ifndef INC_SP140_LED_H_
#define INC_SP140_LED_H_

#include <Arduino.h>
#include "sp140/led_color.h"

// External declarations
extern uint32_t led_color;

// Set the color of the NeoPixel LED (0x00RRGGBB)
void setLEDColor(uint32_t color);

// Set the state of LEDs (on/off)
void setLEDs(byte state);

// Toggle LED state for blinking
void blinkLED();

#endif  // INC_SP140_LED_H_
