#ifndef INC_SP140_LED_H_
#define INC_SP140_LED_H_

#include "platform/runtime.h"


// External declarations

extern uint32_t led_color;

// Set the color of the NeoPixel LED
void setLEDColor(uint32_t color);

// Set the state of LEDs (on/off)
void setLEDs(uint8_t state);

// Toggle LED state for blinking
void blinkLED();

#endif  // INC_SP140_LED_H_
