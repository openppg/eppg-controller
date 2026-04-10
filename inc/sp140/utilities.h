#ifndef INC_SP140_UTILITIES_H_
#define INC_SP140_UTILITIES_H_

#include <Arduino.h>

// Function to get unique chip ID
String chipId();

// Controller status LED colors in WRGB format for NeoPixel.
#define STATUS_LED_RED 0x00FF0000
#define STATUS_LED_YELLOW 0x00FFFF00
#define STATUS_LED_GREEN 0x0000FF00

#endif  // INC_SP140_UTILITIES_H_
