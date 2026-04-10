#ifndef INC_SP140_UTILITIES_H_
#define INC_SP140_UTILITIES_H_

#include <Arduino.h>

// Function to get unique chip ID
String chipId();

// Definitions for the controller status LED colors in WRGB format.
// Keep this limited to the status colors we actually use so it does not imply
// ESC protocol colors that the SINE library does not support.

#define STATUS_LED_RED 0x00FF0000
#define STATUS_LED_YELLOW 0x00FFFF00
#define STATUS_LED_GREEN 0x0000FF00

#endif  // INC_SP140_UTILITIES_H_
