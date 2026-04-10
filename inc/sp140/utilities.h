#ifndef INC_SP140_UTILITIES_H_
#define INC_SP140_UTILITIES_H_

#include <Arduino.h>

// Function to get unique chip ID
String chipId();

// Definitions for main rainbow colors in WRGB format for NeoPixel.
// The 32-bit color value is WRGB. W (White) is ignored for RGB pixels.
// The next bytes are R (Red), G (Green), and B (Blue).
// For example, YELLOW is 0x00FFFF00, with FF for Red and Green, and 00 for Blue.

#define NEOPIXEL_RED 0x00FF0000
#define NEOPIXEL_ORANGE 0x00FF7F00
#define NEOPIXEL_YELLOW 0x00FFFF00
#define NEOPIXEL_GREEN 0x0000FF00
#define NEOPIXEL_BLUE 0x000000FF
#define NEOPIXEL_INDIGO 0x004B0082
#define NEOPIXEL_VIOLET 0x008000FF

#endif  // INC_SP140_UTILITIES_H_
