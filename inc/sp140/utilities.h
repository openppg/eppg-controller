#ifndef INC_SP140_UTILITIES_H_
#define INC_SP140_UTILITIES_H_

double mapd(double x, double in_min, double in_max, double out_min, double out_max);

// Definitions for main rainbow colors in WRGB format for NeoPixel.
// The 32-bit color value is WRGB. W (White) is ignored for RGB pixels.
// The next bytes are R (Red), G (Green), and B (Blue).
// For example, YELLOW is 0x00FFFF00, with FF for Red and Green, and 00 for Blue.

#define LED_RED 0x00FF0000
#define LED_ORANGE 0x00FF7F00
#define LED_YELLOW 0x00FFFF00
#define LED_GREEN 0x0000FF00
#define LED_BLUE 0x000000FF
#define LED_INDIGO 0x004B0082
#define LED_VIOLET 0x008000FF

#endif  // INC_SP140_UTILITIES_H_
