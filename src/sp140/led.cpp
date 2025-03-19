#include "sp140/led.h"
#include "Arduino.h"
#include "sp140/globals.h"
#include "sp140/esp32s3-config.h"

// Global variable to track LED state for blinking
extern HardwareConfig board_config;
byte ledState = LOW;

// Set LED color for NeoPixel
void setLEDColor(uint32_t color) {
  if (board_config.enable_neopixel) {
    led_color = color;
    pixels.setPixelColor(0, color);
    pixels.show();
  }
}

/**
 * Sets the state of the LEDs.
 *
 * If board_config.enable_neopixel is true, sets the color of the NeoPixel LED to led_color
 * when the state is HIGH, and clears the NeoPixel LED when the state is LOW.
 * Otherwise, sets the state of the LED_SW pin to the given state.
 *
 * @param state The state to set the LEDs to (HIGH or LOW).
 */
void setLEDs(byte state) {
  if (board_config.enable_neopixel) {
    if (state == HIGH) {
      pixels.setPixelColor(0, led_color);
    } else {
      pixels.clear();
    }
    pixels.show();
  } else {
    digitalWrite(board_config.led_sw, state);
  }
}

// Toggle LEDs for blinking
void blinkLED() {
  ledState = !ledState;
  setLEDs(ledState);
}
