#include "sp140/led.h"
#include "Arduino.h"
#include "sp140/globals.h"
#include "sp140/esp32s3-config.h"

// Global variable to track LED state for blinking
extern HardwareConfig board_config;
byte ledState = LOW;

// Single WS2812 status LED (GRB). Driven with the core's neopixelWrite(),
// which lazily claims one RMT channel on first use.
static constexpr uint8_t kStatusNeoPixelPin = 21;

static void writeStatusPixel(uint32_t color) {
  const RgbColor rgb = unpackRgb(color);
  neopixelWrite(kStatusNeoPixelPin, rgb.r, rgb.g, rgb.b);
}

// Set LED color for NeoPixel
void setLEDColor(uint32_t color) {
  if (board_config.enable_neopixel) {
    led_color = color;
    writeStatusPixel(color);
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
    writeStatusPixel(state == HIGH ? led_color : 0);
  } else {
    digitalWrite(board_config.led_sw, state);
  }
}

// Toggle LEDs for blinking
void blinkLED() {
  ledState = !ledState;
  setLEDs(ledState);
}
