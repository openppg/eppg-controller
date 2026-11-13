#include "sp140/led.h"
#include "platform/runtime.h"
#include "sp140/globals.h"
#include "sp140/esp32s3-config.h"

// Global variable to track LED state for blinking
extern HardwareConfig board_config;
uint8_t ledState = 0;

// Set LED color for NeoPixel
void setLEDColor(uint32_t color) {
  if (board_config.enable_neopixel) {
    led_color = color;
    eppg_pixel_write(board_config.led_sw, color);
  }
}

/**
 * Sets the state of the LEDs.
 *
 * If board_config.enable_neopixel is true, sets the color of the NeoPixel LED to led_color
 * when the state is 1, and clears the NeoPixel LED when the state is 0.
 * Otherwise, sets the state of the LED_SW pin to the given state.
 *
 * @param state The state to set the LEDs to (1 or 0).
 */
void setLEDs(uint8_t state) {
  if (board_config.enable_neopixel) {
    eppg_pixel_write(board_config.led_sw, state ? led_color : 0);
  } else {
    eppg_gpio_write(board_config.led_sw, state);
  }
}

// Toggle LEDs for blinking
void blinkLED() {
  ledState = !ledState;
  setLEDs(ledState);
}
