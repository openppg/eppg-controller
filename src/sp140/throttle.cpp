// Throttle utilities and hardware interface
#include "sp140/throttle.h"
#include "sp140/shared-config.h"
#include "sp140/esp32s3-config.h"

// Access board configuration from main unit
extern HardwareConfig board_config;

/**
 * Throttle easing function based on threshold/performance mode
 * Limits how quickly throttle can increase or decrease.
 *
 * Behavior:
 * - Acceleration is limited to `threshold` counts per tick.
 * - Deceleration is limited to `threshold * DECEL_MULTIPLIER` counts per tick.
 *
 * @param current Current throttle value (e.g., averaged pot counts 0..4095)
 * @param last Previous throttle value from the prior tick
 * @param threshold Maximum allowed increase per tick (mode-dependent)
 * @return Limited throttle value to apply this tick
 */
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast
    return last + threshold;
  } else if (last - current >= threshold * DECEL_MULTIPLIER) {  // decelerating too fast
    return last - threshold * DECEL_MULTIPLIER;
  }
  return current;
}

void initThrottleInput() {
  pinMode(board_config.throttle_pin, INPUT);
  #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
  #endif
}

uint16_t readThrottleRaw() {
  return analogRead(board_config.throttle_pin);
}
