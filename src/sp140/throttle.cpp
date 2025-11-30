// Throttle utilities and hardware interface
#include "sp140/throttle.h"
#include "sp140/shared-config.h"
#include "sp140/esp32s3-config.h"
#include <CircularBuffer.hpp>

// Access board configuration from main unit
extern HardwareConfig board_config;

// Internal PWM smoothing buffer (8 samples)
static CircularBuffer<int, 8> pwmBuffer;

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
  int delta = current - last;
  int maxDelta = (delta > 0) ? threshold : threshold * DECEL_MULTIPLIER;
  return last + constrain(delta, -maxDelta, maxDelta);
}

/**
 * Configure the throttle input pin and ADC resolution.
 * ESP32 uses 12-bit ADC resolution (0..4095) for throttle.
 */
void initThrottleInput() {
  pinMode(board_config.throttle_pin, INPUT);
  #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
  #endif
}

/**
 * Read the raw ADC value for the throttle potentiometer.
 * Range: 0..4095 (12-bit).
 */
uint16_t readThrottleRaw() {
  return analogRead(board_config.throttle_pin);
}

/** Map raw ADC (0..4095) to PWM (ESC_MIN_PWM..ESC_MAX_PWM). */
int potRawToPwm(uint16_t raw) {
  // map() already constrains output, no need for explicit constrain()
  return map(raw, POT_MIN_VALUE, POT_MAX_VALUE, ESC_MIN_PWM, ESC_MAX_PWM);
}

/**
 * Apply mode-specific ramp limiting (in microseconds per tick) and clamp
 * the result to the current mode's max PWM. Updates prevPwm to the final value.
 */
int applyModeRampClamp(int pwmAvg, int& prevPwm, uint8_t performance_mode) {
  const int rampRateUs = (performance_mode == 0) ? CHILL_MODE_RAMP_RATE : SPORT_MODE_RAMP_RATE;
  const int maxPWM = (performance_mode == 0) ? CHILL_MODE_MAX_PWM : ESC_MAX_PWM;

  int limitedPwm = limitedThrottle(pwmAvg, prevPwm, rampRateUs);
  int clampedPwm = constrain(limitedPwm, ESC_MIN_PWM, maxPWM);
  prevPwm = clampedPwm;
  return clampedPwm;
}

// Buffer helpers
/** Clear all samples from the internal PWM smoothing buffer. */
void throttleFilterClear() {
  pwmBuffer.clear();
}

/** Clear and pre-fill the buffer with the given PWM value for smooth transitions. */
void throttleFilterReset(int pwmValue) {
  pwmBuffer.clear();
  // Use while loop - cleaner for filling to capacity
  while (!pwmBuffer.isFull()) {
    pwmBuffer.push(pwmValue);
  }
}

/** Push a new PWM sample into the smoothing buffer. */
void throttleFilterPush(int pwmValue) {
  pwmBuffer.push(pwmValue);
}

/** Average and return the current PWM from the smoothing buffer. */
int throttleFilterAverage() {
  if (pwmBuffer.isEmpty()) return 0;

  // Use int32_t for clarity and avoid casts
  int32_t sum = 0;
  for (uint8_t i = 0; i < pwmBuffer.size(); i++) {
    sum += pwmBuffer[i];
  }
  return sum / pwmBuffer.size();  // Integer division is fine for PWM values
}

/**
 * Read throttle input and return smoothed PWM value.
 * This is the core throttle processing pipeline without any state logic.
 *
 * @return Smoothed PWM value from throttle input
 */
int getSmoothedThrottlePwm() {
  // Read and convert raw pot to PWM
  int potValRaw = readThrottleRaw();
  int targetPwm = potRawToPwm(potValRaw);

  // Smooth in PWM domain using ring buffer
  throttleFilterPush(targetPwm);
  return throttleFilterAverage();
}

/**
 * Reset throttle state for clean startup/disarm.
 * Clears smoothing buffer and resets previous PWM tracking.
 */
void resetThrottleState(int& prevPwm) {
  prevPwm = ESC_MIN_PWM;
  throttleFilterClear();
}

/**
 * Calculate the cruise control PWM value from a raw pot reading.
 * Uses the same mapping as normal throttle: map to full range first,
 * then clamp to the mode maximum and cruise maximum.
 *
 * This ensures cruise maintains the actual throttle output level,
 * not a re-mapped value that would cause throttle drop in chill mode.
 */
uint16_t calculateCruisePwm(uint16_t potVal, uint8_t performance_mode, float cruiseMaxPct) {
  // Step 1: Map to full PWM range (same as normal throttle)
  uint16_t pwm = potRawToPwm(potVal);

  // Step 2: Clamp to mode maximum (chill mode caps at lower PWM)
  int maxPwmForMode = (performance_mode == 0) ? CHILL_MODE_MAX_PWM : ESC_MAX_PWM;
  pwm = min(pwm, (uint16_t)maxPwmForMode);

  // Step 3: Apply absolute cruise max cap
  uint16_t absoluteMaxCruisePwm = ESC_MIN_PWM + (uint16_t)((ESC_MAX_PWM - ESC_MIN_PWM) * cruiseMaxPct);
  pwm = min(pwm, absoluteMaxCruisePwm);

  return pwm;
}

/**
 * Check if pot value is in valid range for cruise activation.
 * Must be above engagement level and below max activation threshold.
 */
bool isPotInCruiseActivationRange(uint16_t potVal, uint16_t engagementLevel, float maxActivationPct) {
  uint16_t maxActivationVal = (uint16_t)(POT_MAX_VALUE * maxActivationPct);
  return potVal >= engagementLevel && potVal <= maxActivationVal;
}

/**
 * Check if pot value should trigger cruise disengagement.
 * Returns true when current pot >= threshold percentage of activation value.
 */
bool shouldPotDisengageCruise(uint16_t currentPotVal, uint16_t activationPotVal, float thresholdPct) {
  uint16_t disengageThreshold = (uint16_t)(activationPotVal * thresholdPct);
  return currentPotVal >= disengageThreshold;
}
