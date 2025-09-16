#ifndef INC_SP140_THROTTLE_H_
#define INC_SP140_THROTTLE_H_

#include <Arduino.h>

/**
 * Throttle module (PWM-first pipeline)
 *
 * Responsibilities:
 * - Initialize throttle ADC input
 * - Read raw potentiometer value (0..4095)
 * - Convert raw value to PWM (ESC_MIN_PWM..ESC_MAX_PWM)
 * - Smooth PWM via an internal 8-sample ring buffer
 * - Apply mode-based ramp limiting and clamping in PWM domain
 */

// Constants related to throttle behavior
#define DECEL_MULTIPLIER 2.0     // How much faster deceleration is vs acceleration

// Throttle control (PWM-first) constants
// Ramping in PWM microseconds per tick (~20ms per tick in throttle task)
#define CHILL_MODE_MAX_PWM 1850   // 85% max power in chill mode
#define CHILL_MODE_RAMP_RATE 10   // us/tick in chill mode (~1.6s 1035->1850)
#define SPORT_MODE_RAMP_RATE 27   // us/tick in sport mode (~0.68s 1035->1950)

/**
 * Limits how quickly a value may change between ticks.
 * - Caps acceleration to `threshold` per tick.
 * - Caps deceleration to `threshold * DECEL_MULTIPLIER` per tick.
 *
 * @param current  Proposed value for this tick
 * @param last     Value applied in the previous tick
 * @param threshold Max allowed increase per tick (units depend on caller)
 * @return Value adjusted to respect ramp limits
 */
int limitedThrottle(int current, int last, int threshold);

/** Initialize throttle input pin and ADC (12-bit on ESP32). */
void initThrottleInput();

/** Read raw throttle value from ADC (0..4095). */
uint16_t readThrottleRaw();

/** Convert raw pot reading (0..4095) to PWM microseconds. */
int potRawToPwm(uint16_t raw);

/**
 * Apply mode-based ramp (us/tick) and clamp to the mode's max PWM.
 * Updates `prevPwm` with the final value.
 *
 * @param pwmAvg            Smoothed PWM input (microseconds)
 * @param prevPwm           Reference to previous PWM for ramping
 * @param performance_mode  0 = CHILL, 1 = SPORT
 * @return Final PWM to send to ESC (microseconds)
 */
int applyModeRampClamp(int pwmAvg, int& prevPwm, uint8_t performance_mode);

// Throttle PWM filter (8-sample ring buffer)
/** Clear the internal PWM smoothing buffer. */
void throttleFilterClear();
/** Clear and pre-fill the buffer with `pwmValue` for a smooth restart. */
void throttleFilterReset(int pwmValue);
/** Push a new PWM sample into the smoothing buffer. */
void throttleFilterPush(int pwmValue);
/** Return the averaged PWM from the smoothing buffer. */
int throttleFilterAverage();

/**
 * Checks if throttle is in safe position (below threshold)
 *
 * @param threshold Threshold level to consider safe
 * @return true if throttle is below threshold, false otherwise
 */
bool throttleSafe(int threshold);

/**
 * Checks if throttle is engaged (above engagement threshold)
 *
 * @return true if throttle is engaged, false otherwise
 */
bool throttleEngaged();

/**
 * Read throttle input and return smoothed PWM value.
 * This is the core throttle processing pipeline without any state logic.
 *
 * @return Smoothed PWM value from throttle input
 */
int getSmoothedThrottlePwm();

/**
 * Reset throttle state for clean startup/disarm.
 * Clears smoothing buffer and resets previous PWM tracking.
 *
 * @param prevPwm Reference to previous PWM variable to reset
 */
void resetThrottleState(int& prevPwm);

/**
 * Main throttle handling function - processes throttle input
 * and applies appropriate limits based on device state
 */
void handleThrottle();

#endif  // INC_SP140_THROTTLE_H_
