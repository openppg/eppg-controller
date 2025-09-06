#ifndef INC_SP140_THROTTLE_H_
#define INC_SP140_THROTTLE_H_

#include <Arduino.h>

// Constants related to throttle behavior
#define DECEL_MULTIPLIER 2.0     // How much faster deceleration is vs acceleration

/**
 * Throttle easing function based on threshold/performance mode
 * Limits how quickly throttle can increase or decrease
 *
 * @param current Current throttle value
 * @param last Previous throttle value
 * @param threshold Maximum allowed change per cycle
 * @return Limited throttle value
 */
int limitedThrottle(int current, int last, int threshold);

// Initialize throttle input pin and ADC settings
void initThrottleInput();

// Read raw throttle value (0-4095)
uint16_t readThrottleRaw();

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
 * Main throttle handling function - processes throttle input
 * and applies appropriate limits based on device state
 */
void handleThrottle();

#endif  // INC_SP140_THROTTLE_H_
