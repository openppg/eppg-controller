#include "sp140/throttle.h"
#include "sp140/shared-config.h"

// Platform-specific hardware config
#ifdef RP_PIO
  #include "sp140/rp2040-config.h"
#elif CAN_PIO
  #include "sp140/esp32s3-config.h"
#else
  #include "sp140/m0-config.h"
#endif

/**
 * Throttle easing function based on threshold/performance mode
 * Limits how quickly throttle can increase or decrease
 *
 * @param current Current throttle value
 * @param last Previous throttle value
 * @param threshold Maximum allowed change per cycle
 * @return Limited throttle value
 */
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast. limit
    // Calculate limited throttle without modifying global
    return last + threshold;
  } else if (last - current >= threshold * DECEL_MULTIPLIER) {  // decelerating too fast. limit
    // Calculate limited throttle without modifying global
    return last - threshold * DECEL_MULTIPLIER;
  }
  // Return current value without modification
  return current;
}
