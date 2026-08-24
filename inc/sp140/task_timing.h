#ifndef INC_SP140_TASK_TIMING_H_
#define INC_SP140_TASK_TIMING_H_

#include <stdint.h>

// Advance a periodic task's wake base to the latest elapsed phase boundary.
// Unsigned subtraction intentionally preserves normal 32-bit tick wraparound.
inline uint32_t phaseAlignedWakeBase(uint32_t lastWake, uint32_t now,
                                     uint32_t period) {
  if (period == 0) return lastWake;

  const uint32_t elapsed = now - lastWake;
  if (elapsed < period) return lastWake;
  return lastWake + (elapsed / period) * period;
}

#endif  // INC_SP140_TASK_TIMING_H_
