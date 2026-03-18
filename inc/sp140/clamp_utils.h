#ifndef INC_SP140_CLAMP_UTILS_H_
#define INC_SP140_CLAMP_UTILS_H_

#include <cstdint>

inline int8_t clampI8(int32_t value) {
  if (value < -128) return -128;
  if (value > 127) return 127;
  return static_cast<int8_t>(value);
}

inline uint8_t clampU8(int32_t value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return static_cast<uint8_t>(value);
}

inline int16_t clampI16(int32_t value, int16_t min_v, int16_t max_v) {
  if (value < static_cast<int32_t>(min_v)) return min_v;
  if (value > static_cast<int32_t>(max_v)) return max_v;
  return static_cast<int16_t>(value);
}

inline uint16_t clampU16(int32_t value) {
  if (value < 0) return 0;
  if (value > 65535) return 65535;
  return static_cast<uint16_t>(value);
}

inline uint32_t clampU32(int64_t value, uint32_t min_v, uint32_t max_v) {
  if (value < static_cast<int64_t>(min_v)) return min_v;
  if (value > static_cast<int64_t>(max_v)) return max_v;
  return static_cast<uint32_t>(value);
}

#endif  // INC_SP140_CLAMP_UTILS_H_
