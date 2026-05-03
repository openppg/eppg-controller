#pragma once

#include <stdint.h>

// Stub BMS_CAN class for native builds
class BMS_CAN {
 public:
  static constexpr float TEMP_PROBE_DISCONNECTED = -999.0f;
  BMS_CAN() {}
};
