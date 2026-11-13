#pragma once
#include "bmp3.h"
#include <cmath>

namespace platform {
class Barometer {
 public:
  bool initialize();
  float readAltitude(float sea_pressure_hpa);
  float readTemperature();
  float readPressure();

 private:
  bool sample();
  bmp3_dev sensor_{};
  double temperature_ = 0;
  double pressure_ = 0;
};
}  // namespace platform
