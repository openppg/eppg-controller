#ifndef INC_SP140_ALTIMETER_H_
#define INC_SP140_ALTIMETER_H_

#include "sp140/shared-config.h"
#include "sp140/structs.h"
#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <freertos/semphr.h>

// Constants
#define VARIO_BUFFER_SIZE 50  // Number of samples to average for vertical speed
#define MAX_VERTICAL_SPEED 250.0f  // Maximum vertical speed to display (m/s)
#define BAROMETER_ARM_MAX_AGE_MS 250  // Reject stale samples during arm preflight

struct BarometerSnapshot {
  float relativeAltitude;
  float absoluteAltitude;
  float temperatureC;
  float pressureHpa;
  float verticalSpeedMps;
  uint32_t sampledAtMs;
  bool valid;
};

// Set up the barometer
bool setupAltimeter();

// Perform one pressure/temperature conversion and atomically publish all
// derived values. Called only by the dedicated 25 Hz barometer task.
bool sampleBarometer(const STR_DEVICE_DATA_140_V1 &deviceData);

// Copy the latest coherent sample. Returns false until the first conversion.
bool getBarometerSnapshot(BarometerSnapshot *snapshot);

// Get the altitude (in meters)
float getAltitude(const STR_DEVICE_DATA_140_V1 &deviceData);

// Get the vertical speed in meters per second
float getVerticalSpeed();

// Set the ground altitude from a fresh cached sample. Returns false if the
// sensor has not sampled recently enough to establish a trustworthy datum.
bool setGroundAltitude(const STR_DEVICE_DATA_140_V1 &deviceData);

// Get the temperature in degrees Celsius
float getBaroTemperature();

// Get the pressure in hPa
float getBaroPressure();

// Compatibility getters backed by the coherent snapshot.
float getCachedAltitude();
float getCachedBaroTemperature();
float getCachedBaroPressure();
float getCachedVerticalSpeed();

#endif  // INC_SP140_ALTIMETER_H_
