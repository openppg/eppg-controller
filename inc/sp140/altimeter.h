#ifndef INC_SP140_ALTIMETER_H_
#define INC_SP140_ALTIMETER_H_

#include "sp140/shared-config.h"
#include "sp140/structs.h"
#include "platform/runtime.h"
#include "platform/ring_buffer.h"
#include <freertos/semphr.h>

// Constants
#define VARIO_BUFFER_SIZE 50  // Number of samples to average for vertical speed
#define MAX_VERTICAL_SPEED 250.0f  // Maximum vertical speed to display (m/s)

// Set up the barometer
bool setupAltimeter();

struct BarometerSnapshot {
  float altitude;
  float temperature;
  float pressure;
  float verticalSpeed;
  uint32_t sampledAt;
  bool valid;
};

// One conversion supplies every consumer; only ctrlSensorTask samples periodically.
bool sampleBarometer(const STR_DEVICE_DATA_140_V1 &deviceData);
BarometerSnapshot getBarometerSnapshot();

// Get cached altitude (in meters), without accessing the sensor.
float getAltitude(const STR_DEVICE_DATA_140_V1 &deviceData);

// Get the vertical speed in meters per second
float getVerticalSpeed();

// Set the ground altitude to the current altitude to compute AGL
void setGroundAltitude(const STR_DEVICE_DATA_140_V1 &deviceData);

// Get the temperature in degrees Celsius
float getBaroTemperature();

// Get the pressure in hPa
float getBaroPressure();

// Cached getters copy a snapshot under a cross-core critical section.
float getCachedAltitude();
float getCachedBaroTemperature();
float getCachedVerticalSpeed();

#endif  // INC_SP140_ALTIMETER_H_
