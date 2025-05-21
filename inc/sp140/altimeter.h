#ifndef INC_SP140_ALTIMETER_H_
#define INC_SP140_ALTIMETER_H_

#include <Arduino.h>
#include <CircularBuffer.hpp>
#include "sp140/structs.h"
#include "sp140/shared-config.h"

// Constants
#define VARIO_BUFFER_SIZE 10  // Number of samples to average for vertical speed
#define MAX_VERTICAL_SPEED 250.0f  // Maximum vertical speed to display (m/s)

// Set up the barometer
bool setupAltimeter(bool alt_wire = false);

// Get the altitude (in meters)
float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

// Get the vertical speed in meters per second
float getVerticalSpeed();

// Set the ground altitude to the current altitude to compute AGL
void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

// Get the temperature in degrees Celsius
float getBaroTemperature();

// Get the pressure in hPa (mbar)
float getBaroPressure();

#endif  // INC_SP140_ALTIMETER_H_
