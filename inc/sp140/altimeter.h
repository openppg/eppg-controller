#ifndef INCLUDE_SP140_ALTIMETER_H_
#define INCLUDE_SP140_ALTIMETER_H_

#include <Arduino.h>

#include "sp140/structs.h"

// Set up the barometer
void setupAltimeter();

// Get the altitude (in meters)
float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

// Set the ground altitude to the current altitude to compute AGL
void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

#endif  // INCLUDE_SP140_ALTIMETER_H_
