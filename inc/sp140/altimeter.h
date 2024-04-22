#ifndef INC_SP140_ALTIMETER_H_
#define INC_SP140_ALTIMETER_H_

#include <Arduino.h>

#include "sp140/structs.h"

// Set up the barometer
bool setupAltimeter(bool alt_wire = false);

// Get the altitude (in meters)
float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

// Set the ground altitude to the current altitude to compute AGL
void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData);

#endif  // INC_SP140_ALTIMETER_H_
