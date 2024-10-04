/*
 * globals.cpp
 *
 * This file exists to define global variables that are used across multiple
 * files in the project. It's paired with globals.h, which declares these
 * variables as extern.
 *
 * IMPORTANT NOTE:
 * Global variables should be avoided whenever possible as they can lead to:
 * 1. Difficulty in tracking state changes
 * 2. Increased coupling between different parts of the code
 * 3. Potential thread-safety issues in multi-threaded environments
 * 4. Challenges in unit testing
 *
 * Instead, consider using:
 * - Function parameters
 * - Return values
 * - Class member variables
 * - Dependency injection
 *
 * If you find yourself needing to add a new global variable, first consider
 * if there's a way to refactor the code to avoid it. If a global is truly
 * necessary, add it here and in globals.h, and document why it's needed.
 */

#include "Arduino.h"

#include "sp140/globals.h"

byte escData[ESC_DATA_SIZE];
byte escDataV2[ESC_DATA_V2_SIZE];
unsigned long cruisedAtMillis = 0;
volatile bool cruising = false;
int prevPotLvl = 0;
int cruisedPotVal = 0;
volatile float throttlePWM = 0;
bool throttledFlag = true;

float watts = 0;
float wattHoursUsed = 0;

int16_t _amps = 0;

STR_DEVICE_DATA_140_V1 deviceData;
