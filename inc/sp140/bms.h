#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <BMS_CAN.h>
#include <math.h>

#include "sp140/structs.h"

// BMS-related constants
#define MCP_CS 5        // MCP2515 CS pin
#define MCP_BAUDRATE 250000

// BMS cell probe disconnect handling.
// Requirement: a raw value of exactly -40C indicates a disconnected probe
// unless explicitly kept as a valid reading.
constexpr float BMS_CELL_TEMP_DISCONNECTED_C = -40.0f;

inline float sanitizeBmsCellTempC(float tempC, bool treatDisconnectedAsInvalid = true) {
  // Return NaN for disconnected/invalid readings so monitor/UI logic can skip
  // this probe the same way we handle disconnected ESC motor temp.
  return (!isnan(tempC) &&
          (treatDisconnectedAsInvalid ? tempC > BMS_CELL_TEMP_DISCONNECTED_C : tempC >= BMS_CELL_TEMP_DISCONNECTED_C))
           ? tempC
           : NAN;
}

// External declarations
extern STR_BMS_TELEMETRY_140 bmsTelemetryData;
extern BMS_CAN* bms_can;

// BMS functions
void updateBMSData();
void printBMSData();
