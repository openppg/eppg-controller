#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <BMS_CAN.h>

#include "sp140/structs.h"

// BMS-related constants
#define MCP_CS 5        // MCP2515 CS pin
#define MCP_BAUDRATE 250000

// BMS cell probe disconnect handling
constexpr float BMS_CELL_TEMP_DISCONNECTED_C = -40.0f;
constexpr float BMS_CELL_TEMP_DISCONNECT_THRESHOLD_C = BMS_CELL_TEMP_DISCONNECTED_C + 0.5f;

inline bool isBmsCellTempValidC(float tempC) {
  return !isnan(tempC) && tempC > BMS_CELL_TEMP_DISCONNECT_THRESHOLD_C;
}

inline float sanitizeBmsCellTempC(float tempC) {
  return isBmsCellTempValidC(tempC) ? tempC : NAN;
}

// External declarations
extern STR_BMS_TELEMETRY_140 bmsTelemetryData;
extern BMS_CAN* bms_can;

// BMS functions
void updateBMSData();
void printBMSData();
