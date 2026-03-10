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
// The BMS reports TEMP_PROBE_DISCONNECTED (-40C) for unplugged probes.
// Detection is handled by the BMS_CAN library; policy (how many disconnected
// probes to tolerate) lives here in the application.
constexpr uint8_t BMS_CELL_PROBE_COUNT = 4;
constexpr uint8_t BMS_MAX_IGNORED_DISCONNECTED_PROBES = 2;

inline float sanitizeBmsCellTempC(float tempC) {
  // Return NaN for disconnected/invalid readings so monitor/UI logic can skip
  // this probe the same way we handle disconnected ESC motor temp.
  return (!isnan(tempC) && tempC > BMS_CAN::TEMP_PROBE_DISCONNECTED) ? tempC : NAN;
}

inline void sanitizeCellProbeTemps(
    const float rawTemps[BMS_CELL_PROBE_COUNT],
    float sanitizedTemps[BMS_CELL_PROBE_COUNT]) {
  uint8_t ignoredDisconnectedProbeCount = 0;

  for (uint8_t i = 0; i < BMS_CELL_PROBE_COUNT; i++) {
    const float tempC = rawTemps[i];

    if (isnan(tempC) || tempC < BMS_CAN::TEMP_PROBE_DISCONNECTED) {
      sanitizedTemps[i] = NAN;
      continue;
    }

    if (tempC == BMS_CAN::TEMP_PROBE_DISCONNECTED) {
      if (ignoredDisconnectedProbeCount < BMS_MAX_IGNORED_DISCONNECTED_PROBES) {
        sanitizedTemps[i] = NAN;
        ignoredDisconnectedProbeCount++;
      } else {
        sanitizedTemps[i] = tempC;
      }
      continue;
    }

    sanitizedTemps[i] = tempC;
  }
}

// External declarations
extern STR_BMS_TELEMETRY_140 bmsTelemetryData;
extern BMS_CAN* bms_can;

// BMS functions
void updateBMSData();
void printBMSData();
