#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <BMS_CAN.h>
#include <math.h>

#include "sp140/structs.h"

// BMS-related constants
#define MCP_CS 5        // MCP2515 CS pin
#define MCP_BAUDRATE 250000

// BMS cell probe disconnect policy.
// The library returns NaN for disconnected probes. This app tolerates up to
// BMS_MAX_IGNORED_DISCONNECTED_PROBES disconnected; beyond that the NaN is
// replaced with the sentinel value so downstream temp monitors fire an alert.
constexpr uint8_t BMS_CELL_PROBE_COUNT = 4;
constexpr uint8_t BMS_MAX_IGNORED_DISCONNECTED_PROBES = 2;

inline void sanitizeCellProbeTemps(
    const float temps[BMS_CELL_PROBE_COUNT],
    float out[BMS_CELL_PROBE_COUNT]) {
  uint8_t disconnectedCount = 0;

  // First pass: count disconnected probes (NaN from library)
  for (uint8_t i = 0; i < BMS_CELL_PROBE_COUNT; i++) {
    if (isnan(temps[i])) disconnectedCount++;
  }

  // Second pass: if within tolerance, pass NaN through (silently ignored).
  // If too many are disconnected, replace excess NaN with the sentinel value
  // so temperature monitors fire a CRIT_LOW alert.
  uint8_t ignoredCount = 0;
  for (uint8_t i = 0; i < BMS_CELL_PROBE_COUNT; i++) {
    if (!isnan(temps[i])) {
      out[i] = temps[i];
    } else if (ignoredCount < BMS_MAX_IGNORED_DISCONNECTED_PROBES) {
      out[i] = NAN;
      ignoredCount++;
    } else {
      out[i] = BMS_CAN::TEMP_PROBE_DISCONNECTED;
    }
  }
}

// External declarations
extern STR_BMS_TELEMETRY_140 bmsTelemetryData;
extern BMS_CAN* bms_can;

// BMS functions
void updateBMSData();
void printBMSData();
