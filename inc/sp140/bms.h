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

// How long the BMS link may go silent before it counts as disconnected. Must
// stay well clear of the bmsTask poll period (100 ms): with the library's 100 ms
// default there is zero timing margin, so a single skipped CAN drain — e.g. the
// SPI-mutex bail in updateBMSData() — publishes a spurious disconnect, which
// force-clears every BMS alert.
constexpr unsigned long BMS_LINK_TIMEOUT_MS = 500;

// A just-connected BMS spreads its state across several CAN frames (basic
// info 1: pack voltage/SOC/current, basic info 2: cell voltages, 0x18B4:
// temperatures). Treating the link as CONNECTED on the first frame let
// monitors and the UI see zero-initialized fields — a 0.000 V "highest cell"
// fired a critical displayed as cell-voltage-high at every boot. Only report
// CONNECTED once both basic frames have populated the snapshot.
//
// Deliberately tests HIGHEST cell voltage, not lowest: both are written by the
// same frame (0x18FE28F4), so either proves that frame arrived — but a shorted
// cell or broken sense lead drives the LOWEST cell to 0.000 V on a healthy,
// still-transmitting pack. Keying on lowest would read that catastrophic fault
// as "no data" and drop the whole BMS to NOT_CONNECTED, suppressing the very
// low-cell and voltage-differential alerts that exist to catch it. No
// single-cell fault can pull the highest cell of a live pack below this floor.
// Callers must latch the result (see bmsTask) so this stays a boot-ordering
// gate and never becomes a runtime state.
inline bool bmsSnapshotCoherent(const STR_BMS_TELEMETRY_140& t) {
  return t.battery_voltage > 5.0f &&     // basic info 1 received
         t.highest_cell_voltage > 0.5f;  // basic info 2 received
}

// True once at least one reading from the BMS temperature frame has been
// parsed. Before that frame arrives every probe reads NaN — indistinguishable
// from "all probes disconnected" — and the disconnect sentinel policy below
// must not run, or it fires -40 °C criticals for T3/T4 at every boot.
inline bool bmsTempFrameSeen(float mosTemp, float balanceTemp,
                             const float cellTemps[BMS_CELL_PROBE_COUNT]) {
  if (!isnan(mosTemp) || !isnan(balanceTemp)) return true;
  for (uint8_t i = 0; i < BMS_CELL_PROBE_COUNT; i++) {
    if (!isnan(cellTemps[i])) return true;
  }
  return false;
}

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
bool initBMSCAN(SPIClass* spi);
void updateBMSData();
void printBMSData();
