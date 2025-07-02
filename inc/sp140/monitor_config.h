#ifndef INC_SP140_MONITOR_CONFIG_H_
#define INC_SP140_MONITOR_CONFIG_H_

#include "simple_monitor.h"

// This file serves as the single source of truth for all sensor monitoring thresholds.

// -- ESC Thresholds --
static const Thresholds escMosTempThresholds = {.warnLow = -10, .warnHigh = 90, .critLow = -20, .critHigh = 110};
static const Thresholds escMcuTempThresholds = {.warnLow = -10, .warnHigh = 80, .critLow = -20, .critHigh = 95};
static const Thresholds escCapTempThresholds = {.warnLow = -10, .warnHigh = 85, .critLow = -20, .critHigh = 100};
static const Thresholds motorTempThresholds = {.warnLow = -20, .warnHigh = 90, .critLow = -25, .critHigh = 110};

// -- BMS Thresholds --
static const Thresholds bmsTempThresholds = {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 60};
static const Thresholds bmsCellTempThresholds = {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 56};
static const Thresholds bmsHighCellVoltageThresholds = {.warnLow = 0.0, .warnHigh = 4.1, .critLow = 0.0, .critHigh = 4.2};
static const Thresholds bmsLowCellVoltageThresholds = {.warnLow = 3.2, .warnHigh = 5.0, .critLow = 3.0, .critHigh = 5.0};
static const Thresholds bmsSOCThresholds = {.warnLow = 15.0, .warnHigh = 101.0, .critLow = 5.0, .critHigh = 110.0};
static const Thresholds bmsTotalVoltageHighThresholds = {.warnLow = 0.0, .warnHigh = 100.4, .critLow = 0.0, .critHigh = 100.8};
static const Thresholds bmsVoltageDifferentialThresholds = {.warnLow = -1.0, .warnHigh = 0.2, .critLow = -2.0, .critHigh = 0.4};

// -- Altimeter Thresholds --
static const Thresholds baroTempThresholds = {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80};

// -- Internal Thresholds --
static const Thresholds cpuTempThresholds = {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80};

#endif  // INC_SP140_MONITOR_CONFIG_H_
