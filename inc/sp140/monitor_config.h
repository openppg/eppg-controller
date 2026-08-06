#ifndef INC_SP140_MONITOR_CONFIG_H_
#define INC_SP140_MONITOR_CONFIG_H_

#include "simple_monitor.h"

// This file serves as the single source of truth for all sensor monitoring thresholds.

// -- ESC Thresholds --
static const Thresholds escMosTempThresholds = {.warnLow = -10, .warnHigh = 90, .critLow = -20, .critHigh = 110, .hysteresis = 1.0f};
static const Thresholds escMcuTempThresholds = {.warnLow = -10, .warnHigh = 80, .critLow = -20, .critHigh = 95, .hysteresis = 1.0f};
static const Thresholds escCapTempThresholds = {.warnLow = -10, .warnHigh = 85, .critLow = -20, .critHigh = 100, .hysteresis = 1.0f};
static const Thresholds motorTempThresholds = {.warnLow = -20, .warnHigh = 105, .critLow = -25, .critHigh = 115, .hysteresis = 1.0f};

// -- BMS Thresholds --
static const Thresholds bmsTempThresholds = {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 60, .hysteresis = 1.0f};
static const Thresholds bmsCellTempThresholds = {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 56, .hysteresis = 1.0f};
// High-cell monitor is high-side only: SensorMonitor fires at v <= critLow, so
// the old critLow of 0.0 turned a zero-initialized (pre-data) reading into a
// critical that displayed as "BC-CV-H". Low readings belong to the low monitor.
static const Thresholds bmsHighCellVoltageThresholds = {.warnLow = -1.0, .warnHigh = 4.19, .critLow = -1.0, .critHigh = 4.20};
static const Thresholds bmsLowCellVoltageThresholds = {.warnLow = 3.2, .warnHigh = 4.5, .critLow = 3.0, .critHigh = 4.8};
static const Thresholds bmsSOCThresholds = {.warnLow = 15.0, .warnHigh = 101.0, .critLow = 5.0, .critHigh = 110.0};
static const Thresholds bmsTotalVoltageThresholds = {.warnLow = 79.2, .warnHigh = 100.4, .critLow = 69.6, .critHigh = 100.8};
static const Thresholds bmsVoltageDifferentialThresholds = {.warnLow = -1.0, .warnHigh = 0.2, .critLow = -2.0, .critHigh = 0.4};

// Hold BMS monitors off for this long after the FIRST NOT_CONNECTED ->
// CONNECTED transition: a BMS that is itself still booting can emit
// sentinel/garbage readings in its first frames, and alarming on those caused
// spurious criticals at power-on and on hot-plug. Applied once per power-on —
// see checkAllSensorsWithData for why re-arming it per reconnect is unsafe.
static const uint32_t BMS_ALERT_GRACE_MS = 2000;

// -- Altimeter Thresholds --
static const Thresholds baroTempThresholds = {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80, .hysteresis = 1.0f};

// -- Internal Thresholds --
static const Thresholds cpuTempThresholds = {.warnLow = 0, .warnHigh = 60, .critLow = -10, .critHigh = 80, .hysteresis = 1.0f};

#endif  // INC_SP140_MONITOR_CONFIG_H_
