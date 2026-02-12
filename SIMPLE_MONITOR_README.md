# Simple Sensor Monitoring System

## Overview
A lightweight, category-based monitoring system for ESP32 electric ultralight sensors. Features intelligent alert suppression when controllers disconnect and modern OOP design with device categories.

## Design Philosophy
- **Safety-First**: Automatically suppresses ESC alerts when ESC disconnected, BMS alerts when BMS disconnected
- **Category-Based**: Organizes sensors by device type (ESC, BMS, ALTIMETER, INTERNAL) for grouped management
- **Modern OOP**: Interface-based design with virtual methods and clean inheritance
- **Thread-Safe**: Queue-based telemetry snapshots for safe multi-task operation

## Usage

### Device Categories & Alert Suppression
```cpp
enum class SensorCategory {
  ESC,        // ESC temps, motor temp, ESC errors
  BMS,        // Battery voltage, current, temps, SOC
  ALTIMETER,  // Barometric sensors
  INTERNAL    // CPU temp, system sensors
};

// When ESC disconnects: All ESC alerts automatically cleared
// When BMS disconnects: All BMS alerts automatically cleared
// ALTIMETER/INTERNAL: Always monitored (no connection dependency)
```

### Current Monitoring Coverage
```cpp
// ESC Category (suppressed when ESC disconnected):
// - ESC MOS: Warning 90°C, Critical 110°C
// - ESC MCU: Warning 80°C, Critical 95°C
// - ESC CAP: Warning 85°C, Critical 100°C
// - Motor: Warning 90°C, Critical 110°C
// - ESC Error Conditions (overcurrent, overtemp, etc.)

// BMS Category (suppressed when BMS disconnected):
// - Battery voltage, current, SOC, cell voltages
// - BMS temperatures, charge/discharge MOS status
```

### Adding New Sensors
```cpp
// Example: New ESC sensor
static SensorMonitor* newEscSensor = new SensorMonitor(
  SensorID::ESC_New_Sensor,      // Unique ID from enum
  SensorCategory::ESC,           // Category determines suppression behavior
  {.warnLow = 10, .warnHigh = 80, .critLow = 5, .critHigh = 90},
  []() { return escTelemetryData.new_value; },  // Data source
  &multiLogger                   // Fan-out to multiple outputs
);
monitors.push_back(newEscSensor);

// Example: Boolean error monitor
static BooleanMonitor* newError = new BooleanMonitor(
  SensorID::ESC_New_Error,
  SensorCategory::ESC,
  []() { return checkErrorCondition(); },
  true,                          // Alert when condition is true
  AlertLevel::CRIT_HIGH,
  &multiLogger
);
monitors.push_back(newError);
```

### Example Output
```
[15234] [WARN_HIGH] ESC_MOS_Temp = 92.50
[15467] [CRIT_HIGH] Motor_Temp = 112.30
[15890] [OK] ESC_MOS_Temp = 88.20
[16123] ESC disconnected - clearing all ESC alerts
[16450] [WARN_LOW] BMS_SOC = 12.3
```

## Easy Extensions

### SD Card Logging
```cpp
struct SDLogger : ILogger {
  void log(SensorID id, AlertLevel lvl, float v) override {
    File logFile = SD.open("/alerts.log", FILE_APPEND);
    logFile.printf("%lu,%s,%d,%.2f\n", millis(), sensorIDToString(id), (int)lvl, v);
    logFile.close();
  }
};
```

### Custom Alert Processing
```cpp
struct CustomLogger : ILogger {
  void log(SensorID id, AlertLevel lvl, float v) override {
    // Custom processing based on sensor category
    SensorCategory cat = getSensorCategory(id);
    if (cat == SensorCategory::ESC && lvl >= AlertLevel::CRIT_HIGH) {
      triggerEmergencyShutdown();
    }
  }
};
```

### Multiple Outputs (Built-in)
```cpp
// MultiLogger automatically fans out to all registered sinks
multiLogger.addSink(&serialLogger);  // Console output
multiLogger.addSink(&uiLogger);      // LVGL alerts
multiLogger.addSink(&customLogger);  // Your custom handler

// All monitors automatically use multiLogger
```

## Architecture

```cpp
IMonitor (interface)
├── virtual SensorID getSensorID() = 0
├── virtual SensorCategory getCategory() = 0
└── virtual void check() = 0

SensorMonitor : IMonitor              BooleanMonitor : IMonitor
├── SensorID id                       ├── SensorID id
├── SensorCategory category           ├── SensorCategory category
├── Thresholds thr                    ├── std::function<bool()> read
├── std::function<float()> read       ├── bool alertOnTrue
└── ILogger* logger                   └── AlertLevel level

ILogger (interface)
├── SerialLogger (debug output)
├── AlertUILogger (LVGL display)
└── MultiLogger (fan-out to multiple sinks)

Categories & Connection Logic:
├── ESC sensors → suppressed when escState != CONNECTED
├── BMS sensors → suppressed when bmsState != CONNECTED
├── ALTIMETER sensors → always active
└── INTERNAL sensors → always active
```

## Integration
- **Queue-Based**: Dedicated monitoring task receives telemetry snapshots via FreeRTOS queue
- **Thread-Safe**: No direct access to volatile telemetry data from monitoring task
- **Connection-Aware**: Automatically clears alerts when devices disconnect
- **Smart Suppression**: Only runs ESC monitors when ESC connected, BMS monitors when BMS connected
- **Change Detection**: Only logs when alert level changes (no spam)
- **Zero Overhead**: Minimal CPU usage when all sensors in OK state

## Memory Usage
- **Code Size**: ~3KB total (includes OOP infrastructure, queue handling, UI integration)
- **Per Sensor**: ~150 bytes (SensorMonitor) or ~120 bytes (BooleanMonitor)
- **Static Allocation**: All monitors allocated at compile-time (embedded-friendly)
- **Queue Memory**: ~200 bytes for telemetry snapshot queue
- **Category Lookup**: O(1) via virtual methods (no external mapping tables)

## Key Benefits
1. **Safety**: ESC alerts disappear when ESC disconnects (no false warnings during connection issues)
2. **Maintainability**: Add new sensors by category, automatic suppression behavior
3. **Robustness**: Thread-safe design prevents race conditions between telemetry and monitoring
4. **Extensibility**: Clean OOP interfaces for new monitor types and output methods
5. **Performance**: Smart suppression reduces unnecessary checks when devices offline

This design balances safety, maintainability, and performance - essential for flight-critical embedded systems.
