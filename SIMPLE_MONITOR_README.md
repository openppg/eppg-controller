# Simple Sensor Monitoring System

## Overview
A lightweight, elegant monitoring system for ESP32 electric ultralight sensors. Based on modern C++ patterns with easy extensibility.

## Design Philosophy
- **Simple**: ~50 lines of core code vs complex alerting systems
- **Flexible**: Easy to add new sensors and output methods
- **Modern**: Uses `std::function` and lambdas for clean sensor definitions
- **Extensible**: Interface-based design for multiple output channels

## Usage

### Current ESC Temperature Monitoring
```cpp
// Automatically monitors:
// - ESC MOS: Warning 90°C, Critical 110°C
// - ESC MCU: Warning 80°C, Critical 95°C
// - ESC CAP: Warning 85°C, Critical 100°C
// - Motor: Warning 90°C, Critical 110°C
```

### Adding New Sensors
```cpp
// Example: Battery voltage monitoring
static SensorMonitor batteryVoltage = {
  "BatteryVolt",
  {.warnLow = 70, .warnHigh = 105, .critLow = 65, .critHigh = 110},
  []() { return bmsTelemetryData.battery_voltage; },
  AlertLevel::OK,
  &serialLogger
};
sensors.push_back(&batteryVoltage);
```

### Example Output
```
[15234] [WARN_HIGH] ESC_MOS_Temp = 92.50
[15467] [CRIT_HIGH] Motor_Temp = 112.30
[15890] [OK] ESC_MOS_Temp = 88.20
```

## Easy Extensions

### SD Card Logging
```cpp
struct SDLogger : ILogger {
  void log(const char* name, AlertLevel lvl, float v) override {
    // Write timestamp, name, level, value to SD card
    File logFile = SD.open("/alerts.log", FILE_APPEND);
    logFile.printf("%lu,%s,%d,%.2f\n", millis(), name, (int)lvl, v);
    logFile.close();
  }
};
```

### Display Alerts
```cpp
struct DisplayLogger : ILogger {
  void log(const char* name, AlertLevel lvl, float v) override {
    // Show alert on LVGL display
    if (lvl >= AlertLevel::WARN_HIGH) {
      showAlertPopup(name, lvl, v);
    }
  }
};
```

### Multiple Outputs
```cpp
// Log to both serial and SD card
static SerialLogger serialLog;
static SDLogger sdLog;

sensorMonitor.logger = &serialLog;  // Or use both with composite pattern
```

## Architecture

```cpp
ILogger (interface)
├── SerialLogger (serial console)
├── SDLogger (SD card - future)
└── DisplayLogger (LVGL display - future)

SensorMonitor
├── name (string identifier)
├── thresholds (warn/crit levels)
├── read (lambda function)
└── logger (output interface)
```

## Integration
- Monitors check every 40ms in main SPI communication task
- Only logs when alert level changes (no spam)
- Uses existing telemetry data (no additional sensor reads)
- Zero overhead when sensors are in OK state

## Memory Usage
- ~1KB total code size
- ~200 bytes per monitored sensor
- Minimal RAM footprint
- No dynamic allocation during runtime

This approach perfectly balances simplicity with extensibility - start with basic serial logging, easily expand to SD cards, displays, or remote monitoring as needed.
