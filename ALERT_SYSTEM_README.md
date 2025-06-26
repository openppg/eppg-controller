# ESP32 Electric Ultralight Alert System

## Overview

This flexible alert system provides comprehensive monitoring and logging capabilities for the ESP32 electric ultralight project. It can monitor various sensors, detect threshold violations, and output alerts through multiple channels.

## Features

- **Flexible Sensor Support**: Monitor temperature, voltage, current, and communication status
- **Multiple Alert Levels**: INFO, WARNING, CRITICAL, and FATAL severity levels
- **Configurable Thresholds**: Set low/high warning and critical thresholds per sensor
- **Multiple Output Channels**: Currently supports serial console, extensible to SD card, display, network
- **Alert Management**: Track active alerts, acknowledge alerts, maintain history
- **Individual Cell Monitoring**: Support for up to 24 individual battery cells
- **Real-time Processing**: Non-blocking alert checking integrated into main telemetry loop

## Architecture

### Core Components

1. **AlertSystem Class**: Main controller that manages thresholds, alerts, and outputs
2. **SensorID Enum**: Unique identifiers for all monitored sensors (ESC, BMS, individual cells)
3. **AlertRecord Structure**: Contains alert details including timestamp, severity, and message
4. **AlertOutput Interface**: Pluggable output system for different alert destinations

### Sensor Categories

- **ESC Sensors**: MOS, MCU, Capacitor, Motor temperatures; Voltage, Current, Power
- **BMS Sensors**: High/Low temperatures, Voltage, Current, SOC, Cell voltages (individual)
- **Communication**: ESC and BMS connection status
- **System**: Altimeter, Vibration, LED, Display status

## Usage

### Basic Setup

```cpp
#include "sp140/alert_system.h"

void setup() {
    // Initialize the alert system with serial output
    initAlertSystem();
}

void loop() {
    // Update the alert system (call regularly)
    alertSystem.update();
}
```

### Monitoring Sensor Values

```cpp
// Check a sensor value against configured thresholds
alertSystem.checkValue(SensorID::MOTOR_TEMP, AlertCategory::TEMPERATURE,
                      temperature_reading, "Motor Temperature");

// Manually raise an alert
alertSystem.raiseAlert(SensorID::ESC_COMM, AlertCategory::COMMUNICATION,
                      AlertSeverity::CRITICAL, 0, 0, "ESC Communication Lost");

// Log a general alert message
logAlert(AlertSeverity::WARNING, AlertCategory::SYSTEM, "Low memory warning");
```

### Custom Thresholds

```cpp
// Configure custom thresholds for a sensor
SensorThresholds customThresh = {
    .min_value = -20,      // Minimum acceptable value
    .max_value = 120,      // Maximum acceptable value
    .warning_low = 10,     // Low warning threshold
    .warning_high = 90,    // High warning threshold
    .critical_low = 5,     // Low critical threshold
    .critical_high = 110,  // High critical threshold
    .enabled = true        // Enable monitoring
};

alertSystem.setThresholds(SensorID::MOTOR_TEMP, customThresh);
```

### Default Thresholds

The system comes with preconfigured thresholds based on typical operating ranges:

#### ESC Temperature Thresholds
- **MOS**: Warning 75-90°C, Critical 30-110°C
- **MCU**: Warning 65-80°C, Critical 20-95°C
- **Capacitor**: Warning 70-85°C, Critical 25-100°C
- **Motor**: Warning 75-90°C, Critical 30-110°C

#### BMS Thresholds
- **Battery Voltage**: Warning 70-105V, Critical 65-110V
- **Cell Voltage**: Warning 3.0-4.2V, Critical 2.8-4.3V
- **Cell Differential**: Warning >0.5V, Critical >1.0V
- **State of Charge**: Warning <10%, Critical <5%

## Integration Points

### ESC Telemetry Integration
```cpp
void handleThrottle() {
    readESCTelemetry();
    checkESCTelemetry();  // Alert system integration
    syncESCTelemetry();
}
```

### BMS Telemetry Integration
```cpp
void spiCommTask() {
    updateBMSData();
    checkBMSTelemetry();  // Alert system integration
}
```

## Output Channels

### Current: Serial Console
All alerts are logged to USB serial with format:
```
[TIMESTAMP] [SEVERITY] [CATEGORY] SENSOR: Message (Value: X, Threshold: Y)
```

Example output:
```
[15234] [WARNING] [TEMP] MOTOR_TEMP: Motor Temperature value 95.00 exceeds warning threshold (Value: 95.00, Threshold: 90.00)
[15467] [CRITICAL] [COMM] ESC_COMM: ESC Communication Lost (Value: 0.00, Threshold: 0.00)
```

### Future Extensions

The modular design allows easy addition of new output channels:

```cpp
// SD Card Output (future implementation)
class SDCardAlertOutput : public AlertOutput {
public:
    void init() override {
        // Initialize SD card
    }

    void outputAlert(const AlertRecord& alert) override {
        // Write to SD card log file
    }
};

// Display Output (future implementation)
class DisplayAlertOutput : public AlertOutput {
public:
    void outputAlert(const AlertRecord& alert) override {
        // Show alert on LVGL display
    }
};

// Network Output (future implementation)
class NetworkAlertOutput : public AlertOutput {
public:
    void outputAlert(const AlertRecord& alert) override {
        // Send alert via WiFi/BLE
    }
};
```

## Alert Management

### Query Active Alerts
```cpp
size_t totalAlerts = alertSystem.getActiveAlertCount();
size_t criticalAlerts = alertSystem.getActiveAlertCount(AlertSeverity::CRITICAL);
size_t tempAlerts = alertSystem.getActiveAlertCount(AlertCategory::TEMPERATURE);

const auto& alerts = alertSystem.getActiveAlerts();
for (const auto& alert : alerts) {
    // Process each active alert
}
```

### Acknowledge Alerts
```cpp
// Acknowledge specific alert by index
alertSystem.acknowledgeAlert(0);

// Acknowledge all alerts
alertSystem.acknowledgeAllAlerts();

// Clear all alerts
alertSystem.clearAllAlerts();
```

### Alert History
```cpp
const auto& history = alertSystem.getAlertHistory();
// Review up to 100 most recent alerts
```

## Configuration

### Enable/Disable Sensors
```cpp
// Disable a specific sensor
alertSystem.enableSensor(SensorID::ESC_MOS_TEMP, false);

// Enable the entire system
alertSystem.enable(true);

// Disable the entire system
alertSystem.enable(false);
```

## Memory Usage

- **RAM**: ~6KB for alert storage (100 history + active alerts)
- **Flash**: ~8KB for alert system code
- **Per Alert**: ~150 bytes (including message buffer)

## Performance

- **Alert Checking**: <1ms per sensor
- **Threshold Evaluation**: O(1) lookup
- **Output Processing**: Depends on output channel (serial ~2ms, SD card ~10ms)
- **Update Frequency**: Called every 40ms in main SPI communication task

## Testing

Use the demonstration functions to test the alert system:

```cpp
demonstrateAlertSystem();     // Show various alert types
testAlertThresholds();        // Test threshold boundaries
printAlertSystemStatus();     // Display current system state
```

## Future Enhancements

1. **Persistent Storage**: Save alert history to EEPROM/flash
2. **Alert Hysteresis**: Prevent alert flapping near thresholds
3. **Alert Escalation**: Auto-escalate unacknowledged alerts
4. **Geofencing**: Location-based alert rules
5. **Predictive Alerts**: Trend analysis for early warnings
6. **Alert Filtering**: Rate limiting and duplicate suppression
7. **Emergency Actions**: Auto-shutdown on critical alerts
8. **Remote Monitoring**: Real-time alert streaming via WiFi/cellular

## Troubleshooting

### No Alerts Appearing
- Check `alertSystem.system_enabled_` is true
- Verify sensor thresholds are configured and enabled
- Ensure `alertSystem.update()` is being called regularly

### Too Many Alerts
- Adjust threshold values to be more appropriate for your system
- Implement alert rate limiting or filtering
- Use acknowledge functionality to manage alert noise

### Missing Sensor Data
- Verify telemetry data is being read correctly
- Check sensor communication status
- Enable debug output to trace data flow

## Code Structure

```
inc/sp140/alert_system.h      - Header with class definitions and enums
src/sp140/alert_system.cpp    - Main implementation
src/sp140/alert_demo.cpp      - Testing and demonstration functions
```

The alert system is designed to be modular, extensible, and efficient for real-time embedded applications while providing comprehensive monitoring capabilities for flight safety.
