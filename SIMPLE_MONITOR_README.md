# Sensor Monitoring System

## Overview
Real-time health monitoring for ESP32 electric ultralight sensors. Monitors ESC, BMS, altimeter, and system health with threshold-based alerting.

## Features
- **Comprehensive**: 50+ sensors monitored across all flight systems
- **Smart Alerting**: Hysteresis protection prevents false alerts
- **Multiple Outputs**: Serial, display, and extensible logging
- **Efficient**: Minimal overhead, change-only notifications

## What's Monitored

### ESC
- Temperatures (MOS, MCU, CAP, Motor)
- Error states (over-current, over-temp, voltage issues)
- Hardware faults and diagnostics

### BMS
- Cell voltages and temperature sensors
- State of charge and charge/discharge status
- Voltage differential monitoring

### System
- CPU temperature and altimeter readings
- System health and diagnostics

## How It Works

System automatically monitors all sensors and alerts on threshold violations. Hysteresis prevents false alerts from sensor noise.

### Alert Levels
- **OK**: Normal operation
- **WARN**: Advisory thresholds exceeded
- **CRIT**: Critical thresholds requiring attention

### Example Output
```
[15234] [WARN_HIGH] ESC_MOS_Temp = 92.50
[15467] [CRIT_HIGH] Motor_Temp = 112.30
[15890] [OK] ESC_MOS_Temp = 88.20
```

### Integration
- Runs every 40ms in FreeRTOS task
- Displays alerts on screen with haptic feedback
- Serial logging for diagnostics
- Thread-safe with minimal CPU overhead

The system provides comprehensive flight safety monitoring while maintaining performance for electric ultralight operation.
