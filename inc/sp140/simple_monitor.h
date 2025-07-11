#ifndef INC_SP140_SIMPLE_MONITOR_H_
#define INC_SP140_SIMPLE_MONITOR_H_

#include <vector>
#include <functional>
#include <Arduino.h>
#include "sp140/structs.h"

// Unique identifiers for each sensor
enum class SensorID {
  // ESC
  ESC_MOS_Temp,
  ESC_MCU_Temp,
  ESC_CAP_Temp,
  Motor_Temp,
  // ESC Running Errors (Critical - High Priority)
  ESC_OverCurrent_Error,      // Bit 0: over_current_protect
  ESC_LockedRotor_Error,      // Bit 1: locked_rotor_protect
  ESC_OverTemp_Error,         // Bit 2: over_temp_protect
  ESC_OverVolt_Error,         // Bit 6: over_volt_protect
  ESC_VoltageDrop_Error,      // Bit 7: voltage_drop
  // ESC Running Warnings (Middle Priority)
  ESC_ThrottleSat_Warning,    // Bit 5: throttle_saturation
  // ESC Self-Check Errors (All Critical)
  ESC_SelfCheck_Error,

  // BMS
  BMS_MOS_Temp,
  BMS_Balance_Temp,
  BMS_T1_Temp,
  BMS_T2_Temp,
  BMS_T3_Temp,
  BMS_T4_Temp,
  BMS_High_Cell_Voltage,
  BMS_Low_Cell_Voltage,
  BMS_SOC,
  BMS_Total_Voltage,
  BMS_Voltage_Differential,
  BMS_Charge_MOS,
  BMS_Discharge_MOS,

  // Altimeter
  Baro_Temp,

  // Internal
  CPU_Temp
};

// Alert levels
enum class AlertLevel { OK, WARN_LOW, WARN_HIGH, CRIT_LOW, CRIT_HIGH, INFO };

// Threshold set
struct Thresholds {
  float warnLow, warnHigh;
  float critLow, critHigh;
};

// Logger interface
struct ILogger {
  virtual void log(SensorID id, AlertLevel lvl, float val) = 0;
  virtual void log(SensorID id, AlertLevel lvl, bool val) = 0;
  virtual ~ILogger() = default;
};

// Telemetry snapshot for thread-safe handoff
struct TelemetrySnapshot {
  STR_ESC_TELEMETRY_140 esc;
  STR_BMS_TELEMETRY_140 bms;
};

extern QueueHandle_t telemetrySnapshotQueue;

// Serial logger implementation
struct SerialLogger : ILogger {
  void log(SensorID id, AlertLevel lvl, float v) override;
  void log(SensorID id, AlertLevel lvl, bool v) override;
};

// Multi-sink logger – forwards events to every registered ILogger sink
struct MultiLogger : ILogger {
  std::vector<ILogger*> sinks;
  void addSink(ILogger* s) { if (s) sinks.push_back(s); }
  void log(SensorID id, AlertLevel lvl, float v) override {
    for (auto* s : sinks) { s->log(id, lvl, v); }
  }
  void log(SensorID id, AlertLevel lvl, bool v) override {
    for (auto* s : sinks) { s->log(id, lvl, v); }
  }
};

extern MultiLogger multiLogger;  // Global fan-out logger

// Base monitor interface
struct IMonitor {
  virtual ~IMonitor() = default;
  virtual void check() = 0;
};

// Sensor monitor for analog values
struct SensorMonitor : public IMonitor {
  SensorID id;
  Thresholds thr;
  std::function<float()> read;
  AlertLevel last;
  ILogger* logger;

  // Constructor
  SensorMonitor(SensorID i, Thresholds t, std::function<float()> r, ILogger* l)
    : id(i), thr(t), read(r), last(AlertLevel::OK), logger(l) {}

  void check() override {
    float v = read();
    AlertLevel now = AlertLevel::OK;

    if      (v <= thr.critLow)  now = AlertLevel::CRIT_LOW;
    else if (v <= thr.warnLow)  now = AlertLevel::WARN_LOW;
    else if (v >= thr.critHigh) now = AlertLevel::CRIT_HIGH;
    else if (v >= thr.warnHigh) now = AlertLevel::WARN_HIGH;

    if (now != last) {
      logger->log(id, now, v);
      last = now;
    }
  }
};

// New monitor for boolean conditions
struct BooleanMonitor : public IMonitor {
    SensorID id;
    std::function<bool()> read;
    bool alertOnTrue;  // true to alert on true, false for false
    AlertLevel level;  // level to report when condition is met
    bool lastState;
    ILogger* logger;

    // Constructor
    BooleanMonitor(SensorID i, std::function<bool()> r, bool alertOn, AlertLevel alertLevel, ILogger* l)
        : id(i), read(r), alertOnTrue(alertOn), level(alertLevel), lastState(!alertOn), logger(l) {}

    void check() override {
        bool currentState = read();
        if (currentState != lastState) {
            if (currentState == alertOnTrue) {
                logger->log(id, level, currentState);
            } else {
                logger->log(id, AlertLevel::OK, currentState);
            }
            lastState = currentState;
        }
    }
};

// Global monitor registry
extern std::vector<IMonitor*> monitors;
extern SerialLogger serialLogger;
extern bool monitoringEnabled;  // Flag to control monitoring state

// Functions
const char* sensorIDToString(SensorID id);
const char* sensorIDToAbbreviation(SensorID id);
void initSimpleMonitor();
void checkAllSensors();
void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140& escData, const STR_BMS_TELEMETRY_140& bmsData);
void addESCMonitors();
void addBMSMonitors();
void addAltimeterMonitors();
void addInternalMonitors();
void enableMonitoring();

#endif  // INC_SP140_SIMPLE_MONITOR_H_
