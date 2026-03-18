#ifndef INC_SP140_SIMPLE_MONITOR_H_
#define INC_SP140_SIMPLE_MONITOR_H_

#include <vector>
#include <functional>
#include <math.h>
#include <Arduino.h>
#include "sp140/structs.h"

// Sensor categories for grouped handling
enum class SensorCategory {
  ESC,        // ESC-related sensors (temperatures, errors)
  BMS,        // BMS-related sensors (temperatures, voltages, etc.)
  ALTIMETER,  // Barometric sensors
  INTERNAL    // CPU and other internal sensors
};

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
  // ESC Self-Check Errors (All Critical - Boot-time hardware faults)
  ESC_MotorCurrentOut_Error,  // Bit 0: motor_i_out_bad
  ESC_TotalCurrentOut_Error,  // Bit 1: total_i_out_bad
  ESC_MotorVoltageOut_Error,  // Bit 2: motor_v_out_bad
  ESC_CapNTC_Error,           // Bit 3: cap_ntc_bad
  ESC_MosNTC_Error,           // Bit 4: mos_ntc_bad
  ESC_BusVoltRange_Error,     // Bit 5: bus_v_range
  ESC_BusVoltSample_Error,    // Bit 6: bus_v_sample_bad
  ESC_MotorZLow_Error,        // Bit 7: motor_z_too_low
  ESC_MotorZHigh_Error,       // Bit 8: motor_z_too_high
  ESC_MotorVDet1_Error,       // Bit 9: motor_v_det1_bad
  ESC_MotorVDet2_Error,       // Bit 10: motor_v_det2_bad
  ESC_MotorIDet2_Error,       // Bit 11: motor_i_det2_bad
  ESC_SwHwIncompat_Error,     // Bit 13: sw_hw_incompatible
  ESC_BootloaderBad_Error,    // Bit 14: bootloader_unsupported
  ESC_TWAI_Init_Failure,      // TWAI/CAN bus initialization failure

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
  BMS_CAN_Init_Failure,

  // Altimeter
  Baro_Temp,
  Baro_Init_Failure,

  // Internal
  CPU_Temp
};

// Alert levels
enum class AlertLevel { OK, WARN_LOW, WARN_HIGH, CRIT_LOW, CRIT_HIGH, INFO };

// Threshold set with hysteresis support
struct Thresholds {
  float warnLow, warnHigh;
  float critLow, critHigh;

  // Hysteresis value - must be positive
  // This defines the deadband around all thresholds to prevent bouncing
  float hysteresis;
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
  void addSink(ILogger* s) {
    if (s) sinks.push_back(s);
  }
  void log(SensorID id, AlertLevel lvl, float v) override {
    for (auto* s : sinks) {
      s->log(id, lvl, v);
    }
  }
  void log(SensorID id, AlertLevel lvl, bool v) override {
    for (auto* s : sinks) {
      s->log(id, lvl, v);
    }
  }
};

extern MultiLogger multiLogger;  // Global fan-out logger

// Base monitor interface
struct IMonitor {
  virtual ~IMonitor() = default;
  virtual void check() = 0;
  virtual SensorID getSensorID() const = 0;
  virtual SensorCategory getCategory() const = 0;  // Each monitor knows its category
  virtual void resetState() = 0;  // Reset internal alert state to OK
};

// Sensor monitor for analog values
struct SensorMonitor : public IMonitor {
  SensorID id;
  SensorCategory category;
  Thresholds thr;
  std::function<float()> read;
  AlertLevel last;
  ILogger* logger;

  // Constructor - now takes category explicitly
  SensorMonitor(SensorID i, SensorCategory cat, Thresholds t, std::function<float()> r, ILogger* l)
    : id(i), category(cat), thr(t), read(r), last(AlertLevel::OK), logger(l) {}

  void check() override {
    float v = read();
    if (isnan(v)) {
      if (last != AlertLevel::OK) {
        logger->log(id, AlertLevel::OK, v);
        last = AlertLevel::OK;
      }
      return;
    }

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

  SensorID getSensorID() const override {
    return id;
  }

  SensorCategory getCategory() const override {
    return category;
  }

  void resetState() override {
    last = AlertLevel::OK;
  }
};

// Enhanced sensor monitor with hysteresis to prevent alert bouncing
struct HysteresisSensorMonitor : public SensorMonitor {
  // Constructor - inherits all member variables from SensorMonitor
  HysteresisSensorMonitor(SensorID i, SensorCategory cat, Thresholds t, std::function<float()> r, ILogger* l)
    : SensorMonitor(i, cat, t, r, l) {}

  void check() override {
    float v = read();
    if (isnan(v)) {
      if (last != AlertLevel::OK) {
        logger->log(id, AlertLevel::OK, v);
        last = AlertLevel::OK;
      }
      return;
    }

    AlertLevel now = last;  // Start with current state

    // Hysteresis logic to prevent bouncing
    switch (last) {
      case AlertLevel::OK:
        // From OK, we can transition to warnings/criticals
        if      (v <= thr.critLow)  now = AlertLevel::CRIT_LOW;
        else if (v <= thr.warnLow)  now = AlertLevel::WARN_LOW;
        else if (v >= thr.critHigh) now = AlertLevel::CRIT_HIGH;
        else if (v >= thr.warnHigh) now = AlertLevel::WARN_HIGH;
        break;

      case AlertLevel::WARN_LOW:
        // From WARN_LOW, escalate to critical at exact threshold (no hysteresis)
        // Use hysteresis only when clearing (de-escalating to OK)
        if (v <= thr.critLow) now = AlertLevel::CRIT_LOW;
        else if (v > thr.warnLow + thr.hysteresis) now = AlertLevel::OK;
        break;

      case AlertLevel::WARN_HIGH:
        // From WARN_HIGH, escalate to critical at exact threshold (no hysteresis)
        // Use hysteresis only when clearing (de-escalating to OK)
        if (v >= thr.critHigh) now = AlertLevel::CRIT_HIGH;
        else if (v < thr.warnHigh - thr.hysteresis) now = AlertLevel::OK;
        break;

      case AlertLevel::CRIT_LOW:
        // From CRIT_LOW, need to go above critLow + hysteresis to de-escalate
        if (v > thr.critLow + thr.hysteresis) {
          if (v <= thr.warnLow + thr.hysteresis) now = AlertLevel::WARN_LOW;
          else now = AlertLevel::OK;
        }
        break;

      case AlertLevel::CRIT_HIGH:
        // From CRIT_HIGH, need to go below critHigh - hysteresis to de-escalate
        if (v < thr.critHigh - thr.hysteresis) {
          if (v >= thr.warnHigh - thr.hysteresis) now = AlertLevel::WARN_HIGH;
          else now = AlertLevel::OK;
        }
        break;

      default:
        break;
    }

    if (now != last) {
      logger->log(id, now, v);
      last = now;
    }
  }
};

// New monitor for boolean conditions
struct BooleanMonitor : public IMonitor {
    SensorID id;
    SensorCategory category;
    std::function<bool()> read;
    bool alertOnTrue;  // true to alert on true, false for false
    AlertLevel level;  // level to report when condition is met
    bool lastState;
    ILogger* logger;

    // Constructor - now takes category explicitly
    BooleanMonitor(SensorID i, SensorCategory cat, std::function<bool()> r, bool alertOn, AlertLevel alertLevel, ILogger* l)
        : id(i), category(cat), read(r), alertOnTrue(alertOn), level(alertLevel), lastState(!alertOn), logger(l) {}

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

    SensorID getSensorID() const override {
        return id;
    }

    SensorCategory getCategory() const override {
        return category;
    }

    void resetState() override {
        lastState = !alertOnTrue;  // Reset to the non-alerting state
    }
};

// Global monitor registry
extern std::vector<IMonitor*> monitors;
extern SerialLogger serialLogger;
extern bool monitoringEnabled;  // Flag to control monitoring state

// Functions
const char* sensorIDToString(SensorID id);
const char* sensorIDToAbbreviation(SensorID id);
const char* sensorIDToAbbreviationWithLevel(SensorID id, AlertLevel level);
SensorCategory getSensorCategory(SensorID id);  // Get category for a sensor
void initSimpleMonitor();
void checkAllSensors();
void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140& escData, const STR_BMS_TELEMETRY_140& bmsData);
void addESCMonitors();
void addBMSMonitors();
void addAltimeterMonitors();
void addInternalMonitors();
void enableMonitoring();

#endif  // INC_SP140_SIMPLE_MONITOR_H_
