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
  virtual void resetState() = 0;  // Reset internal state for reconnection scenarios
};

// Sensor monitor for analog values
struct SensorMonitor : public IMonitor {
  SensorID id;
  Thresholds thr;
  std::function<float()> read;
  AlertLevel last;
  ILogger* logger;
  
  // Hysteresis support to prevent alert spam
  static constexpr float DEFAULT_HYSTERESIS = 2.0f;  // Default 2°C/2V hysteresis
  float hysteresis;
  bool inHysteresisZone;  // Track if we're in a hysteresis zone
  AlertLevel lastStableLevel;  // Last stable alert level (outside hysteresis)

  // Constructor
  SensorMonitor(SensorID i, Thresholds t, std::function<float()> r, ILogger* l, float hyst = DEFAULT_HYSTERESIS)
    : id(i), thr(t), read(r), last(AlertLevel::OK), logger(l), 
      hysteresis(hyst), inHysteresisZone(false), lastStableLevel(AlertLevel::OK) {}

  void check() override {
    float v = read();
    
    // Check for sentinel value (-1.0f) indicating no data available
    if (v <= -1.0f) {
      // If we were previously in an alert state, clear it
      if (last != AlertLevel::OK) {
        logger->log(id, AlertLevel::OK, v);
        last = AlertLevel::OK;
        lastStableLevel = AlertLevel::OK;
        inHysteresisZone = false;
      }
      return;  // Skip processing for sentinel values
    }
    
    AlertLevel now = AlertLevel::OK;

    // Determine the alert level based on current value
    if      (v <= thr.critLow)  now = AlertLevel::CRIT_LOW;
    else if (v <= thr.warnLow)  now = AlertLevel::WARN_LOW;
    else if (v >= thr.critHigh) now = AlertLevel::CRIT_HIGH;
    else if (v >= thr.warnHigh) now = AlertLevel::WARN_HIGH;

    // Apply hysteresis logic
    bool shouldAlert = false;
    
    if (now != AlertLevel::OK) {
      // We're in an alert state - check if we should trigger
      if (last == AlertLevel::OK) {
        // First time entering alert - always trigger
        shouldAlert = true;
        lastStableLevel = now;
      } else if (now != last) {
        // Level changed - check if it's a significant change
        if (isSignificantLevelChange(now, last)) {
          shouldAlert = true;
          lastStableLevel = now;
        }
      }
    } else {
      // We're in OK state - check if we should clear the alert
      if (last != AlertLevel::OK) {
        // Check if we've moved far enough away from the threshold
        if (isClearOfThreshold(v)) {
          shouldAlert = true;
          lastStableLevel = AlertLevel::OK;
        }
      }
    }

    if (shouldAlert) {
      logger->log(id, now, v);
      last = now;
    }
  }
  
  void resetState() override {
    last = AlertLevel::OK;  // Reset to force state change detection
    inHysteresisZone = false;
    lastStableLevel = AlertLevel::OK;
  }

private:
  // Check if the level change is significant enough to trigger an alert
  bool isSignificantLevelChange(AlertLevel newLevel, AlertLevel oldLevel) {
    // Always trigger for critical changes
    if (newLevel == AlertLevel::CRIT_LOW || newLevel == AlertLevel::CRIT_HIGH) {
      return true;
    }
    if (oldLevel == AlertLevel::CRIT_LOW || oldLevel == AlertLevel::CRIT_HIGH) {
      return true;
    }
    
    // For warning levels, only trigger if moving between warn and OK
    // (not between warn_low and warn_high)
    if ((newLevel == AlertLevel::WARN_LOW || newLevel == AlertLevel::WARN_HIGH) &&
        oldLevel == AlertLevel::OK) {
      return true;
    }
    if (newLevel == AlertLevel::OK &&
        (oldLevel == AlertLevel::WARN_LOW || oldLevel == AlertLevel::WARN_HIGH)) {
      return true;
    }
    
    return false;
  }
  
  // Check if the value has moved far enough away from threshold to clear alert
  bool isClearOfThreshold(float value) {
    // For high thresholds (warnHigh, critHigh), check if we're below threshold - hysteresis
    if (last == AlertLevel::WARN_HIGH && value < (thr.warnHigh - hysteresis)) {
      return true;
    }
    if (last == AlertLevel::CRIT_HIGH && value < (thr.critHigh - hysteresis)) {
      return true;
    }
    
    // For low thresholds (warnLow, critLow), check if we're above threshold + hysteresis
    if (last == AlertLevel::WARN_LOW && value > (thr.warnLow + hysteresis)) {
      return true;
    }
    if (last == AlertLevel::CRIT_LOW && value > (thr.critLow + hysteresis)) {
      return true;
    }
    
    return false;
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
    
    void resetState() override {
        lastState = !alertOnTrue;  // Reset to force state change detection
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
void initSimpleMonitor();
void checkAllSensors();
void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140& escData, const STR_BMS_TELEMETRY_140& bmsData);
void addESCMonitors();
void addBMSMonitors();
void addAltimeterMonitors();
void addInternalMonitors();
void enableMonitoring();

// Function to reset monitor states (for reconnection scenarios)
void resetMonitorStates(bool bms, bool esc);

#endif  // INC_SP140_SIMPLE_MONITOR_H_
