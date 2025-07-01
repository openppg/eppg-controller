#ifndef INC_SP140_SIMPLE_MONITOR_H_
#define INC_SP140_SIMPLE_MONITOR_H_

#include <vector>
#include <functional>
#include <Arduino.h>

// Unique identifiers for each sensor
enum class SensorID {
  // ESC
  ESC_MOS_Temp,
  ESC_MCU_Temp,
  ESC_CAP_Temp,
  Motor_Temp,

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

  // Altimeter
  Baro_Temp,

  // Internal
  CPU_Temp
};

// Alert levels
enum class AlertLevel { OK, WARN_LOW, WARN_HIGH, CRIT_LOW, CRIT_HIGH };

// Threshold set
struct Thresholds {
  float warnLow, warnHigh;
  float critLow, critHigh;
};

// Logger interface
struct ILogger {
  virtual void log(SensorID id, AlertLevel lvl, float val) = 0;
  virtual ~ILogger() = default;
};

// Serial logger implementation
struct SerialLogger : ILogger {
  void log(SensorID id, AlertLevel lvl, float v) override;
};

// Sensor monitor
struct SensorMonitor {
  SensorID id;
  Thresholds thr;
  std::function<float()> read;
  AlertLevel last;
  ILogger* logger;

  // Constructor
  SensorMonitor(SensorID i, Thresholds t, std::function<float()> r, ILogger* l)
    : id(i), thr(t), read(r), last(AlertLevel::OK), logger(l) {}

  void check() {
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

// Global monitor registry
extern std::vector<SensorMonitor*> sensors;
extern SerialLogger serialLogger;

// Functions
const char* sensorIDToString(SensorID id);
void initSimpleMonitor();
void checkAllSensors();
void addESCMonitors();
void addBMSMonitors();
void addAltimeterMonitors();
void addInternalMonitors();

#endif  // INC_SP140_SIMPLE_MONITOR_H_
