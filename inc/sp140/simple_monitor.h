#ifndef INC_SP140_SIMPLE_MONITOR_H_
#define INC_SP140_SIMPLE_MONITOR_H_

#include <Arduino.h>
#include <vector>
#include "sp140/structs.h"
#include "sp140/globals.h"

// Forward declarations
class ILogger;
class IMonitor;

// Sensor categories for organization
enum class SensorCategory {
  ESC,
  BMS, 
  ALTIMETER,
  INTERNAL
};

// Alert levels
enum class AlertLevel {
  OK = 0,
  WARN_LOW = 1,
  WARN_HIGH = 2, 
  CRIT_LOW = 3,
  CRIT_HIGH = 4,
  INFO = 5
};

// Sensor ID enumeration
enum class SensorID {
  // ESC Temperature sensors
  ESC_MOS_Temp,
  ESC_MCU_Temp,
  ESC_CAP_Temp,
  Motor_Temp,
  
  // ESC Running Errors (Critical)
  ESC_OverCurrent_Error,
  ESC_LockedRotor_Error,
  ESC_OverTemp_Error,
  ESC_OverVolt_Error,
  ESC_VoltageDrop_Error,
  
  // ESC Running Warnings
  ESC_ThrottleSat_Warning,
  
  // ESC Self-Check Errors
  ESC_MotorCurrentOut_Error,
  ESC_TotalCurrentOut_Error,
  ESC_MotorVoltageOut_Error,
  ESC_CapNTC_Error,
  ESC_MosNTC_Error,
  ESC_BusVoltRange_Error,
  ESC_BusVoltSample_Error,
  ESC_MotorZLow_Error,
  ESC_MotorZHigh_Error,
  ESC_MotorVDet1_Error,
  ESC_MotorVDet2_Error,
  ESC_MotorIDet2_Error,
  ESC_SwHwIncompat_Error,
  ESC_BootloaderBad_Error,

  // BMS sensors
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

// Logger interface
class ILogger {
public:
  virtual ~ILogger() = default;
  virtual void log(SensorID id, AlertLevel level, float value) = 0;
  virtual void log(SensorID id, AlertLevel level, bool value) = 0;
};

// Multi-logger that dispatches to multiple sinks
class MultiLogger : public ILogger {
private:
  std::vector<ILogger*> sinks;

public:
  void addSink(ILogger* sink);
  void log(SensorID id, AlertLevel level, float value) override;
  void log(SensorID id, AlertLevel level, bool value) override;
};

// Serial logger implementation
class SerialLogger : public ILogger {
public:
  void log(SensorID id, AlertLevel level, float value) override;
  void log(SensorID id, AlertLevel level, bool value) override;
};

// UI logger for alert events
class AlertUILogger : public ILogger {
public:
  void log(SensorID id, AlertLevel level, float value) override;
  void log(SensorID id, AlertLevel level, bool value) override;
};

// Monitor interface
class IMonitor {
public:
  virtual ~IMonitor() = default;
  virtual void check() = 0;
  virtual SensorID getSensorID() const = 0;
  virtual SensorCategory getCategory() const = 0;
  virtual void resetState() = 0;
};

// Global instances
extern MultiLogger multiLogger;
extern std::vector<IMonitor*> monitors;
extern SerialLogger serialLogger;
extern bool monitoringEnabled;

// Thread-safe copies for monitoring
extern STR_ESC_TELEMETRY_140 monitoringEscData;
extern STR_BMS_TELEMETRY_140 monitoringBmsData;

// Core functions
void initSimpleMonitor();
void checkAllSensors();
void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140& escData,
                             const STR_BMS_TELEMETRY_140& bmsData);
void enableMonitoring();
void sendAlertEvent(SensorID id, AlertLevel level);

// Utility functions
const char* sensorIDToString(SensorID id);
const char* sensorIDToAbbreviation(SensorID id);
const char* sensorIDToAbbreviationWithLevel(SensorID id, AlertLevel level);

// Monitor registration functions (to be implemented)
void addESCMonitors();
void addBMSMonitors(); 
void addAltimeterMonitors();
void addInternalMonitors();

#endif  // INC_SP140_SIMPLE_MONITOR_H_