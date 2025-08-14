#include "sp140/esc_monitors.h"
#include "sp140/monitor_config.h"
#include "sp140/esc.h"  // For ESC error checking functions
#include <Arduino.h>

// External references to core monitoring infrastructure
extern std::vector<IMonitor*> monitors;
extern MultiLogger multiLogger;

// External references to thread-safe data copies
extern STR_ESC_TELEMETRY_140 monitoringEscData;

void addESCMonitors() {
  // ESC MOS Temperature Monitor
  static SensorMonitor* escMosTemp = new SensorMonitor(
    SensorID::ESC_MOS_Temp,
    escMosTempThresholds,
    []() { return monitoringEscData.mos_temp; },
    &multiLogger);
  monitors.push_back(escMosTemp);

  // ESC MCU Temperature Monitor
  static SensorMonitor* escMcuTemp = new SensorMonitor(
    SensorID::ESC_MCU_Temp,
    escMcuTempThresholds,
    []() { return monitoringEscData.mcu_temp; },
    &multiLogger);
  monitors.push_back(escMcuTemp);

  // ESC Capacitor Temperature Monitor
  static SensorMonitor* escCapTemp = new SensorMonitor(
    SensorID::ESC_CAP_Temp,
    escCapTempThresholds,
    []() { return monitoringEscData.cap_temp; },
    &multiLogger);
  monitors.push_back(escCapTemp);

  // Motor Temperature Monitor
  static SensorMonitor* motorTemp = new SensorMonitor(
    SensorID::Motor_Temp,
    motorTempThresholds,
    []() { return monitoringEscData.motor_temp; },
    &multiLogger);
  monitors.push_back(motorTemp);

  // Individual ESC Running Error Monitors (Critical)
  static BooleanMonitor* escOverCurrentError = new BooleanMonitor(
    SensorID::ESC_OverCurrent_Error,
    []() { return hasOverCurrentError(monitoringEscData.running_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escOverCurrentError);

  static BooleanMonitor* escLockedRotorError = new BooleanMonitor(
    SensorID::ESC_LockedRotor_Error,
    []() { return hasLockedRotorError(monitoringEscData.running_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escLockedRotorError);

  static BooleanMonitor* escOverTempError = new BooleanMonitor(
    SensorID::ESC_OverTemp_Error,
    []() { return hasOverTempError(monitoringEscData.running_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escOverTempError);

  static BooleanMonitor* escOverVoltError = new BooleanMonitor(
    SensorID::ESC_OverVolt_Error,
    []() { return hasOverVoltError(monitoringEscData.running_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escOverVoltError);

  static BooleanMonitor* escVoltageDropError = new BooleanMonitor(
    SensorID::ESC_VoltageDrop_Error,
    []() { return hasVoltagDropError(monitoringEscData.running_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escVoltageDropError);

  // Individual ESC Running Warning Monitors
  static BooleanMonitor* escThrottleSatWarning = new BooleanMonitor(
    SensorID::ESC_ThrottleSat_Warning,
    []() { return hasThrottleSatWarning(monitoringEscData.running_error); },
    true, AlertLevel::WARN_HIGH, &multiLogger);
  monitors.push_back(escThrottleSatWarning);

  // Individual ESC Self-Check Error Monitors (All Critical)
  static BooleanMonitor* escMotorCurrentOutError = new BooleanMonitor(
    SensorID::ESC_MotorCurrentOut_Error,
    []() { return hasMotorCurrentOutError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorCurrentOutError);

  static BooleanMonitor* escTotalCurrentOutError = new BooleanMonitor(
    SensorID::ESC_TotalCurrentOut_Error,
    []() { return hasTotalCurrentOutError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escTotalCurrentOutError);

  static BooleanMonitor* escMotorVoltageOutError = new BooleanMonitor(
    SensorID::ESC_MotorVoltageOut_Error,
    []() { return hasMotorVoltageOutError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorVoltageOutError);

  static BooleanMonitor* escCapNTCError = new BooleanMonitor(
    SensorID::ESC_CapNTC_Error,
    []() { return hasCapNTCError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escCapNTCError);

  static BooleanMonitor* escMosNTCError = new BooleanMonitor(
    SensorID::ESC_MosNTC_Error,
    []() { return hasMosNTCError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMosNTCError);

  static BooleanMonitor* escBusVoltRangeError = new BooleanMonitor(
    SensorID::ESC_BusVoltRange_Error,
    []() { return hasBusVoltRangeError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escBusVoltRangeError);

  static BooleanMonitor* escBusVoltSampleError = new BooleanMonitor(
    SensorID::ESC_BusVoltSample_Error,
    []() { return hasBusVoltSampleError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escBusVoltSampleError);

  static BooleanMonitor* escMotorZLowError = new BooleanMonitor(
    SensorID::ESC_MotorZLow_Error,
    []() { return hasMotorZLowError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorZLowError);

  static BooleanMonitor* escMotorZHighError = new BooleanMonitor(
    SensorID::ESC_MotorZHigh_Error,
    []() { return hasMotorZHighError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorZHighError);

  static BooleanMonitor* escMotorVDet1Error = new BooleanMonitor(
    SensorID::ESC_MotorVDet1_Error,
    []() { return hasMotorVDet1Error(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorVDet1Error);

  static BooleanMonitor* escMotorVDet2Error = new BooleanMonitor(
    SensorID::ESC_MotorVDet2_Error,
    []() { return hasMotorVDet2Error(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorVDet2Error);

  static BooleanMonitor* escMotorIDet2Error = new BooleanMonitor(
    SensorID::ESC_MotorIDet2_Error,
    []() { return hasMotorIDet2Error(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escMotorIDet2Error);

  static BooleanMonitor* escSwHwIncompatError = new BooleanMonitor(
    SensorID::ESC_SwHwIncompat_Error,
    []() { return hasSwHwIncompatError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escSwHwIncompatError);

  static BooleanMonitor* escBootloaderBadError = new BooleanMonitor(
    SensorID::ESC_BootloaderBad_Error,
    []() { return hasBootloaderBadError(monitoringEscData.selfcheck_error); },
    true, AlertLevel::CRIT_HIGH, &multiLogger);
  monitors.push_back(escBootloaderBadError);
}
