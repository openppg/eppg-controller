#include "sp140/simple_monitor.h"
#include "sp140/monitor_config.h"
#include "sp140/globals.h"
#include "sp140/altimeter.h"
#include "sp140/utilities.h"
#include "sp140/alert_display.h"  // UI logger & event queue
#include "sp140/esc.h"  // For ESC error checking functions

// Global instances
MultiLogger multiLogger;  // Defined here, declared extern in header
static AlertUILogger uiLogger;
std::vector<IMonitor*> monitors;
SerialLogger serialLogger;
bool monitoringEnabled = false;  // Start with monitoring disabled

// Thread-safe copies for monitoring (updated by checkAllSensorsWithData)
static STR_ESC_TELEMETRY_140 monitoringEscData = {};
static STR_BMS_TELEMETRY_140 monitoringBmsData = {};

// Ensure the multiLogger has all sinks registered exactly once
static void setupLoggerSinks() {
  static bool sinksInit = false;
  if (sinksInit) return;
  multiLogger.addSink(&serialLogger); // Serial output for debug
  multiLogger.addSink(&uiLogger);     // UI event sink
  sinksInit = true;
}

void SerialLogger::log(SensorID id, AlertLevel lvl, float v) {
  const char* levelNames[] = {"OK", "WARN_LOW", "WARN_HIGH", "CRIT_LOW", "CRIT_HIGH", "INFO"};
  USBSerial.printf("[%lu] [%s] %s = %.2f\n", millis(), levelNames[(int)lvl], sensorIDToString(id), v);
}

void SerialLogger::log(SensorID id, AlertLevel lvl, bool v) {
  const char* levelNames[] = {"OK", "WARN_LOW", "WARN_HIGH", "CRIT_LOW", "CRIT_HIGH", "INFO"};

            // Enhanced logging for ESC running errors - show decoded error details
  if (id == SensorID::ESC_OverCurrent_Error || id == SensorID::ESC_LockedRotor_Error ||
      id == SensorID::ESC_OverTemp_Error || id == SensorID::ESC_OverVolt_Error ||
      id == SensorID::ESC_VoltageDrop_Error || id == SensorID::ESC_ThrottleSat_Warning) {
    if (v) {
      USBSerial.printf("[%lu] [%s] %s = %s (0x%04X: %s)\n",
                       millis(), levelNames[(int)lvl], sensorIDToString(id),
                       v ? "ON" : "OFF", monitoringEscData.running_error,
                       decodeRunningError(monitoringEscData.running_error).c_str());
    } else {
      USBSerial.printf("[%lu] [%s] %s = %s (cleared)\n",
                       millis(), levelNames[(int)lvl], sensorIDToString(id),
                       v ? "ON" : "OFF");
    }
  // Enhanced logging for ESC self-check errors - show decoded error details
  } else if (id == SensorID::ESC_MotorCurrentOut_Error || id == SensorID::ESC_TotalCurrentOut_Error ||
             id == SensorID::ESC_MotorVoltageOut_Error || id == SensorID::ESC_CapNTC_Error ||
             id == SensorID::ESC_MosNTC_Error || id == SensorID::ESC_BusVoltRange_Error ||
             id == SensorID::ESC_BusVoltSample_Error || id == SensorID::ESC_MotorZLow_Error ||
             id == SensorID::ESC_MotorZHigh_Error || id == SensorID::ESC_MotorVDet1_Error ||
             id == SensorID::ESC_MotorVDet2_Error || id == SensorID::ESC_MotorIDet2_Error ||
             id == SensorID::ESC_SwHwIncompat_Error || id == SensorID::ESC_BootloaderBad_Error) {
    if (v) {
      USBSerial.printf("[%lu] [%s] %s = %s (0x%04X: %s)\n",
                       millis(), levelNames[(int)lvl], sensorIDToString(id),
                       v ? "ON" : "OFF", monitoringEscData.selfcheck_error,
                       decodeSelfCheckError(monitoringEscData.selfcheck_error).c_str());
    } else {
      USBSerial.printf("[%lu] [%s] %s = %s (cleared)\n",
                       millis(), levelNames[(int)lvl], sensorIDToString(id),
                       v ? "ON" : "OFF");
    }
    } else {
    // Standard logging for all other sensors
    USBSerial.printf("[%lu] [%s] %s = %s\n", millis(), levelNames[(int)lvl], sensorIDToString(id), v ? "ON" : "OFF");
  }
}

const char* sensorIDToString(SensorID id) {
  switch (id) {
    // ESC
    case SensorID::ESC_MOS_Temp: return "ESC_MOS_Temp";
    case SensorID::ESC_MCU_Temp: return "ESC_MCU_Temp";
    case SensorID::ESC_CAP_Temp: return "ESC_CAP_Temp";
    case SensorID::Motor_Temp: return "Motor_Temp";
    // ESC Running Errors (Critical)
    case SensorID::ESC_OverCurrent_Error: return "ESC_OverCurrent_Error";
    case SensorID::ESC_LockedRotor_Error: return "ESC_LockedRotor_Error";
    case SensorID::ESC_OverTemp_Error: return "ESC_OverTemp_Error";
    case SensorID::ESC_OverVolt_Error: return "ESC_OverVolt_Error";
    case SensorID::ESC_VoltageDrop_Error: return "ESC_VoltageDrop_Error";
    // ESC Running Warnings
    case SensorID::ESC_ThrottleSat_Warning: return "ESC_ThrottleSat_Warning";
    // ESC Self-Check Errors (Boot-time hardware faults)
    case SensorID::ESC_MotorCurrentOut_Error: return "ESC_MotorCurrentOut_Error";
    case SensorID::ESC_TotalCurrentOut_Error: return "ESC_TotalCurrentOut_Error";
    case SensorID::ESC_MotorVoltageOut_Error: return "ESC_MotorVoltageOut_Error";
    case SensorID::ESC_CapNTC_Error: return "ESC_CapNTC_Error";
    case SensorID::ESC_MosNTC_Error: return "ESC_MosNTC_Error";
    case SensorID::ESC_BusVoltRange_Error: return "ESC_BusVoltRange_Error";
    case SensorID::ESC_BusVoltSample_Error: return "ESC_BusVoltSample_Error";
    case SensorID::ESC_MotorZLow_Error: return "ESC_MotorZLow_Error";
    case SensorID::ESC_MotorZHigh_Error: return "ESC_MotorZHigh_Error";
    case SensorID::ESC_MotorVDet1_Error: return "ESC_MotorVDet1_Error";
    case SensorID::ESC_MotorVDet2_Error: return "ESC_MotorVDet2_Error";
    case SensorID::ESC_MotorIDet2_Error: return "ESC_MotorIDet2_Error";
    case SensorID::ESC_SwHwIncompat_Error: return "ESC_SwHwIncompat_Error";
    case SensorID::ESC_BootloaderBad_Error: return "ESC_BootloaderBad_Error";

    // BMS
    case SensorID::BMS_MOS_Temp: return "BMS_MOS_Temp";
    case SensorID::BMS_Balance_Temp: return "BMS_Balance_Temp";
    case SensorID::BMS_T1_Temp: return "BMS_T1_Temp";
    case SensorID::BMS_T2_Temp: return "BMS_T2_Temp";
    case SensorID::BMS_T3_Temp: return "BMS_T3_Temp";
    case SensorID::BMS_T4_Temp: return "BMS_T4_Temp";
    case SensorID::BMS_High_Cell_Voltage: return "BMS_High_Cell_Voltage";
    case SensorID::BMS_Low_Cell_Voltage: return "BMS_Low_Cell_Voltage";
    case SensorID::BMS_SOC: return "BMS_SOC";
    case SensorID::BMS_Total_Voltage: return "BMS_Total_Voltage";
    case SensorID::BMS_Voltage_Differential: return "BMS_Voltage_Differential";
    case SensorID::BMS_Charge_MOS: return "BMS_Charge_MOS";
    case SensorID::BMS_Discharge_MOS: return "BMS_Discharge_MOS";

    // Altimeter
    case SensorID::Baro_Temp: return "Baro_Temp";

    // Internal
    case SensorID::CPU_Temp: return "CPU_Temp";

    default: return "Unknown_Sensor";
  }
}

// Compact abbreviations for UI strings
const char* sensorIDToAbbreviation(SensorID id) {
  switch (id) {
    // ESC
    case SensorID::ESC_MOS_Temp: return "ESC-M";   // MOSFET temp
    case SensorID::ESC_MCU_Temp: return "ESC-C";   // Controller temp
    case SensorID::ESC_CAP_Temp: return "ESC-P";   // Capacitor temp
    case SensorID::Motor_Temp:    return "MTR-T";    // Motor temp
    // ESC Running Errors (Critical) - with bit numbers
    case SensorID::ESC_OverCurrent_Error: return "ESC-RE-0";  // Bit 0
    case SensorID::ESC_LockedRotor_Error: return "ESC-RE-1";  // Bit 1
    case SensorID::ESC_OverTemp_Error: return "ESC-RE-2";     // Bit 2
    case SensorID::ESC_OverVolt_Error: return "ESC-RE-6";     // Bit 6
    case SensorID::ESC_VoltageDrop_Error: return "ESC-RE-7";  // Bit 7
        // ESC Running Warnings - with bit numbers
    case SensorID::ESC_ThrottleSat_Warning: return "ESC-RW-5"; // Bit 5
    // ESC Self-Check Errors - with bit numbers
    case SensorID::ESC_MotorCurrentOut_Error: return "ESC-SE-0";  // Bit 0
    case SensorID::ESC_TotalCurrentOut_Error: return "ESC-SE-1";  // Bit 1
    case SensorID::ESC_MotorVoltageOut_Error: return "ESC-SE-2";  // Bit 2
    case SensorID::ESC_CapNTC_Error: return "ESC-SE-3";          // Bit 3
    case SensorID::ESC_MosNTC_Error: return "ESC-SE-4";          // Bit 4
    case SensorID::ESC_BusVoltRange_Error: return "ESC-SE-5";    // Bit 5
    case SensorID::ESC_BusVoltSample_Error: return "ESC-SE-6";   // Bit 6
    case SensorID::ESC_MotorZLow_Error: return "ESC-SE-7";       // Bit 7
    case SensorID::ESC_MotorZHigh_Error: return "ESC-SE-8";      // Bit 8
    case SensorID::ESC_MotorVDet1_Error: return "ESC-SE-9";      // Bit 9
    case SensorID::ESC_MotorVDet2_Error: return "ESC-SE-10";     // Bit 10
    case SensorID::ESC_MotorIDet2_Error: return "ESC-SE-11";     // Bit 11
    case SensorID::ESC_SwHwIncompat_Error: return "ESC-SE-13";   // Bit 13
    case SensorID::ESC_BootloaderBad_Error: return "ESC-SE-14";  // Bit 14

    // BMS (Battery Management System)
    case SensorID::BMS_MOS_Temp:            return "BMS-M";
    case SensorID::BMS_Balance_Temp:        return "BMS-B";
    case SensorID::BMS_T1_Temp:             return "BMS-T1";
    case SensorID::BMS_T2_Temp:             return "BMS-T2";
    case SensorID::BMS_T3_Temp:             return "BMS-T3";
    case SensorID::BMS_T4_Temp:             return "BMS-T4";
    case SensorID::BMS_High_Cell_Voltage:   return "BMS-CV-H";
    case SensorID::BMS_Low_Cell_Voltage:    return "BMS-CV-L";
    case SensorID::BMS_SOC:                 return "BMS-SOC";
    case SensorID::BMS_Total_Voltage:       return "BMS-Vtot";
    case SensorID::BMS_Voltage_Differential:return "BMS-dV";
    case SensorID::BMS_Charge_MOS:          return "BMS-CHG";
    case SensorID::BMS_Discharge_MOS:       return "BMS-DSG";

    // Altimeter
    case SensorID::Baro_Temp: return "BARO-T";

    // Internal
    case SensorID::CPU_Temp: return "CPU-T";

    default: return "UNK-ERR";
  }
}

void initSimpleMonitor() {
  USBSerial.println("Initializing Simple Monitor System");
  setupLoggerSinks();
  monitors.clear();
  addESCMonitors();
  addBMSMonitors();
  addAltimeterMonitors();
  addInternalMonitors();
  USBSerial.printf("Monitoring %d sensors\n", monitors.size());
}

void checkAllSensors() {
  if (!monitoringEnabled) return;  // Skip if monitoring is disabled

  for (auto* monitor : monitors) {
    if (monitor) {
      monitor->check();
    }
  }
}

void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140& escData, const STR_BMS_TELEMETRY_140& bmsData) {
  if (!monitoringEnabled) return;  // Skip if monitoring is disabled

  // Update our monitoring copies with the safe data
  monitoringEscData = escData;
  monitoringBmsData = bmsData;

  // Now check all monitors (they'll use the safe copies)
  for (auto* monitor : monitors) {
    if (monitor) {
      monitor->check();
    }
  }
}

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

void addBMSMonitors() {
  // BMS MOSFET Temperature (Warning: 50°C, Critical: 60°C)
  static SensorMonitor* bmsMosTemp = new SensorMonitor(
    SensorID::BMS_MOS_Temp,
    bmsTempThresholds,
    []() { return monitoringBmsData.mos_temperature; },
    &multiLogger);
  monitors.push_back(bmsMosTemp);

  // BMS Balance Resistor Temperature (Warning: 50°C, Critical: 60°C)
  static SensorMonitor* bmsBalanceTemp = new SensorMonitor(
    SensorID::BMS_Balance_Temp,
    bmsTempThresholds,
    []() { return monitoringBmsData.balance_temperature; },
    &multiLogger);
  monitors.push_back(bmsBalanceTemp);

  // T1-T4 Cell Temperature Sensors (Warning: 50°C, Critical: 56°C)
  static SensorMonitor* bmsT1Temp = new SensorMonitor(
    SensorID::BMS_T1_Temp,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t1_temperature; },
    &multiLogger);
  monitors.push_back(bmsT1Temp);

  static SensorMonitor* bmsT2Temp = new SensorMonitor(
    SensorID::BMS_T2_Temp,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t2_temperature; },
    &multiLogger);
  monitors.push_back(bmsT2Temp);

  static SensorMonitor* bmsT3Temp = new SensorMonitor(
    SensorID::BMS_T3_Temp,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t3_temperature; },
    &multiLogger);
  monitors.push_back(bmsT3Temp);

  static SensorMonitor* bmsT4Temp = new SensorMonitor(
    SensorID::BMS_T4_Temp,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t4_temperature; },
    &multiLogger);
  monitors.push_back(bmsT4Temp);

  // High Cell Voltage (Warn: 4.1V, Crit: 4.2V)
  static SensorMonitor* bmsHighCellVoltage = new SensorMonitor(
    SensorID::BMS_High_Cell_Voltage,
    bmsHighCellVoltageThresholds,
    []() { return monitoringBmsData.highest_cell_voltage; },
    &multiLogger);
  monitors.push_back(bmsHighCellVoltage);

  // Low Cell Voltage (Warn: 3.2V, Crit: 3.0V)
  static SensorMonitor* bmsLowCellVoltage = new SensorMonitor(
    SensorID::BMS_Low_Cell_Voltage,
    bmsLowCellVoltageThresholds,
    []() { return monitoringBmsData.lowest_cell_voltage; },
    &multiLogger);
  monitors.push_back(bmsLowCellVoltage);

  // State of Charge (Warn: 15%, Crit: 5%)
  static SensorMonitor* bmsSoc = new SensorMonitor(
    SensorID::BMS_SOC,
    bmsSOCThresholds,
    []() { return monitoringBmsData.soc; },
    &multiLogger);
  monitors.push_back(bmsSoc);

  // Total Voltage (Low - Warn: 79.2V, Crit: 69.6V | High - Warn: 100.4V, Crit: 100.8V)
  static SensorMonitor* bmsTotalVoltage = new SensorMonitor(
    SensorID::BMS_Total_Voltage,
    bmsTotalVoltageThresholds,
    []() { return monitoringBmsData.battery_voltage; },
    &multiLogger);
  monitors.push_back(bmsTotalVoltage);

  // Voltage Differential (Warn: 0.20V, Crit: 0.40V)
  static SensorMonitor* bmsVoltageDifferential = new SensorMonitor(
    SensorID::BMS_Voltage_Differential,
    bmsVoltageDifferentialThresholds,
    []() { return monitoringBmsData.voltage_differential; },
    &multiLogger);
  monitors.push_back(bmsVoltageDifferential);

  // Charge MOS (Alert when OFF)
  static BooleanMonitor* bmsChargeMos = new BooleanMonitor(
    SensorID::BMS_Charge_MOS,
    []() { return monitoringBmsData.is_charge_mos; },
    false,  // Alert when false
    AlertLevel::CRIT_HIGH,
    &multiLogger);
  monitors.push_back(bmsChargeMos);

  // Discharge MOS (Alert when OFF)
  static BooleanMonitor* bmsDischargeMos = new BooleanMonitor(
    SensorID::BMS_Discharge_MOS,
    []() { return monitoringBmsData.is_discharge_mos; },
    false,  // Alert when false
    AlertLevel::CRIT_HIGH,
    &multiLogger);
  monitors.push_back(bmsDischargeMos);
}

void addAltimeterMonitors() {
  // Barometer Temperature monitor disabled for now.
  // TODO:
  // The BMP barometer shares the I2C bus with other peripherals; without a
  // global I2C mutex the `Wire` driver can log "Unfinished Repeated Start"
  // errors when transactions from multiple tasks interleave.  Until we
  // introduce a proper mutex (or move barometer access into a dedicated
  // task), skip registering this monitor to avoid the noise and potential
  // bus resets.
  //
  // static SensorMonitor* baroTemp = new SensorMonitor(
  //   SensorID::Baro_Temp,
  //   baroTempThresholds,
  //   []() { return getBaroTemperature(); },
  //   &multiLogger);
  // monitors.push_back(baroTemp);
}

void addInternalMonitors() {
  // ESP32-S3 CPU Temperature (Warning: 50°C, Critical: 80°C)
  static SensorMonitor* cpuTemp = new SensorMonitor(
    SensorID::CPU_Temp,
    cpuTempThresholds,
    []() { return temperatureRead(); },
    &multiLogger);
  monitors.push_back(cpuTemp);
}

void enableMonitoring() {
  monitoringEnabled = true;
  USBSerial.println("Sensor monitoring enabled");
}
