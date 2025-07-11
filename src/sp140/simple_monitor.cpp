#include "sp140/simple_monitor.h"
#include "sp140/monitor_config.h"
#include "sp140/globals.h"
#include "sp140/altimeter.h"
#include "sp140/utilities.h"
#include "sp140/alert_display.h"  // UI logger & event queue
#include "sp140/esc.h"  // For ESC error checking functions

// Include the split monitor files
#include "sp140/esc_monitors.h"
#include "sp140/bms_monitors.h"
#include "sp140/system_monitors.h"

// Global instances
MultiLogger multiLogger;  // Defined here, declared extern in header
static AlertUILogger uiLogger;
std::vector<IMonitor*> monitors;
SerialLogger serialLogger;
bool monitoringEnabled = false;  // Start with monitoring disabled

// Thread-safe copies for monitoring (updated by checkAllSensorsWithData)
STR_ESC_TELEMETRY_140 monitoringEscData = {};
STR_BMS_TELEMETRY_140 monitoringBmsData = {};

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
    case SensorID::ESC_MOS_Temp: return "MC-F";   // MOSFET temp
    case SensorID::ESC_MCU_Temp: return "MC-C";   // Controller temp
    case SensorID::ESC_CAP_Temp: return "MC-P";   // Capacitor temp
    case SensorID::Motor_Temp:    return "MC-M";    // Motor temp
    // ESC Running Errors (Critical) - with bit numbers
    case SensorID::ESC_OverCurrent_Error: return "MC-RE-0";  // Bit 0
    case SensorID::ESC_LockedRotor_Error: return "MC-RE-1";  // Bit 1
    case SensorID::ESC_OverTemp_Error: return "MC-RE-2";     // Bit 2
    case SensorID::ESC_OverVolt_Error: return "MC-RE-6";     // Bit 6
    case SensorID::ESC_VoltageDrop_Error: return "MC-RE-7";  // Bit 7
    // ESC Running Warnings - with bit numbers
    case SensorID::ESC_ThrottleSat_Warning: return "MC-RW-5"; // Bit 5
    // ESC Self-Check Errors - with bit numbers
    case SensorID::ESC_MotorCurrentOut_Error: return "MC-SE-0";  // Bit 0
    case SensorID::ESC_TotalCurrentOut_Error: return "MC-SE-1";  // Bit 1
    case SensorID::ESC_MotorVoltageOut_Error: return "MC-SE-2";  // Bit 2
    case SensorID::ESC_CapNTC_Error: return "MC-SE-3";          // Bit 3
    case SensorID::ESC_MosNTC_Error: return "MC-SE-4";          // Bit 4
    case SensorID::ESC_BusVoltRange_Error: return "MC-SE-5";    // Bit 5
    case SensorID::ESC_BusVoltSample_Error: return "MC-SE-6";   // Bit 6
    case SensorID::ESC_MotorZLow_Error: return "MC-SE-7";       // Bit 7
    case SensorID::ESC_MotorZHigh_Error: return "MC-SE-8";      // Bit 8
    case SensorID::ESC_MotorVDet1_Error: return "MC-SE-9";      // Bit 9
    case SensorID::ESC_MotorVDet2_Error: return "MC-SE-10";     // Bit 10
    case SensorID::ESC_MotorIDet2_Error: return "MC-SE-11";     // Bit 11
    case SensorID::ESC_SwHwIncompat_Error: return "MC-SE-13";   // Bit 13
    case SensorID::ESC_BootloaderBad_Error: return "MC-SE-14";  // Bit 14

    // BMS (Battery Management System)
    case SensorID::BMS_MOS_Temp:            return "BC-F";
    case SensorID::BMS_Balance_Temp:        return "BC-B";
    case SensorID::BMS_T1_Temp:             return "BC-T1";
    case SensorID::BMS_T2_Temp:             return "BC-T2";
    case SensorID::BMS_T3_Temp:             return "BC-T3";
    case SensorID::BMS_T4_Temp:             return "BC-T4";
    case SensorID::BMS_High_Cell_Voltage:   return "BC-CV-H";
    case SensorID::BMS_Low_Cell_Voltage:    return "BC-CV-L";
    case SensorID::BMS_SOC:                 return "BC-SOC";
    case SensorID::BMS_Total_Voltage:       return "BC-Vtot";
    case SensorID::BMS_Voltage_Differential:return "BC-dV";
    case SensorID::BMS_Charge_MOS:          return "BC-CHG";
    case SensorID::BMS_Discharge_MOS:       return "BC-DSG";

    // Altimeter
    case SensorID::Baro_Temp: return "BARO-T";

    // Internal
    case SensorID::CPU_Temp: return "CPU-T";

    default: return "UNK-ERR";
  }
}

// Dynamic abbreviations for temperature sensors based on alert level
const char* sensorIDToAbbreviationWithLevel(SensorID id, AlertLevel level) {
  switch (id) {
    // ESC Temperature sensors with dynamic suffixes
    case SensorID::ESC_MOS_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "MC-FT-H";  // MOSFET temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "MC-FT-L";  // MOSFET temp low
      }
      return "MC-F";  // Default
      
    case SensorID::ESC_MCU_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "MC-CT-H";  // Controller temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "MC-CT-L";  // Controller temp low
      }
      return "MC-C";  // Default
      
    case SensorID::ESC_CAP_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "MC-PT-H";  // Capacitor temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "MC-PT-L";  // Capacitor temp low
      }
      return "MC-P";  // Default
      
    case SensorID::Motor_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "MC-MT-H";  // Motor temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "MC-MT-L";  // Motor temp low
      }
      return "MC-M";  // Default
      
    // BMS Temperature sensors with dynamic suffixes
    case SensorID::BMS_MOS_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "BC-F-H";  // MOS temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "BC-F-L";  // MOS temp low
      }
      return "BC-F";  // Default
      
    case SensorID::BMS_Balance_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "BC-B-H";  // Balance temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "BC-B-L";  // Balance temp low
      }
      return "BC-B";  // Default
      
    case SensorID::BMS_T1_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "BC-T1-H";  // Cell 1 temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "BC-T1-L";  // Cell 1 temp low
      }
      return "BC-T1";  // Default
      
    case SensorID::BMS_T2_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "BC-T2-H";  // Cell 2 temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "BC-T2-L";  // Cell 2 temp low
      }
      return "BC-T2";  // Default
      
    case SensorID::BMS_T3_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "BC-T3-H";  // Cell 3 temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "BC-T3-L";  // Cell 3 temp low
      }
      return "BC-T3";  // Default
      
    case SensorID::BMS_T4_Temp:
      if (level == AlertLevel::WARN_HIGH || level == AlertLevel::CRIT_HIGH) {
        return "BC-T4-H";  // Cell 4 temp high
      } else if (level == AlertLevel::WARN_LOW || level == AlertLevel::CRIT_LOW) {
        return "BC-T4-L";  // Cell 4 temp low
      }
      return "BC-T4";  // Default
      
    // For all other sensors, use the standard abbreviation
    default:
      return sensorIDToAbbreviation(id);
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

void enableMonitoring() {
  monitoringEnabled = true;
  USBSerial.println("Sensor monitoring enabled");
}
