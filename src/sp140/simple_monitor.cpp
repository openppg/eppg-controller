#include "sp140/simple_monitor.h"
#include <vector>
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
  multiLogger.addSink(&serialLogger);  // Serial output for debug
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

void initSimpleMonitor() {
  USBSerial.println("Initializing Simple Monitor System");
  setupLoggerSinks();
  monitors.clear();
  addESCMonitors();
  addBMSMonitors();
  addAltimeterMonitors();
  // Prime while setup is still single-threaded so monitor readers never see
  // the zero-initialized cache and no runtime task races tsens initialization.
  primeCpuTemperatureCache();
  addInternalMonitors();
  USBSerial.printf("Monitoring %d sensors\n", monitors.size());
}

// No more giant switch statement! Each monitor knows its own category.

void checkAllSensors() {
  if (!monitoringEnabled) return;  // Skip if monitoring is disabled

  for (auto* monitor : monitors) {
    if (monitor) {
      monitor->check();
    }
  }
}

void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140& escData,
                             const STR_BMS_TELEMETRY_140& bmsData) {
  if (!monitoringEnabled) return;  // Skip if monitoring is disabled

  // Check connection status
  bool escConnected = (escData.escState == TelemetryState::CONNECTED);
  bool bmsConnected = (bmsData.bmsState == TelemetryState::CONNECTED);

  // Track previous connection states to detect disconnections
  static bool prevEscConnected = false;
  static bool prevBmsConnected = false;

  // If a device disconnected, clear all its alerts
  if (prevEscConnected && !escConnected) {
    // ESC just disconnected - send OK alerts for all ESC sensors to clear them
    for (auto* monitor : monitors) {
      if (monitor && monitor->getCategory() == SensorCategory::ESC) {
        // Force clear this alert by sending OK - this will clear it from the UI
        sendAlertEvent(monitor->getSensorID(), AlertLevel::OK);
      }
    }
  }

  if (prevBmsConnected && !bmsConnected) {
    // BMS just disconnected - send OK alerts for all BMS sensors to clear them
    for (auto* monitor : monitors) {
      if (monitor && monitor->getCategory() == SensorCategory::BMS) {
        // Force clear this alert by sending OK - this will clear it from the UI
        sendAlertEvent(monitor->getSensorID(), AlertLevel::OK);
      }
    }
  }

  // If a device reconnected, reset monitor states so they can alert fresh (like at boot)
  if (!prevEscConnected && escConnected) {
    USBSerial.println("[MONITOR] ESC reconnected - resetting ESC monitor states");
    for (auto* monitor : monitors) {
      if (monitor && monitor->getCategory() == SensorCategory::ESC) {
        monitor->resetState();
      }
    }
  }

  // Grace period: a BMS that is itself still booting can emit sentinel/garbage
  // readings in its first frames. Hold BMS monitors off briefly after the FIRST
  // connect so half-baked data can never fire alerts.
  //
  // One-shot by design. Re-arming on every reconnect edge would mean each
  // transient link drop — a chattering connector, or the SPI-mutex bail in
  // updateBMSData() — blanks all BMS monitoring for another 2 s, right after
  // the disconnect edge above already force-cleared every BMS alert. A real
  // in-flight critical would vanish from the screen and could not be re-raised;
  // flaps recurring faster than the grace would suppress it indefinitely. The
  // booting-BMS garbage this protects against only happens once per power-on.
  static uint32_t bmsGraceStartMs = 0;
  static bool bmsGraceStarted = false;
  static bool bmsGraceExpired = false;
  if (!prevBmsConnected && bmsConnected) {
    USBSerial.println("[MONITOR] BMS reconnected - resetting BMS monitor states");
    if (!bmsGraceStarted) {
      bmsGraceStartMs = millis();
      bmsGraceStarted = true;
    }
    for (auto* monitor : monitors) {
      if (monitor && monitor->getCategory() == SensorCategory::BMS) {
        monitor->resetState();
      }
    }
  }
  if (bmsGraceStarted && !bmsGraceExpired &&
      (millis() - bmsGraceStartMs >= BMS_ALERT_GRACE_MS)) {
    bmsGraceExpired = true;
  }
  const bool bmsMonitorsArmed = bmsConnected && bmsGraceExpired;

  // Update previous connection states
  prevEscConnected = escConnected;
  prevBmsConnected = bmsConnected;

  // Update our monitoring copies with the safe data
  monitoringEscData = escData;
  monitoringBmsData = bmsData;

  // Now check all monitors, but only for connected devices
  for (auto* monitor : monitors) {
    if (monitor) {
      // Determine if this monitor should run based on device connection status
      SensorCategory category = monitor->getCategory();

      bool shouldRun = true;
      switch (category) {
        case SensorCategory::ESC:
          shouldRun = escConnected;
          break;
        case SensorCategory::BMS:
          shouldRun = bmsMonitorsArmed;
          break;
        case SensorCategory::ALTIMETER:
        case SensorCategory::INTERNAL:
        default:
          shouldRun = true;  // Always check altimeter and internal sensors
          break;
      }

      if (shouldRun) {
        monitor->check();
      }
    }
  }
}

void enableMonitoring() {
  monitoringEnabled = true;
  USBSerial.println("Sensor monitoring enabled");
}
