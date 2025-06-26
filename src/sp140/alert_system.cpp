#include "sp140/alert_system.h"
#include "sp140/globals.h"
#include <algorithm>

// Global alert system instance
AlertSystem alertSystem;

// SerialAlertOutput implementation
void SerialAlertOutput::init() {
  // Serial is already initialized in main setup
}

void SerialAlertOutput::outputAlert(const AlertRecord& alert) {
  // Format: [TIMESTAMP] [SEVERITY] [CATEGORY] Sensor: Message (Value: X, Threshold: Y)
  USBSerial.printf("[%lu] [%s] [%s] %s: %s (Value: %.2f, Threshold: %.2f)\n",
                   alert.timestamp,
                   AlertSystem::severityToString(alert.severity),
                   AlertSystem::categoryToString(alert.category),
                   AlertSystem::sensorToString(alert.sensor_id),
                   alert.message,
                   alert.value,
                   alert.threshold);
}

// AlertSystem implementation
AlertSystem::AlertSystem() : last_check_time_(0), system_enabled_(false) {
  // Initialize all thresholds as disabled
  for (int i = 0; i < 512; i++) {
    thresholds_[i].enabled = false;
  }
}

AlertSystem::~AlertSystem() {
  // Clean up outputs (don't delete, just clear pointers)
  outputs_.clear();
}

void AlertSystem::init() {
  system_enabled_ = true;
  last_check_time_ = millis();

  // Clear any existing alerts
  active_alerts_.clear();
  alert_history_.clear();

  // Initialize all registered outputs
  for (auto* output : outputs_) {
    if (output) {
      output->init();
    }
  }

  USBSerial.println("Alert System initialized");
}

void AlertSystem::enable(bool enabled) {
  system_enabled_ = enabled;
  if (!enabled) {
    clearAllAlerts();
  }
}

void AlertSystem::update() {
  if (!system_enabled_) return;

  unsigned long current_time = millis();

  // Update timestamp
  last_check_time_ = current_time;

  // Check if any critical alerts need system action
  size_t critical_count = getActiveAlertCount(AlertSeverity::CRITICAL);
  size_t fatal_count = getActiveAlertCount(AlertSeverity::FATAL);

  if (fatal_count > 0) {
    // Could implement emergency shutdown logic here
    USBSerial.printf("FATAL ALERT COUNT: %d - System requires immediate attention!\n", fatal_count);
  } else if (critical_count > 0) {
    // Could implement warning signals (buzzer, LED, etc.)
    USBSerial.printf("CRITICAL ALERT COUNT: %d\n", critical_count);
  }
}

void AlertSystem::addOutput(AlertOutput* output) {
  if (output && std::find(outputs_.begin(), outputs_.end(), output) == outputs_.end()) {
    outputs_.push_back(output);
  }
}

void AlertSystem::removeOutput(AlertOutput* output) {
  outputs_.erase(std::remove(outputs_.begin(), outputs_.end(), output), outputs_.end());
}

void AlertSystem::setThresholds(SensorID sensor_id, const SensorThresholds& thresholds) {
  uint16_t index = static_cast<uint16_t>(sensor_id);
  if (index < 512) {
    thresholds_[index] = thresholds;
  }
}

SensorThresholds AlertSystem::getThresholds(SensorID sensor_id) const {
  uint16_t index = static_cast<uint16_t>(sensor_id);
  if (index < 512) {
    return thresholds_[index];
  }
  return SensorThresholds{0, 0, 0, 0, 0, 0, false};
}

void AlertSystem::enableSensor(SensorID sensor_id, bool enabled) {
  uint16_t index = static_cast<uint16_t>(sensor_id);
  if (index < 512) {
    thresholds_[index].enabled = enabled;
  }
}

AlertSeverity AlertSystem::evaluateValue(SensorID sensor_id, float value, float* threshold_crossed) {
  uint16_t index = static_cast<uint16_t>(sensor_id);
  if (index >= 512 || !thresholds_[index].enabled) {
    return AlertSeverity::INFO;  // No alert if not configured
  }

  const SensorThresholds& thresh = thresholds_[index];

  // Check critical thresholds first
  if (value <= thresh.critical_low) {
    if (threshold_crossed) *threshold_crossed = thresh.critical_low;
    return AlertSeverity::CRITICAL;
  }
  if (value >= thresh.critical_high) {
    if (threshold_crossed) *threshold_crossed = thresh.critical_high;
    return AlertSeverity::CRITICAL;
  }

  // Check warning thresholds
  if (value <= thresh.warning_low) {
    if (threshold_crossed) *threshold_crossed = thresh.warning_low;
    return AlertSeverity::WARNING;
  }
  if (value >= thresh.warning_high) {
    if (threshold_crossed) *threshold_crossed = thresh.warning_high;
    return AlertSeverity::WARNING;
  }

  return AlertSeverity::INFO;  // Normal range
}

void AlertSystem::checkValue(SensorID sensor_id, AlertCategory category, float value, const char* sensor_name) {
  if (!system_enabled_) return;

  float threshold_crossed;
  AlertSeverity severity = evaluateValue(sensor_id, value, &threshold_crossed);

  if (severity == AlertSeverity::WARNING || severity == AlertSeverity::CRITICAL) {
    // Check if this alert is already active
    if (!isAlertActive(sensor_id, severity)) {
      char message[128];
      const char* name = sensor_name ? sensor_name : sensorToString(sensor_id);

      if (severity == AlertSeverity::WARNING) {
        snprintf(message, sizeof(message), "%s value %.2f exceeds warning threshold", name, value);
      } else {
        snprintf(message, sizeof(message), "%s value %.2f exceeds CRITICAL threshold", name, value);
      }

      raiseAlert(sensor_id, category, severity, value, threshold_crossed, message);
    }
  } else {
    // Value is normal, clear any existing alerts for this sensor
    clearAlert(sensor_id, AlertSeverity::WARNING);
    clearAlert(sensor_id, AlertSeverity::CRITICAL);
  }
}

void AlertSystem::raiseAlert(SensorID sensor_id, AlertCategory category, AlertSeverity severity,
                            float value, float threshold, const char* message) {
  if (!system_enabled_) return;

  AlertRecord alert;
  alert.sensor_id = sensor_id;
  alert.category = category;
  alert.severity = severity;
  alert.value = value;
  alert.threshold = threshold;
  alert.timestamp = millis();
  alert.active = true;
  alert.acknowledged = false;
  strncpy(alert.message, message, sizeof(alert.message) - 1);
  alert.message[sizeof(alert.message) - 1] = '\0';

  // Add to active alerts
  active_alerts_.push_back(alert);

  // Add to history
  addToHistory(alert);

  // Output to all registered outputs
  for (auto* output : outputs_) {
    if (output) {
      output->outputAlert(alert);
    }
  }
}

bool AlertSystem::isAlertActive(SensorID sensor_id, AlertSeverity severity) {
  for (const auto& alert : active_alerts_) {
    if (alert.sensor_id == sensor_id && alert.severity == severity && alert.active) {
      return true;
    }
  }
  return false;
}

void AlertSystem::clearAlert(SensorID sensor_id, AlertSeverity severity) {
  for (auto& alert : active_alerts_) {
    if (alert.sensor_id == sensor_id && alert.severity == severity) {
      alert.active = false;
    }
  }

  // Remove inactive alerts
  active_alerts_.erase(
    std::remove_if(active_alerts_.begin(), active_alerts_.end(),
                   [](const AlertRecord& alert) { return !alert.active; }),
    active_alerts_.end());
}

void AlertSystem::addToHistory(const AlertRecord& alert) {
  alert_history_.push_back(alert);

  // Limit history size
  if (alert_history_.size() > MAX_HISTORY_SIZE) {
    alert_history_.erase(alert_history_.begin());
  }
}

void AlertSystem::acknowledgeAlert(size_t alert_index) {
  if (alert_index < active_alerts_.size()) {
    active_alerts_[alert_index].acknowledged = true;
  }
}

void AlertSystem::acknowledgeAllAlerts() {
  for (auto& alert : active_alerts_) {
    alert.acknowledged = true;
  }
}

void AlertSystem::clearAllAlerts() {
  active_alerts_.clear();
}

size_t AlertSystem::getActiveAlertCount(AlertSeverity severity) const {
  return std::count_if(active_alerts_.begin(), active_alerts_.end(),
                       [severity](const AlertRecord& alert) {
                         return alert.severity == severity && alert.active;
                       });
}

size_t AlertSystem::getActiveAlertCount(AlertCategory category) const {
  return std::count_if(active_alerts_.begin(), active_alerts_.end(),
                       [category](const AlertRecord& alert) {
                         return alert.category == category && alert.active;
                       });
}

const char* AlertSystem::severityToString(AlertSeverity severity) {
  switch (severity) {
    case AlertSeverity::INFO: return "INFO";
    case AlertSeverity::WARNING: return "WARNING";
    case AlertSeverity::CRITICAL: return "CRITICAL";
    case AlertSeverity::FATAL: return "FATAL";
    default: return "UNKNOWN";
  }
}

const char* AlertSystem::categoryToString(AlertCategory category) {
  switch (category) {
    case AlertCategory::TEMPERATURE: return "TEMP";
    case AlertCategory::VOLTAGE: return "VOLT";
    case AlertCategory::CURRENT: return "CURR";
    case AlertCategory::COMMUNICATION: return "COMM";
    case AlertCategory::HARDWARE: return "HW";
    case AlertCategory::BATTERY: return "BAT";
    case AlertCategory::SYSTEM: return "SYS";
    default: return "UNKNOWN";
  }
}

const char* AlertSystem::sensorToString(SensorID sensor_id) {
  switch (sensor_id) {
    // ESC Temperature
    case SensorID::ESC_MOS_TEMP: return "ESC_MOS_TEMP";
    case SensorID::ESC_MCU_TEMP: return "ESC_MCU_TEMP";
    case SensorID::ESC_CAP_TEMP: return "ESC_CAP_TEMP";
    case SensorID::MOTOR_TEMP: return "MOTOR_TEMP";

    // ESC Electrical
    case SensorID::ESC_VOLTAGE: return "ESC_VOLTAGE";
    case SensorID::ESC_CURRENT: return "ESC_CURRENT";
    case SensorID::ESC_POWER: return "ESC_POWER";

    // BMS Temperature
    case SensorID::BMS_TEMP_HIGH: return "BMS_TEMP_HIGH";
    case SensorID::BMS_TEMP_LOW: return "BMS_TEMP_LOW";

    // BMS Electrical
    case SensorID::BMS_VOLTAGE: return "BMS_VOLTAGE";
    case SensorID::BMS_CURRENT: return "BMS_CURRENT";
    case SensorID::BMS_CELL_VOLTAGE_HIGH: return "BMS_CELL_HIGH";
    case SensorID::BMS_CELL_VOLTAGE_LOW: return "BMS_CELL_LOW";
    case SensorID::BMS_CELL_VOLTAGE_DIFF: return "BMS_CELL_DIFF";
    case SensorID::BMS_SOC: return "BMS_SOC";

    // Communication
    case SensorID::ESC_COMM: return "ESC_COMM";
    case SensorID::BMS_COMM: return "BMS_COMM";

    // System
    case SensorID::ALTIMETER: return "ALTIMETER";
    case SensorID::VIBRATION: return "VIBRATION";
    case SensorID::LED_SYSTEM: return "LED_SYSTEM";
    case SensorID::DISPLAY: return "DISPLAY";

    default:
      // Handle individual cell voltages
      uint16_t id = static_cast<uint16_t>(sensor_id);
      if (id >= 220 && id <= 243) {
        static char cell_name[16];
        snprintf(cell_name, sizeof(cell_name), "BMS_CELL_%02d", id - 219);
        return cell_name;
      }
      return "UNKNOWN_SENSOR";
  }
}

// Convenience functions
void initAlertSystem() {
  // Add serial output
  static SerialAlertOutput serialOutput;
  alertSystem.addOutput(&serialOutput);

  // Initialize the system
  alertSystem.init();

  // Setup default thresholds
  setupDefaultThresholds();
}

void setupDefaultThresholds() {
  // ESC Temperature thresholds
  SensorThresholds escMosThresh = {-50, 200, 75, 90, 30, 110, true};
  SensorThresholds escMcuThresh = {-50, 200, 65, 80, 20, 95, true};
  SensorThresholds escCapThresh = {-50, 200, 70, 85, 25, 100, true};
  SensorThresholds motorThresh = {-50, 200, 75, 90, 30, 110, true};

  alertSystem.setThresholds(SensorID::ESC_MOS_TEMP, escMosThresh);
  alertSystem.setThresholds(SensorID::ESC_MCU_TEMP, escMcuThresh);
  alertSystem.setThresholds(SensorID::ESC_CAP_TEMP, escCapThresh);
  alertSystem.setThresholds(SensorID::MOTOR_TEMP, motorThresh);

  // ESC Electrical thresholds
  SensorThresholds escVoltThresh = {60, 120, 70, 105, 65, 110, true};
  SensorThresholds escCurrThresh = {-50, 200, -5, 150, -10, 180, true};

  alertSystem.setThresholds(SensorID::ESC_VOLTAGE, escVoltThresh);
  alertSystem.setThresholds(SensorID::ESC_CURRENT, escCurrThresh);

  // BMS Temperature thresholds
  SensorThresholds bmsTempThresh = {-20, 80, 5, 50, 0, 60, true};
  alertSystem.setThresholds(SensorID::BMS_TEMP_HIGH, bmsTempThresh);
  alertSystem.setThresholds(SensorID::BMS_TEMP_LOW, bmsTempThresh);

  // BMS Electrical thresholds
  SensorThresholds bmsVoltThresh = {60, 120, 70, 105, 65, 110, true};
  SensorThresholds bmsCurrThresh = {-100, 100, -50, 80, -80, 90, true};
  SensorThresholds cellVoltThresh = {2.5, 4.5, 3.0, 4.2, 2.8, 4.3, true};
  SensorThresholds cellDiffThresh = {0, 2.0, 0, 0.5, 0, 1.0, true};
  SensorThresholds socThresh = {0, 100, 10, 95, 5, 100, true};

  alertSystem.setThresholds(SensorID::BMS_VOLTAGE, bmsVoltThresh);
  alertSystem.setThresholds(SensorID::BMS_CURRENT, bmsCurrThresh);
  alertSystem.setThresholds(SensorID::BMS_CELL_VOLTAGE_HIGH, cellVoltThresh);
  alertSystem.setThresholds(SensorID::BMS_CELL_VOLTAGE_LOW, cellVoltThresh);
  alertSystem.setThresholds(SensorID::BMS_CELL_VOLTAGE_DIFF, cellDiffThresh);
  alertSystem.setThresholds(SensorID::BMS_SOC, socThresh);

  // Set thresholds for individual cells
  for (int i = 220; i <= 243; i++) {
    alertSystem.setThresholds(static_cast<SensorID>(i), cellVoltThresh);
  }
}

void checkESCTelemetry() {
  if (!alertSystem.isEnabled()) return;

  // Check ESC temperatures
  alertSystem.checkValue(SensorID::ESC_MOS_TEMP, AlertCategory::TEMPERATURE,
                        escTelemetryData.mos_temp, "ESC MOS Temperature");
  alertSystem.checkValue(SensorID::ESC_MCU_TEMP, AlertCategory::TEMPERATURE,
                        escTelemetryData.mcu_temp, "ESC MCU Temperature");
  alertSystem.checkValue(SensorID::ESC_CAP_TEMP, AlertCategory::TEMPERATURE,
                        escTelemetryData.cap_temp, "ESC Capacitor Temperature");
  alertSystem.checkValue(SensorID::MOTOR_TEMP, AlertCategory::TEMPERATURE,
                        escTelemetryData.motor_temp, "Motor Temperature");

  // Check ESC electrical values
  alertSystem.checkValue(SensorID::ESC_VOLTAGE, AlertCategory::VOLTAGE,
                        escTelemetryData.volts, "ESC Voltage");
  alertSystem.checkValue(SensorID::ESC_CURRENT, AlertCategory::CURRENT,
                        escTelemetryData.amps, "ESC Current");

  // Check communication status
  if (escTelemetryData.escState == TelemetryState::NOT_CONNECTED) {
    alertSystem.raiseAlert(SensorID::ESC_COMM, AlertCategory::COMMUNICATION,
                          AlertSeverity::CRITICAL, 0, 0, "ESC Communication Lost");
  }
}

void checkBMSTelemetry() {
  if (!alertSystem.isEnabled()) return;

  // Check BMS temperatures
  alertSystem.checkValue(SensorID::BMS_TEMP_HIGH, AlertCategory::TEMPERATURE,
                        bmsTelemetryData.highest_temperature, "BMS High Temperature");
  alertSystem.checkValue(SensorID::BMS_TEMP_LOW, AlertCategory::TEMPERATURE,
                        bmsTelemetryData.lowest_temperature, "BMS Low Temperature");

  // Check BMS electrical values
  alertSystem.checkValue(SensorID::BMS_VOLTAGE, AlertCategory::VOLTAGE,
                        bmsTelemetryData.battery_voltage, "BMS Voltage");
  alertSystem.checkValue(SensorID::BMS_CURRENT, AlertCategory::CURRENT,
                        bmsTelemetryData.battery_current, "BMS Current");
  alertSystem.checkValue(SensorID::BMS_CELL_VOLTAGE_HIGH, AlertCategory::VOLTAGE,
                        bmsTelemetryData.highest_cell_voltage, "BMS Highest Cell");
  alertSystem.checkValue(SensorID::BMS_CELL_VOLTAGE_LOW, AlertCategory::VOLTAGE,
                        bmsTelemetryData.lowest_cell_voltage, "BMS Lowest Cell");
  alertSystem.checkValue(SensorID::BMS_CELL_VOLTAGE_DIFF, AlertCategory::VOLTAGE,
                        bmsTelemetryData.voltage_differential, "BMS Cell Differential");
  alertSystem.checkValue(SensorID::BMS_SOC, AlertCategory::BATTERY,
                        bmsTelemetryData.soc, "BMS State of Charge");

  // Check individual cell voltages
  for (int i = 0; i < BMS_CELLS_NUM; i++) {
    if (bmsTelemetryData.cell_voltages[i] > 0) {  // Only check if valid
      SensorID cellSensorId = static_cast<SensorID>(220 + i);
      char cellName[20];
      snprintf(cellName, sizeof(cellName), "Cell %d", i + 1);
      alertSystem.checkValue(cellSensorId, AlertCategory::VOLTAGE,
                            bmsTelemetryData.cell_voltages[i], cellName);
    }
  }

  // Check communication status
  if (bmsTelemetryData.bmsState == TelemetryState::NOT_CONNECTED) {
    alertSystem.raiseAlert(SensorID::BMS_COMM, AlertCategory::COMMUNICATION,
                          AlertSeverity::CRITICAL, 0, 0, "BMS Communication Lost");
  }
}

void logAlert(AlertSeverity severity, AlertCategory category, const char* message) {
  alertSystem.raiseAlert(SensorID::DISPLAY, category, severity, 0, 0, message);
}
