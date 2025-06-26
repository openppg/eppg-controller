#ifndef INC_SP140_ALERT_SYSTEM_H_
#define INC_SP140_ALERT_SYSTEM_H_

#include <Arduino.h>
#include <vector>
#include <functional>

// Alert severity levels
enum class AlertSeverity : uint8_t {
  INFO = 0,      // Informational message
  WARNING = 1,   // Warning condition
  CRITICAL = 2,  // Critical condition requiring immediate attention
  FATAL = 3      // System failure condition
};

// Alert categories for grouping similar alerts
enum class AlertCategory : uint8_t {
  TEMPERATURE = 0,
  VOLTAGE = 1,
  CURRENT = 2,
  COMMUNICATION = 3,
  HARDWARE = 4,
  BATTERY = 5,
  SYSTEM = 6
};

// Sensor/component identifiers
enum class SensorID : uint16_t {
  // ESC Temperature sensors
  ESC_MOS_TEMP = 100,
  ESC_MCU_TEMP = 101,
  ESC_CAP_TEMP = 102,
  MOTOR_TEMP = 103,

  // ESC Electrical
  ESC_VOLTAGE = 110,
  ESC_CURRENT = 111,
  ESC_POWER = 112,

  // BMS Temperature sensors
  BMS_TEMP_HIGH = 200,
  BMS_TEMP_LOW = 201,

  // BMS Electrical
  BMS_VOLTAGE = 210,
  BMS_CURRENT = 211,
  BMS_CELL_VOLTAGE_HIGH = 212,
  BMS_CELL_VOLTAGE_LOW = 213,
  BMS_CELL_VOLTAGE_DIFF = 214,
  BMS_SOC = 215,

  // Individual cell voltages (220-243 for 24 cells)
  BMS_CELL_01 = 220, BMS_CELL_02 = 221, BMS_CELL_03 = 222, BMS_CELL_04 = 223,
  BMS_CELL_05 = 224, BMS_CELL_06 = 225, BMS_CELL_07 = 226, BMS_CELL_08 = 227,
  BMS_CELL_09 = 228, BMS_CELL_10 = 229, BMS_CELL_11 = 230, BMS_CELL_12 = 231,
  BMS_CELL_13 = 232, BMS_CELL_14 = 233, BMS_CELL_15 = 234, BMS_CELL_16 = 235,
  BMS_CELL_17 = 236, BMS_CELL_18 = 237, BMS_CELL_19 = 238, BMS_CELL_20 = 239,
  BMS_CELL_21 = 240, BMS_CELL_22 = 241, BMS_CELL_23 = 242, BMS_CELL_24 = 243,

  // Communication
  ESC_COMM = 300,
  BMS_COMM = 301,

  // System components
  ALTIMETER = 400,
  VIBRATION = 401,
  LED_SYSTEM = 402,
  DISPLAY = 403
};

// Structure to define sensor thresholds
struct SensorThresholds {
  float min_value;        // Minimum acceptable value
  float max_value;        // Maximum acceptable value
  float warning_low;      // Low warning threshold
  float warning_high;     // High warning threshold
  float critical_low;     // Low critical threshold
  float critical_high;    // High critical threshold
  bool enabled;           // Whether monitoring is enabled for this sensor
};

// Alert record structure
struct AlertRecord {
  SensorID sensor_id;
  AlertCategory category;
  AlertSeverity severity;
  float value;
  float threshold;
  unsigned long timestamp;
  char message[128];
  bool active;            // Whether the alert is currently active
  bool acknowledged;      // Whether the alert has been acknowledged
};

// Alert output interface
class AlertOutput {
public:
  virtual ~AlertOutput() = default;
  virtual void outputAlert(const AlertRecord& alert) = 0;
  virtual void init() = 0;
};

// Serial console output implementation
class SerialAlertOutput : public AlertOutput {
public:
  void init() override;
  void outputAlert(const AlertRecord& alert) override;
};

// Future implementations could include:
// class SDCardAlertOutput : public AlertOutput { ... };
// class DisplayAlertOutput : public AlertOutput { ... };
// class NetworkAlertOutput : public AlertOutput { ... };

// Main alert system class
class AlertSystem {
private:
  std::vector<AlertRecord> active_alerts_;
  std::vector<AlertRecord> alert_history_;
  std::vector<AlertOutput*> outputs_;
  SensorThresholds thresholds_[512];  // Array indexed by SensorID
  unsigned long last_check_time_;
  bool system_enabled_;
  static const size_t MAX_HISTORY_SIZE = 100;

  void addToHistory(const AlertRecord& alert);
  bool isAlertActive(SensorID sensor_id, AlertSeverity severity);
  void clearAlert(SensorID sensor_id, AlertSeverity severity);
  AlertSeverity evaluateValue(SensorID sensor_id, float value, float* threshold_crossed = nullptr);

public:
  AlertSystem();
  ~AlertSystem();

  // System control
  void init();
  void enable(bool enabled = true);
  void update();  // Call this regularly in main loop

  // Output management
  void addOutput(AlertOutput* output);
  void removeOutput(AlertOutput* output);

  // Threshold configuration
  void setThresholds(SensorID sensor_id, const SensorThresholds& thresholds);
  SensorThresholds getThresholds(SensorID sensor_id) const;
  void enableSensor(SensorID sensor_id, bool enabled = true);

  // Alert generation
  void checkValue(SensorID sensor_id, AlertCategory category, float value, const char* sensor_name = nullptr);
  void raiseAlert(SensorID sensor_id, AlertCategory category, AlertSeverity severity,
                  float value, float threshold, const char* message);

  // Alert management
  void acknowledgeAlert(size_t alert_index);
  void acknowledgeAllAlerts();
  void clearAllAlerts();

  // Getters
  const std::vector<AlertRecord>& getActiveAlerts() const { return active_alerts_; }
  const std::vector<AlertRecord>& getAlertHistory() const { return alert_history_; }
  size_t getActiveAlertCount() const { return active_alerts_.size(); }
  size_t getActiveAlertCount(AlertSeverity severity) const;
  size_t getActiveAlertCount(AlertCategory category) const;
  bool isEnabled() const { return system_enabled_; }

  // Utility functions
  static const char* severityToString(AlertSeverity severity);
  static const char* categoryToString(AlertCategory category);
  static const char* sensorToString(SensorID sensor_id);
};

// Global alert system instance
extern AlertSystem alertSystem;

// Convenience functions for common operations
void initAlertSystem();
void setupDefaultThresholds();
void checkESCTelemetry();
void checkBMSTelemetry();
void logAlert(AlertSeverity severity, AlertCategory category, const char* message);

#endif  // INC_SP140_ALERT_SYSTEM_H_
