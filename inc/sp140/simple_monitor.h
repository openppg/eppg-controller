#ifndef INC_SP140_SIMPLE_MONITOR_H_
#define INC_SP140_SIMPLE_MONITOR_H_

#include <vector>
#include <functional>
#include <Arduino.h>

// Alert levels
enum class AlertLevel { OK, WARN_LOW, WARN_HIGH, CRIT_LOW, CRIT_HIGH };

// Threshold set
struct Thresholds {
  float warnLow, warnHigh;
  float critLow, critHigh;
};

// Logger interface
struct ILogger {
  virtual void log(const char* name, AlertLevel lvl, float val) = 0;
  virtual ~ILogger() = default;
};

// Serial logger implementation
struct SerialLogger : ILogger {
  void log(const char* name, AlertLevel lvl, float v) override {
    const char* levelNames[] = {"OK", "WARN_LOW", "WARN_HIGH", "CRIT_LOW", "CRIT_HIGH"};
    USBSerial.printf("[%lu] [%s] %s = %.2f\n", millis(), levelNames[(int)lvl], name, v);
  }
};

// Sensor monitor
struct SensorMonitor {
  const char* name;
  Thresholds thr;
  std::function<float()> read;
  AlertLevel last;
  ILogger* logger;

  // Constructor
  SensorMonitor(const char* n, Thresholds t, std::function<float()> r, ILogger* l)
    : name(n), thr(t), read(r), last(AlertLevel::OK), logger(l) {}

  void check() {
    float v = read();
    AlertLevel now = AlertLevel::OK;

    if      (v <= thr.critLow)  now = AlertLevel::CRIT_LOW;
    else if (v <= thr.warnLow)  now = AlertLevel::WARN_LOW;
    else if (v >= thr.critHigh) now = AlertLevel::CRIT_HIGH;
    else if (v >= thr.warnHigh) now = AlertLevel::WARN_HIGH;

    if (now != last) {
      logger->log(name, now, v);
      last = now;
    }
  }
};

// Global monitor registry
extern std::vector<SensorMonitor*> sensors;
extern SerialLogger serialLogger;

// Functions
void initSimpleMonitor();
void checkAllSensors();
void addESCMonitors();
void addBMSMonitors();
void addAltimeterMonitors();

#endif  // INC_SP140_SIMPLE_MONITOR_H_
