#ifndef INC_SP140_ALERT_DISPLAY_H_
#define INC_SP140_ALERT_DISPLAY_H_

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <map>

#include "sp140/simple_monitor.h"  // ILogger, SensorID, AlertLevel

// Structure holding the current counts of active alerts
struct AlertCounts {
  uint8_t warningCount;
  uint8_t criticalCount;
};

// Structure passed over the queue for each alert event
struct AlertEvent {
  SensorID sensorId;
  AlertLevel level;   // New level (OK, WARN_*, CRIT_*)
  uint32_t timestamp; // When event was generated (millis)
};

// Global queues
extern QueueHandle_t alertEventQueue;   // depth ~10
extern QueueHandle_t alertCountQueue;   // depth 1
extern QueueHandle_t alertCarouselQueue;  // depth 1, overwrite

// Message for UI to show a single alert text (or hide)
struct AlertDisplayMsg {
  SensorID id;       // Valid sensor when show == true
  AlertLevel level;  // Alert level for dynamic abbreviations
  bool critical;     // true = critical colouring
  bool show;         // false = hide label
};

extern QueueHandle_t alertDisplayQueue;   // depth 1, overwrite

// Snapshot for carousel text
#define MAX_ALERT_ITEMS 8
struct AlertSnapshot {
  uint32_t epoch;
  uint8_t count;          // number of entries in ids[]
  uint8_t criticalMode;   // 1 = displaying critical alerts, 0 = warnings
  SensorID ids[MAX_ALERT_ITEMS];
};


// Initialise alert aggregation + UI system (creates tasks/queues)
void initAlertDisplay();

// --- LVGL helper functions ---
// Called from setupMainScreen to build the counter circles
void setupAlertCounterUI(bool darkMode);
// Called from refreshDisplay() when AlertCounts update is received
void updateAlertCounterDisplay(const AlertCounts& counts);

// Helper for monitors/UI logger to push alert events
void sendAlertEvent(SensorID id, AlertLevel level);

// ILogger sink that pushes events to the alert queue (for UI)
struct AlertUILogger : ILogger {
  void log(SensorID id, AlertLevel lvl, float v) override {
    sendAlertEvent(id, lvl);
  }
  void log(SensorID id, AlertLevel lvl, bool v) override {
    sendAlertEvent(id, lvl);
  }
};

#endif  // INC_SP140_ALERT_DISPLAY_H_
