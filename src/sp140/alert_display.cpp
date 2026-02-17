#include "../../inc/sp140/alert_display.h"
#include <Arduino.h>
#include <freertos/queue.h>
#include <map>
#include <algorithm>
#include "../../inc/sp140/lvgl/lvgl_alerts.h"
#include "../../inc/sp140/lvgl/lvgl_updates.h"
#include "../../inc/sp140/vibration_pwm.h"

// ------------ Globals -------------
QueueHandle_t alertEventQueue = NULL;
QueueHandle_t alertCarouselQueue = NULL;
QueueHandle_t alertUIQueue = NULL;
TaskHandle_t alertAggregationTaskHandle = NULL;

// Internal state: current alert levels per sensor
static std::map<SensorID, AlertLevel> g_currentLevels;
static AlertCounts g_currentCounts = {0, 0};
static AlertCounts g_previousCounts = {0, 0};  // Track previous counts for transitions
static uint32_t g_epoch = 0;

// Rotation state for display - separate lists for warnings and criticals
static std::vector<SensorID> g_critList;
static std::vector<SensorID> g_warnList;
static size_t g_critRotateIdx = 0;
static size_t g_warnRotateIdx = 0;
static unsigned long g_lastRotateMs = 0;

// Forward declarations
static void alertAggregationTask(void* parameter);
static void recalcCountsAndPublish();
static void handleAlertVibration(const AlertCounts& newCounts, const AlertCounts& previousCounts);

// ------------ Public helpers -------------
void initAlertDisplay() {
  // Create queues – small, non-blocking
  alertEventQueue  = xQueueCreate(20, sizeof(AlertEvent));
  alertCarouselQueue = xQueueCreate(1, sizeof(AlertSnapshot));
  alertUIQueue = xQueueCreate(1, sizeof(AlertUIUpdate));  // Unified queue for synchronized updates

  if (!alertEventQueue || !alertCarouselQueue || !alertUIQueue) {
    USBSerial.println("[AlertDisplay] Failed creating queues");
    return;
  }

  // Create aggregation task (low priority)
  xTaskCreate(alertAggregationTask, "AlertAgg", 4096, NULL, 1, &alertAggregationTaskHandle);
  USBSerial.println("[AlertDisplay] Init complete");
}

void sendAlertEvent(SensorID id, AlertLevel level) {
  if (!alertEventQueue) return;
  AlertEvent ev{ id, level, millis() };
  xQueueSend(alertEventQueue, &ev, 0);  // best-effort, drop if full
}

// Helper to build and send UI update with current state
static void sendUIUpdate() {
  AlertUIUpdate update;
  update.counts = g_currentCounts;
  update.criticalAlertsActive = (g_currentCounts.criticalCount > 0);
  update.updateEpoch = g_epoch;

  // Critical display (shows on top, in altitude area)
  if (!g_critList.empty()) {
    update.showCritical = true;
    update.criticalId = g_critList[g_critRotateIdx];
    update.criticalLevel = g_currentLevels[g_critList[g_critRotateIdx]];
  } else {
    update.showCritical = false;
  }

  // Warning display (shows below critical)
  if (!g_warnList.empty()) {
    update.showWarning = true;
    update.warningId = g_warnList[g_warnRotateIdx];
    update.warningLevel = g_currentLevels[g_warnList[g_warnRotateIdx]];
  } else {
    update.showWarning = false;
  }

  if (alertUIQueue) {
    xQueueOverwrite(alertUIQueue, &update);
  }
}

// ------------ Internal implementation -------------
static void alertAggregationTask(void* parameter) {
  AlertEvent ev;
  for (;;) {
    // Wait up to 500ms for new event
    if (xQueueReceive(alertEventQueue, &ev, pdMS_TO_TICKS(500)) == pdTRUE) {
      // Update current levels map
      if (ev.level == AlertLevel::OK) {
        g_currentLevels.erase(ev.sensorId);
      } else {
        g_currentLevels[ev.sensorId] = ev.level;
      }

      recalcCountsAndPublish();
    }

    // Handle rotation every 2s if either list not empty
    bool hasAlerts = !g_critList.empty() || !g_warnList.empty();
    if (hasAlerts) {
      unsigned long now = millis();
      if (now - g_lastRotateMs >= 2000) {
        g_lastRotateMs = now;

        // Rotate both lists independently
        if (!g_critList.empty()) {
          g_critRotateIdx = (g_critRotateIdx + 1) % g_critList.size();
        }
        if (!g_warnList.empty()) {
          g_warnRotateIdx = (g_warnRotateIdx + 1) % g_warnList.size();
        }

        sendUIUpdate();
      }
    } else {
      // If both lists empty, ensure labels hidden once
      static bool hideSent = false;
      if (!hideSent) {
        sendUIUpdate();
        hideSent = true;
      }
    }
  }
}

static void recalcCountsAndPublish() {
  AlertCounts counts{0, 0};
  std::vector<SensorID> newCritList;
  std::vector<SensorID> newWarnList;
  for (const auto& kv : g_currentLevels) {
    switch (kv.second) {
      case AlertLevel::WARN_LOW:
      case AlertLevel::WARN_HIGH:
        newWarnList.push_back(kv.first);
        counts.warningCount++;
        break;
      case AlertLevel::CRIT_LOW:
      case AlertLevel::CRIT_HIGH:
        newCritList.push_back(kv.first);
        counts.criticalCount++;
        break;
      default:
        break;
    }
  }

  // Handle vibration alerts based on count changes
  handleAlertVibration(counts, g_previousCounts);
  g_previousCounts = counts;

  // Always update current counts for internal tracking
  bool countsChanged = (counts.warningCount != g_currentCounts.warningCount ||
                       counts.criticalCount != g_currentCounts.criticalCount);
  g_currentCounts = counts;

  // Check if lists changed
  bool critListChanged = (newCritList != g_critList);
  bool warnListChanged = (newWarnList != g_warnList);

  // Send unified update when counts change or display lists change
  if (countsChanged || critListChanged || warnListChanged) {
    if (critListChanged) {
      g_critList = newCritList;
      g_critRotateIdx = 0;
    }
    if (warnListChanged) {
      g_warnList = newWarnList;
      g_warnRotateIdx = 0;
    }
    g_lastRotateMs = millis();

    USBSerial.printf("[Alert] Sending UI update: crit=%d warn=%d critActive=%d\n",
                     counts.criticalCount, counts.warningCount, (counts.criticalCount > 0));

    sendUIUpdate();
  }

  // Build snapshot for carousel (critical preferred)
  AlertSnapshot snap{};
  snap.epoch = ++g_epoch;
  snap.criticalMode = (!newCritList.empty()) ? 1 : 0;
  const std::vector<SensorID>& srcList = (!newCritList.empty()) ? newCritList : newWarnList;
  snap.count = (uint8_t)std::min<size_t>(srcList.size(), MAX_ALERT_ITEMS);
  for (uint8_t i = 0; i < snap.count; ++i) {
    snap.ids[i] = srcList[i];
  }

  if (alertCarouselQueue) {
    xQueueOverwrite(alertCarouselQueue, &snap);
  }
}

/**
 * Handle vibration alerts based on state transitions
 * Note: This now only handles one-shot vibration patterns.
 * Critical alert border/vibration is handled by UI task based on criticalAlertsActive flag.
 */
static void handleAlertVibration(const AlertCounts& newCounts, const AlertCounts& previousCounts) {
  // Handle warning transitions (only when no critical alerts)
  if (newCounts.criticalCount == 0) {
    const int deltaWarnings = static_cast<int>(newCounts.warningCount) - static_cast<int>(previousCounts.warningCount);
    if (deltaWarnings > 0) {
      // Any increase in warnings: another double pulse
      vTaskDelay(pdMS_TO_TICKS(100));
      executeVibePattern(VIBE_DOUBLE_PULSE);
    }
  }
}
