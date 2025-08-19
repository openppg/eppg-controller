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

// Rotation state for display
static std::vector<SensorID> g_activeList;
static bool g_showingCrit = false;
static size_t g_rotateIdx = 0;
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

  // Init the new alert service
  initCriticalAlertService();

  // Create aggregation task (low priority)
  xTaskCreate(alertAggregationTask, "AlertAgg", 4096, NULL, 1, &alertAggregationTaskHandle);
  USBSerial.println("[AlertDisplay] Init complete");
}

void sendAlertEvent(SensorID id, AlertLevel level) {
  if (!alertEventQueue) return;
  AlertEvent ev{ id, level, millis() };
  xQueueSend(alertEventQueue, &ev, 0);  // best-effort, drop if full
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

    // Handle rotation every 2s if active list not empty
    if (!g_activeList.empty()) {
      unsigned long now = millis();
      if (now - g_lastRotateMs >= 2000) {
        g_lastRotateMs = now;
        g_rotateIdx = (g_rotateIdx + 1) % g_activeList.size();

        // Send unified update with current counts and display info
        AlertUIUpdate update;
        update.counts = g_currentCounts;
        update.showDisplay = true;
        update.displayId = g_activeList[g_rotateIdx];
        update.displayLevel = g_currentLevels[g_activeList[g_rotateIdx]];
        update.displayCritical = g_showingCrit;
        update.criticalAlertsActive = (g_currentCounts.criticalCount > 0);
        update.updateEpoch = g_epoch;
        if (alertUIQueue) {
          xQueueOverwrite(alertUIQueue, &update);
        }
      }
    } else {
      // If list empty ensure label hidden once
      static bool hideSent = false;
      if (!hideSent) {
        AlertUIUpdate update;
        update.counts = g_currentCounts;
        update.showDisplay = false;
        update.criticalAlertsActive = (g_currentCounts.criticalCount > 0);
        update.updateEpoch = g_epoch;
        if (alertUIQueue) xQueueOverwrite(alertUIQueue, &update);
        hideSent = true;
      }
    }
  }
}

static void recalcCountsAndPublish() {
  AlertCounts counts{0, 0};
  std::vector<SensorID> critList;
  std::vector<SensorID> warnList;
  for (const auto& kv : g_currentLevels) {
    switch (kv.second) {
      case AlertLevel::WARN_LOW:
      case AlertLevel::WARN_HIGH:
        warnList.push_back(kv.first);
        counts.warningCount++;
        break;
      case AlertLevel::CRIT_LOW:
      case AlertLevel::CRIT_HIGH:
        critList.push_back(kv.first);
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

  // Update active list for display rotation
  bool newShowingCrit = !critList.empty();
  const std::vector<SensorID>& newList = newShowingCrit ? critList : warnList;

  bool listChanged = (newShowingCrit != g_showingCrit) || (newList != g_activeList);

  // Send unified update when counts change or display list changes
  if (countsChanged || listChanged) {
    if (listChanged) {
      g_activeList = newList;
      g_showingCrit = newShowingCrit;
      g_rotateIdx = 0;
      g_lastRotateMs = millis();
    }

    AlertUIUpdate update;
    update.counts = g_currentCounts;
    update.criticalAlertsActive = (g_currentCounts.criticalCount > 0);
    update.updateEpoch = g_epoch;

    if (g_activeList.empty()) {
      update.showDisplay = false;
    } else {
      update.showDisplay = true;
      update.displayId = g_activeList[g_rotateIdx];
      update.displayLevel = g_currentLevels[g_activeList[g_rotateIdx]];
      update.displayCritical = g_showingCrit;
    }

    USBSerial.printf("[Alert] Sending UI update: crit=%d warn=%d critActive=%d\n",
                     update.counts.criticalCount, update.counts.warningCount, update.criticalAlertsActive);

    if (alertUIQueue) {
      xQueueOverwrite(alertUIQueue, &update);
    }
  }

  // Build snapshot for carousel (critical preferred)
  AlertSnapshot snap{};
  snap.epoch = ++g_epoch;
  snap.criticalMode = (!critList.empty()) ? 1 : 0;
  const std::vector<SensorID>& srcList = (!critList.empty()) ? critList : warnList;
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
  if (newCounts.criticalCount == 0 &&
      previousCounts.warningCount == 0 && newCounts.warningCount > 0) {
    // Short delay to sync with UI
    vTaskDelay(pdMS_TO_TICKS(100));
    // Transition from 0 warnings to >0 warnings - trigger double pulse
    executeVibePattern(VIBE_DOUBLE_PULSE);
  }
}
