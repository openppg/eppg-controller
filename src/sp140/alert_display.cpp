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
QueueHandle_t alertCountQueue = NULL;
QueueHandle_t alertCarouselQueue = NULL;
QueueHandle_t alertDisplayQueue = NULL;
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
  alertEventQueue  = xQueueCreate(10, sizeof(AlertEvent));
  alertCountQueue  = xQueueCreate(1,  sizeof(AlertCounts));
  alertCarouselQueue = xQueueCreate(1, sizeof(AlertSnapshot));
  alertDisplayQueue = xQueueCreate(1, sizeof(AlertDisplayMsg)); // New queue for display rotation

  if (!alertEventQueue || !alertCountQueue || !alertCarouselQueue || !alertDisplayQueue) {
    USBSerial.println("[AlertDisplay] Failed creating queues");
    return;
  }

  // Init the new alert service
  initCriticalAlertService();

  // Create aggregation task (low priority)
  xTaskCreate(alertAggregationTask, "AlertAgg", 3072, NULL, 1, &alertAggregationTaskHandle);
  USBSerial.println("[AlertDisplay] Init complete");
}

void sendAlertEvent(SensorID id, AlertLevel level) {
  if (!alertEventQueue) return;
  AlertEvent ev{ id, level, millis() };
  xQueueSend(alertEventQueue, &ev, 0); // best-effort, drop if full
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

        AlertDisplayMsg msg;
        msg.show = true;
        msg.id = g_activeList[g_rotateIdx];
        msg.level = g_currentLevels[g_activeList[g_rotateIdx]];  // Get the alert level for this sensor
        msg.critical = g_showingCrit;
        if (alertDisplayQueue) {
          xQueueOverwrite(alertDisplayQueue, &msg);
        }
      }
    }
    else {
      // If list empty ensure label hidden once
      static bool hideSent = false;
      if (!hideSent) {
        AlertDisplayMsg msg{};
        msg.show = false;
        if (alertDisplayQueue) xQueueOverwrite(alertDisplayQueue,&msg);
        hideSent = true;
      }
    }
  }
}

static void recalcCountsAndPublish() {
  AlertCounts counts{0,0};
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

  if (counts.warningCount != g_currentCounts.warningCount ||
      counts.criticalCount != g_currentCounts.criticalCount) {
    g_currentCounts = counts;
    if (alertCountQueue) {
      xQueueOverwrite(alertCountQueue, &g_currentCounts);
    }
  }

  // Update active list for display rotation
  bool newShowingCrit = !critList.empty();
  const std::vector<SensorID>& newList = newShowingCrit ? critList : warnList;

  bool listChanged = (newShowingCrit != g_showingCrit) || (newList != g_activeList);

  if (listChanged) {
    g_activeList = newList;
    g_showingCrit = newShowingCrit;
    g_rotateIdx = 0;
    g_lastRotateMs = millis();

    AlertDisplayMsg msg;
    if (g_activeList.empty()) {
      msg.show = false;
    } else {
      msg.show = true;
      msg.id = g_activeList[g_rotateIdx];
      msg.level = g_currentLevels[g_activeList[g_rotateIdx]];  // Get the alert level for this sensor
      msg.critical = g_showingCrit;
    }
    if (alertDisplayQueue) {
      xQueueOverwrite(alertDisplayQueue, &msg);
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

  // Rotate display queue
  AlertSnapshot currentDisplaySnap;
  if (xQueuePeek(alertDisplayQueue, &currentDisplaySnap, 0) == pdTRUE) {
    // If the current display snapshot is the same as the carousel snapshot,
    // we need to rotate it to show the next set of alerts.
    // This logic is simplified here; in a real system, you'd manage a queue of snapshots.
    // For now, we just overwrite the display queue with the current carousel snapshot.
    // A more robust solution would involve a separate queue for display snapshots.
    xQueueOverwrite(alertDisplayQueue, &snap);
  } else {
    // If the display queue is empty, just overwrite it with the current carousel snapshot.
    xQueueOverwrite(alertDisplayQueue, &snap);
  }
}

/**
 * Handle vibration alerts based on state transitions
 */
static void handleAlertVibration(const AlertCounts& newCounts, const AlertCounts& previousCounts) {
  if (newCounts.criticalCount > 0) {
    // Use the new synchronized alert service
    if (!isCriticalAlertActive()) {
      startCriticalAlerts();
    }
  } else {
    // Stop synchronized alerts if no critical alerts remain
    if (isCriticalAlertActive()) {
      stopCriticalAlerts();
    }

    // Handle warning transitions (only when no critical alerts)
    if (previousCounts.warningCount == 0 && newCounts.warningCount > 0) {
      // Transition from 0 warnings to >0 warnings - trigger double pulse
      executeVibePattern(VIBE_DOUBLE_PULSE);
    }
  }
}
