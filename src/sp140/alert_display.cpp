#include "../../inc/sp140/alert_display.h"
#include <Arduino.h>
#include <freertos/queue.h>
#include "../../inc/sp140/lvgl/lvgl_display.h"
#include <map>
#include <algorithm>

// ------------ Globals -------------
QueueHandle_t alertEventQueue = NULL;
QueueHandle_t alertCountQueue = NULL;
QueueHandle_t alertCarouselQueue = NULL;
TaskHandle_t alertAggregationTaskHandle = NULL;

// Internal state: current alert levels per sensor
static std::map<SensorID, AlertLevel> g_currentLevels;
static AlertCounts g_currentCounts = {0, 0};
static uint32_t g_epoch = 0;

// Forward declarations
static void alertAggregationTask(void* parameter);
static void recalcCountsAndPublish();

// ------------ Public helpers -------------
void initAlertDisplay() {
  // Create queues – small, non-blocking
  alertEventQueue  = xQueueCreate(10, sizeof(AlertEvent));
  alertCountQueue  = xQueueCreate(1,  sizeof(AlertCounts));
  alertCarouselQueue = xQueueCreate(1, sizeof(AlertSnapshot));

  if (!alertEventQueue || !alertCountQueue || !alertCarouselQueue) {
    USBSerial.println("[AlertDisplay] Failed creating queues");
    return;
  }

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
    // Wait for next event (block 500ms), allow timeout to avoid watchdog issues
    if (xQueueReceive(alertEventQueue, &ev, pdMS_TO_TICKS(500)) == pdTRUE) {
      // Update map
      if (ev.level == AlertLevel::OK) {
        g_currentLevels.erase(ev.sensorId);
      } else {
        g_currentLevels[ev.sensorId] = ev.level;
      }
      recalcCountsAndPublish();
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

  if (counts.warningCount != g_currentCounts.warningCount ||
      counts.criticalCount != g_currentCounts.criticalCount) {
    g_currentCounts = counts;
    if (alertCountQueue) {
      xQueueOverwrite(alertCountQueue, &g_currentCounts);
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
