#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdio.h>
#include <string.h>

#include "../../inc/sp140/device_state.h"
#include "../../inc/sp140/globals.h"
#include "../../inc/sp140/lvgl/lvgl_banner.h"
#include "../../inc/sp140/notification.h"
#include "../../inc/sp140/vibration_pwm.h"

extern volatile DeviceState currentState;

static QueueHandle_t notificationQueue = NULL;
static bool bingoFired = false;
static bool bingoSeenAboveThreshold = false;

void initNotifications() {
  if (notificationQueue == NULL) {
    notificationQueue = xQueueCreate(3, sizeof(Notification));
    if (notificationQueue == NULL) {
      USBSerial.println("[Notify] Failed to create notification queue");
    }
  }
}

void queueNotification(const char* msg, NotifySeverity sev, uint16_t durationMs, bool vibrate) {
  if (notificationQueue == NULL || msg == NULL) {
    return;
  }

  Notification notification = {};
  strncpy(notification.message, msg, sizeof(notification.message) - 1);
  notification.message[sizeof(notification.message) - 1] = '\0';
  notification.severity = sev;
  notification.displayMs = durationMs;
  notification.vibrate = vibrate;
  xQueueSend(notificationQueue, &notification, 0);
}

void processNotificationQueue() {
  if (notificationQueue == NULL || isBannerActive()) {
    return;
  }

  Notification notification = {};
  if (xQueueReceive(notificationQueue, &notification, 0) != pdTRUE) {
    return;
  }

  // Dispatch to UI
  BannerColor color = (notification.severity == NotifySeverity::CAUTION)
    ? BannerColor::CAUTION : BannerColor::INFO;
  showBanner(notification.message, color, notification.displayMs);

  // Dispatch to vibration
  if (notification.vibrate) {
    executeVibePattern(VIBE_DOUBLE_PULSE);
  }

  // Future: dispatch to logging
}

void resetFlightNotifications() {
  bingoFired = false;
  bingoSeenAboveThreshold = false;
}

void checkArmNotifications() {
  if (shouldNotifyLowBattery(unifiedBatteryData.soc)) {
    char msg[20];
    snprintf(msg, sizeof(msg), "LOW BATT %d%%", (int)unifiedBatteryData.soc);
    queueNotification(msg, NotifySeverity::CAUTION, 3000, true);
  }
}

void checkFlightNotifications() {
  if (currentState == DISARMED) {
    return;
  }

  const float soc = unifiedBatteryData.soc;
  if (soc > 50.0f) {
    bingoSeenAboveThreshold = true;
  }

  if (shouldNotifyBingo(soc, bingoFired, bingoSeenAboveThreshold)) {
    bingoFired = true;
    queueNotification("BINGO FUEL", NotifySeverity::INFO, 3000, true);
  }
}
