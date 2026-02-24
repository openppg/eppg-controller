#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdio.h>
#include <string.h>

#include "../../inc/sp140/device_state.h"
#include "../../inc/sp140/globals.h"
#include "../../inc/sp140/lvgl/lvgl_main_screen.h"
#include "../../inc/sp140/lvgl/lvgl_updates.h"
#include "../../inc/sp140/notification.h"
#include "../../inc/sp140/vibration_pwm.h"

extern volatile DeviceState currentState;

static QueueHandle_t notificationQueue = NULL;
static bool bingoFired = false;
static bool bingoSeenAboveThreshold = false;

static lv_obj_t* notificationOverlay = NULL;
static lv_obj_t* notificationLabel = NULL;
static lv_timer_t* notificationTimer = NULL;
static bool notificationActive = false;

static void hideNotificationOverlay() {
  if (notificationOverlay != NULL) {
    lv_obj_add_flag(notificationOverlay, LV_OBJ_FLAG_HIDDEN);
  }
  notificationActive = false;
}

static void notificationTimerCb(lv_timer_t* timer) {
  (void)timer;
  hideNotificationOverlay();

  if (notificationTimer != NULL) {
    lv_timer_del(notificationTimer);
    notificationTimer = NULL;
  }
}

void initNotifications() {
  if (notificationQueue == NULL) {
    notificationQueue = xQueueCreate(3, sizeof(Notification));
    if (notificationQueue == NULL) {
      USBSerial.println("[Notify] Failed to create notification queue");
    }
  }
}

void setupNotificationOverlay(bool darkMode) {
  if (main_screen == NULL || notificationOverlay != NULL) {
    return;
  }

  notificationOverlay = lv_obj_create(main_screen);
  lv_obj_set_size(notificationOverlay, SCREEN_WIDTH, 40);
  lv_obj_set_pos(notificationOverlay, 0, 0);
  lv_obj_set_style_radius(notificationOverlay, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(notificationOverlay, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(
    notificationOverlay,
    darkMode ? lv_color_make(180, 90, 0) : lv_color_make(230, 120, 0),
    LV_PART_MAIN);
  lv_obj_set_style_bg_opa(notificationOverlay, LV_OPA_COVER, LV_PART_MAIN);

  notificationLabel = lv_label_create(notificationOverlay);
  lv_obj_set_style_text_font(notificationLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(notificationLabel, lv_color_white(), 0);
  lv_obj_set_style_text_align(notificationLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_width(notificationLabel, SCREEN_WIDTH - 8);
  lv_obj_align(notificationLabel, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(notificationLabel, "");

  lv_obj_add_flag(notificationOverlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(notificationOverlay);
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
  if (notificationQueue == NULL || notificationOverlay == NULL || notificationLabel == NULL) {
    return;
  }

  if (notificationActive) {
    return;
  }

  Notification notification = {};
  if (xQueueReceive(notificationQueue, &notification, 0) != pdTRUE) {
    return;
  }

  lv_color_t backgroundColor = lv_color_make(0, 110, 200);
  if (notification.severity == NotifySeverity::CAUTION) {
    backgroundColor = lv_color_make(210, 105, 0);
  }
  lv_obj_set_style_bg_color(notificationOverlay, backgroundColor, LV_PART_MAIN);
  lv_label_set_text(notificationLabel, notification.message);
  lv_obj_clear_flag(notificationOverlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(notificationOverlay);
  notificationActive = true;

  if (notification.vibrate) {
    executeVibePattern(VIBE_DOUBLE_PULSE);
  }

  if (notificationTimer != NULL) {
    lv_timer_del(notificationTimer);
    notificationTimer = NULL;
  }

  const uint16_t duration = (notification.displayMs > 0) ? notification.displayMs : 3000;
  notificationTimer = lv_timer_create(notificationTimerCb, duration, NULL);
  if (notificationTimer == NULL) {
    hideNotificationOverlay();
  }
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
