#ifndef LVGL_UPDATES_H
#define LVGL_UPDATES_H

#include <lvgl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "../bms.h"
#include "../structs.h"
#include "lvgl_main_screen.h"

// Global mutex for LVGL thread safety
extern SemaphoreHandle_t lvglMutex;

// Flash timer globals for animations
extern lv_timer_t* cruise_flash_timer;
extern int cruise_flash_count;
extern bool isFlashingCruiseIcon;
extern lv_color_t original_cruise_icon_color;

extern lv_timer_t* arm_fail_flash_timer;
extern int arm_fail_flash_count;
extern bool isFlashingArmFailIcon;
extern lv_color_t original_arm_fail_icon_color;

extern lv_timer_t* critical_border_flash_timer;
extern bool isFlashingCriticalBorder;

// Main update function
void updateLvglMainScreen(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
);

// Helper update functions
void updateClimbRateIndicator(float climbRate);

// Test function
void updateLvglMainScreenWithTestData(const STR_DEVICE_DATA_140_V1& deviceData);

// Flash animation functions
void startCruiseIconFlash();
void startArmFailIconFlash();
void startCriticalBorderFlash();
void stopCriticalBorderFlash();
bool isCriticalBorderFlashing();

#endif // LVGL_UPDATES_H
