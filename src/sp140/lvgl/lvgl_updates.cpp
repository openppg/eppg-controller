#include "../../../inc/sp140/lvgl/lvgl_updates.h"
#include "../../../inc/sp140/esp32s3-config.h"
#include "../../../inc/sp140/monitor_config.h"  // For direct threshold access
#include "../../../inc/sp140/globals.h"
#include "../../../inc/sp140/vibration_pwm.h"
#include "../../../inc/sp140/shared-config.h"
#include "../../../inc/sp140/ble.h"
#include "../../../inc/sp140/ble/ble_core.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Change-detecting setters for the per-frame update path.
// LVGL invalidates (and re-flushes over SPI) any object touched by
// lv_label_set_text() or a style setter even when the new value is identical
// to the current one. updateLvglMainScreen() runs every UI frame, so
// unguarded setters redraw nearly the whole screen at the full frame rate.
// These helpers skip the LVGL call when nothing actually changed, so only
// areas with new content get invalidated and flushed.

static void setLabelText(lv_obj_t* label, const char* text) {
  if (label == NULL || text == NULL) return;
  const char* current = lv_label_get_text(label);
  if (current != NULL && strcmp(current, text) == 0) return;
  lv_label_set_text(label, text);
}

static void setLabelTextColor(lv_obj_t* label, lv_color_t color) {
  if (label == NULL) return;
  if (lv_color_eq(lv_obj_get_style_text_color(label, LV_PART_MAIN), color)) return;
  lv_obj_set_style_text_color(label, color, 0);
}

static void setBgColor(lv_obj_t* obj, lv_color_t color, lv_part_t part) {
  if (obj == NULL) return;
  if (lv_color_eq(lv_obj_get_style_bg_color(obj, part), color)) return;
  lv_obj_set_style_bg_color(obj, color, part);
}

static void setBgOpa(lv_obj_t* obj, lv_opa_t opa) {
  if (obj == NULL) return;
  if (lv_obj_get_style_bg_opa(obj, LV_PART_MAIN) == opa) return;
  lv_obj_set_style_bg_opa(obj, opa, LV_PART_MAIN);
}

static void setImageRecolor(lv_obj_t* obj, lv_color_t color) {
  if (obj == NULL) return;
  if (lv_color_eq(lv_obj_get_style_image_recolor(obj, LV_PART_MAIN), color)) return;
  lv_obj_set_style_image_recolor(obj, color, LV_PART_MAIN);
}

// Last-applied UI state that cannot be read back from the widgets (label
// alignment mode, which bg style is attached). Kept at file scope so
// resetLvglUpdateCache() can invalidate it whenever the screen widgets are
// (re)created - e.g. setupMainScreen() on boot or between screenshot tests.
static int8_t lastVoltLeftMode = -1;       // 0 = BMS, 1 = "NO BMS", 2 = none
static uint8_t lastBattTempLevel = 0xFF;   // 0 = normal, 1 = warning, 2 = critical
static uint8_t lastEscTempLevel = 0xFF;
static uint8_t lastMotorTempLevel = 0xFF;

void resetLvglUpdateCache() {
  lastVoltLeftMode = -1;
  lastBattTempLevel = 0xFF;
  lastEscTempLevel = 0xFF;
  lastMotorTempLevel = 0xFF;
}

// Flash timer globals - definitions
lv_timer_t* cruise_flash_timer = NULL;
int cruise_flash_count = 0;
bool isFlashingCruiseIcon = false;
lv_color_t original_cruise_icon_color;

lv_timer_t* arm_fail_flash_timer = NULL;
int arm_fail_flash_count = 0;
bool isFlashingArmFailIcon = false;
lv_color_t original_arm_fail_icon_color;

lv_timer_t* critical_border_flash_timer = NULL;
bool isFlashingCriticalBorder = false;

lv_timer_t* ble_pairing_flash_timer = NULL;
bool isFlashingBLEPairingIcon = false;

// Timer callback declarations
static void cruise_flash_timer_cb(lv_timer_t* timer);
static void arm_fail_flash_timer_cb(lv_timer_t* timer);
static void critical_border_flash_timer_cb(lv_timer_t* timer);
static void ble_pairing_flash_timer_cb(lv_timer_t* timer);

// --- Cruise Icon Flashing Implementation ---
static void cruise_flash_timer_cb(lv_timer_t* timer) {
  // This callback runs within the LVGL task handler, which is already protected by lvglMutex

  if (cruise_icon_img == NULL) {
    // Should not happen, but safety check
    if (cruise_flash_timer != NULL) {
      lv_timer_del(cruise_flash_timer);
      cruise_flash_timer = NULL;
    }
    cruise_flash_count = 0;
    isFlashingCruiseIcon = false;
    return;
  }

  // Toggle visibility
  if (lv_obj_has_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_remove_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
  }

  cruise_flash_count++;

  // Check if flashing is complete (3 flashes = 6 toggles)
  if (cruise_flash_count >= 6) {
    lv_timer_del(cruise_flash_timer);
    cruise_flash_timer = NULL;
    cruise_flash_count = 0;
    isFlashingCruiseIcon = false;
    // Ensure icon is hidden after flashing as cruise is off
    lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
  }
}

void startCruiseIconFlash() {
  // This function can be called from other tasks, so protect with mutex
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {  // Use a timeout
    if (cruise_icon_img == NULL) {
      xSemaphoreGive(lvglMutex);
      return;  // Can't flash if icon doesn't exist
    }

    // If a flash timer is already running, delete it first
    if (cruise_flash_timer != NULL) {
      lv_timer_del(cruise_flash_timer);
      cruise_flash_timer = NULL;
    }

    // Reset state and start flashing
    cruise_flash_count = 0;
    isFlashingCruiseIcon = true;

    // Start with the icon visible
    lv_obj_remove_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);

    // Create the timer (250ms interval for on/off cycle)
    cruise_flash_timer = lv_timer_create(cruise_flash_timer_cb, 250, NULL);
    if (cruise_flash_timer == NULL) {
      // Failed to create timer, reset state
      isFlashingCruiseIcon = false;
      lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide it again
      lv_obj_set_style_image_recolor(cruise_icon_img, original_cruise_icon_color, LV_PART_MAIN);  // Restore color on failure
      USBSerial.println("Error: Failed to create cruise flash timer!");
    } else {
      // Set icon to red for flashing
      lv_obj_set_style_image_recolor(cruise_icon_img, LVGL_RED, LV_PART_MAIN);
    }

    xSemaphoreGive(lvglMutex);
  } else {
     USBSerial.println("Warning: Failed to acquire LVGL mutex for startCruiseIconFlash");
  }
}

// --- Arm Fail Icon Flashing Implementation ---
static void arm_fail_flash_timer_cb(lv_timer_t* timer) {
  // This callback runs within the LVGL task handler, which is already protected by lvglMutex

  if (arm_fail_warning_icon_img == NULL) {
    // Safety check
    if (arm_fail_flash_timer != NULL) {
      lv_timer_del(arm_fail_flash_timer);
      arm_fail_flash_timer = NULL;
    }
    arm_fail_flash_count = 0;
    isFlashingArmFailIcon = false;
    return;
  }

  // Toggle visibility
  if (lv_obj_has_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_remove_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
  }

  arm_fail_flash_count++;

  // Check if flashing is complete (3 flashes = 6 toggles)
  if (arm_fail_flash_count >= 6) {
    lv_timer_del(arm_fail_flash_timer);
    arm_fail_flash_timer = NULL;
    arm_fail_flash_count = 0;
    isFlashingArmFailIcon = false;
    // Ensure icon is hidden after flashing
    lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
    // Restore original color after flashing is done
    lv_obj_set_style_image_recolor(arm_fail_warning_icon_img, original_arm_fail_icon_color, LV_PART_MAIN);
  }
}

void startArmFailIconFlash() {
  // This function can be called from other tasks, so protect with mutex
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {  // Use a timeout
    if (arm_fail_warning_icon_img == NULL) {
      xSemaphoreGive(lvglMutex);
      return;  // Can't flash if icon doesn't exist
    }

    // If a flash timer is already running, delete it first
    if (arm_fail_flash_timer != NULL) {
      lv_timer_del(arm_fail_flash_timer);
      arm_fail_flash_timer = NULL;
    }

    // Reset state and start flashing
    arm_fail_flash_count = 0;
    isFlashingArmFailIcon = true;

    // Start with the icon visible
    lv_obj_remove_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);

    // Set icon to red for flashing
    lv_obj_set_style_image_recolor(arm_fail_warning_icon_img, LVGL_RED, LV_PART_MAIN);

    // Create the timer (250ms interval for on/off cycle)
    arm_fail_flash_timer = lv_timer_create(arm_fail_flash_timer_cb, 250, NULL);
    if (arm_fail_flash_timer == NULL) {
      // Failed to create timer, reset state
      isFlashingArmFailIcon = false;
      lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide it again
      lv_obj_set_style_image_recolor(arm_fail_warning_icon_img, original_arm_fail_icon_color, LV_PART_MAIN);  // Restore color
      USBSerial.println("Error: Failed to create arm fail flash timer!");
    }

    xSemaphoreGive(lvglMutex);
  } else {
     USBSerial.println("Warning: Failed to acquire LVGL mutex for startArmFailIconFlash");
  }
}

// --- Critical Alert Border Flashing Implementation ---
static void critical_border_flash_timer_cb(lv_timer_t* timer) {
  // This callback runs within the LVGL task handler, so no mutex needed here.
  if (critical_border != NULL) {
    // Toggle opacity: 300ms on (opaque), 700ms off (transparent)
    uint8_t current_opa = lv_obj_get_style_border_opa(critical_border, LV_PART_MAIN);
    if (current_opa == LV_OPA_100) {
      lv_obj_set_style_border_opa(critical_border, LV_OPA_0, LV_PART_MAIN);
      lv_timer_set_period(timer, 700);  // Off duration
      // Invalidate entire screen when hiding to ensure clean removal of border pixels
      lv_obj_invalidate(lv_screen_active());
    } else {
      lv_obj_set_style_border_opa(critical_border, LV_OPA_100, LV_PART_MAIN);
      lv_timer_set_period(timer, 300);  // On duration

      // Trigger vibration pulse in sync with border "on"
      if (ENABLE_VIBE) {
        pulseVibration(300, 200);  // 300ms pulse, intensity 200
      }
      // Invalidate the border area when showing
      lv_obj_invalidate(critical_border);
    }
    // Force immediate refresh to minimize tearing
    lv_refr_now(lv_display_get_default());
  }
}

void startCriticalBorderFlash() {
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (critical_border != NULL && !isFlashingCriticalBorder) {
      isFlashingCriticalBorder = true;
      lv_obj_set_style_border_opa(critical_border, LV_OPA_100, LV_PART_MAIN);  // Start visible
      critical_border_flash_timer = lv_timer_create(critical_border_flash_timer_cb, 300, NULL);
    }
    xSemaphoreGive(lvglMutex);
  }
}

void stopCriticalBorderFlash() {
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (critical_border_flash_timer != NULL) {
      lv_timer_del(critical_border_flash_timer);
      critical_border_flash_timer = NULL;
    }
    if (critical_border != NULL) {
      lv_obj_set_style_border_opa(critical_border, LV_OPA_0, LV_PART_MAIN);
      // Invalidate entire screen to ensure clean removal
      lv_obj_invalidate(lv_screen_active());
    }
    isFlashingCriticalBorder = false;
    xSemaphoreGive(lvglMutex);
  }
}

bool isCriticalBorderFlashing() {
  return isFlashingCriticalBorder;
}

// Direct control functions for use within UI task (no mutex)
void startCriticalBorderFlashDirect() {
  if (critical_border != NULL && !isFlashingCriticalBorder) {
    isFlashingCriticalBorder = true;
    lv_obj_set_style_border_opa(critical_border, LV_OPA_100, LV_PART_MAIN);  // Start visible
    lv_obj_invalidate(critical_border);  // Ensure clean initial draw
    critical_border_flash_timer = lv_timer_create(critical_border_flash_timer_cb, 300, NULL);
    // Force immediate refresh for clean start
    lv_refr_now(lv_display_get_default());
  }
}

void stopCriticalBorderFlashDirect() {
  if (critical_border_flash_timer != NULL) {
    lv_timer_del(critical_border_flash_timer);
    critical_border_flash_timer = NULL;
  }
  if (critical_border != NULL) {
    lv_obj_set_style_border_opa(critical_border, LV_OPA_0, LV_PART_MAIN);
    lv_obj_invalidate(lv_screen_active());  // Ensure clean removal of border
    // Force immediate refresh for clean stop
    lv_refr_now(lv_display_get_default());
  }
  isFlashingCriticalBorder = false;
}

// --- BLE Pairing Icon Flashing Implementation ---
static void ble_pairing_flash_timer_cb(lv_timer_t* timer) {
  if (ble_pairing_icon == NULL) {
    if (ble_pairing_flash_timer != NULL) {
      lv_timer_del(ble_pairing_flash_timer);
      ble_pairing_flash_timer = NULL;
    }
    isFlashingBLEPairingIcon = false;
    return;
  }

  if (lv_obj_has_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_remove_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
  }
}

void startBLEPairingIconFlash() {
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (ble_pairing_icon == NULL) {
      xSemaphoreGive(lvglMutex);
      return;
    }

    if (ble_pairing_flash_timer != NULL) {
      lv_timer_del(ble_pairing_flash_timer);
      ble_pairing_flash_timer = NULL;
    }

    isFlashingBLEPairingIcon = true;
    lv_obj_remove_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
    ble_pairing_flash_timer = lv_timer_create(ble_pairing_flash_timer_cb, 500, NULL);
    if (ble_pairing_flash_timer == NULL) {
      isFlashingBLEPairingIcon = false;
      lv_obj_add_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
      USBSerial.println("Error: Failed to create BLE pairing flash timer!");
    }

    xSemaphoreGive(lvglMutex);
  }
}

void showBLEStatusIcon() {
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (ble_pairing_flash_timer != NULL) {
      lv_timer_del(ble_pairing_flash_timer);
      ble_pairing_flash_timer = NULL;
    }
    if (ble_pairing_icon != NULL) {
      lv_obj_remove_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
    }
    isFlashingBLEPairingIcon = false;
    xSemaphoreGive(lvglMutex);
  }
}

void hideBLEStatusIcon() {
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (ble_pairing_flash_timer != NULL) {
      lv_timer_del(ble_pairing_flash_timer);
      ble_pairing_flash_timer = NULL;
    }
    if (ble_pairing_icon != NULL) {
      lv_obj_add_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
    }
    isFlashingBLEPairingIcon = false;
    xSemaphoreGive(lvglMutex);
  }
}

void stopBLEPairingIconFlash() {
  hideBLEStatusIcon();
}

// Update the climb rate indicator
void updateClimbRateIndicator(float climbRate) {
  // Vario display scale: 6 segments per direction cover a +/-3 m/s climb/descent
  // envelope. The raw value is NOT clamped here - sectionsToFill is capped at 6
  // below, so rates beyond +/-3 m/s simply pin the gauge to full deflection.
  const float kVarioSegment = 3.0f / 6.0f;            // m/s per segment (0.5)
  const float kVarioDeadzone = kVarioSegment / 2.0f;  // neutral zone near 0 (0.25 m/s)

  // Define colors for positive climb rates (from center upward)
  lv_color_t positive_colors[6] = {
    lv_color_make(0, 255, 0),      // Bright green
    lv_color_make(255, 255, 0),    // Yellow
    lv_color_make(255, 165, 0),    // Orange
    lv_color_make(255, 165, 0),    // Orange (again)
    lv_color_make(255, 0, 255),    // Magenta
    lv_color_make(255, 0, 255)     // Magenta (again)
  };

  // Define colors for negative climb rates (from center downward)
  lv_color_t negative_colors[6] = {
    lv_color_make(173, 216, 230),  // Light blue
    lv_color_make(173, 216, 230),  // Light blue (again)
    lv_color_make(70, 130, 180),   // Darker blue
    lv_color_make(70, 130, 180),   // Darker blue (again)
    lv_color_make(75, 0, 130),     // Dark purple
    lv_color_make(75, 0, 130)      // Dark purple (again)
  };

<<<<<<< HEAD
  // Compute the desired state for all 12 sections, then diff-apply so only
  // sections that actually changed are invalidated and redrawn.
  lv_opa_t section_opa[12];
  lv_color_t section_color[12];
  for (int i = 0; i < 12; i++) {
    section_opa[i] = LV_OPA_0;
    section_color[i] = lv_color_black();  // unused while transparent
  }

=======
>>>>>>> origin/master
  if (climbRate >= kVarioDeadzone) {
    // Positive climb rate - fill sections above center line
    int sectionsToFill = (int)(climbRate / kVarioSegment);
    if (sectionsToFill > 6) sectionsToFill = 6;

    for (int i = 0; i < sectionsToFill; i++) {
      int sectionIndex = 5 - i;  // Section 5 is closest to center line, going up to 0
      section_opa[sectionIndex] = LV_OPA_100;
      section_color[sectionIndex] = positive_colors[i];
    }
  } else if (climbRate <= -kVarioDeadzone) {
    // Negative climb rate - fill sections below center line
    int sectionsToFill = (int)(-climbRate / kVarioSegment);
    if (sectionsToFill > 6) sectionsToFill = 6;

    for (int i = 0; i < sectionsToFill; i++) {
      int sectionIndex = 6 + i;  // Section 6 is closest to center line, going down to 11
      section_opa[sectionIndex] = LV_OPA_100;
      section_color[sectionIndex] = negative_colors[i];
    }
  }
  // If |climb rate| is within kVarioDeadzone, no sections are filled (neutral)
<<<<<<< HEAD

  for (int i = 0; i < 12; i++) {
    if (climb_rate_fill_sections[i] == NULL) continue;
    // Only recolor visible sections; color is irrelevant while transparent
    if (section_opa[i] == LV_OPA_100) {
      setBgColor(climb_rate_fill_sections[i], section_color[i], LV_PART_MAIN);
    }
    setBgOpa(climb_rate_fill_sections[i], section_opa[i]);
  }
=======
>>>>>>> origin/master
}

void updateLvglMainScreen(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
) {
  bool darkMode = (deviceData.theme == 1);
  float batteryPercent = unifiedBatteryData.soc;
  float totalVolts = unifiedBatteryData.volts;
  float lowestCellV = bmsTelemetry.lowest_cell_voltage;
  // Calculate battery temp from connected T1-T4 cell probes only.
  const float cellTemps[] = {
    bmsTelemetry.t1_temperature,
    bmsTelemetry.t2_temperature,
    bmsTelemetry.t3_temperature,
    bmsTelemetry.t4_temperature
  };
  float batteryTemp = NAN;
  bool hasValidBatteryTemp = false;
  for (float cellTemp : cellTemps) {
    if (isnan(cellTemp)) {
      continue;
    }

    if (!hasValidBatteryTemp || cellTemp > batteryTemp) {
      batteryTemp = cellTemp;
      hasValidBatteryTemp = true;
    }
  }
  // Show whichever ESC temperature is hotter - capacitor or MOSFET - and color
  // it against that sensor's own thresholds (cap and MOSFET have different
  // limits). Skip a sensor that reads NaN (invalid); if both are invalid
  // escTemp stays NaN and the tile shows "-".
  const float capTemp = escTelemetry.cap_temp;
  const float mosTemp = escTelemetry.mos_temp;
  float escTemp;
  const Thresholds* escTempThresholds;
  if (!isnan(mosTemp) && (isnan(capTemp) || mosTemp >= capTemp)) {
    escTemp = mosTemp;
    escTempThresholds = &escMosTempThresholds;
  } else {
    escTemp = capTemp;  // may be NaN if both sensors are invalid
    escTempThresholds = &escCapTempThresholds;
  }
  float motorTemp = escTelemetry.motor_temp;
  // Check if BMS or ESC is connected
  bool bmsConnected = (bmsTelemetry.bmsState == TelemetryState::CONNECTED);
  bool escConnected = (escTelemetry.escState == TelemetryState::CONNECTED);

  // Update battery bar and percentage
  if (bmsConnected && batteryPercent >= 0) {
    lv_bar_set_value(battery_bar, (int)batteryPercent, LV_ANIM_OFF);

    // Set color based on percentage
    lv_color_t batteryColor = LVGL_RED;
    if (batteryPercent > bmsSOCThresholds.warnLow) {
      batteryColor = LVGL_GREEN;
    } else if (batteryPercent >= bmsSOCThresholds.critLow) {
      batteryColor = LVGL_YELLOW;
    }

    setBgColor(battery_bar, batteryColor, LV_PART_INDICATOR);

    // Update battery percentage text
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%d%%", (int)batteryPercent);
    setLabelText(battery_label, buffer);
    setLabelTextColor(battery_label, LVGL_BLACK);
  } else if (escConnected) {
    // clear the battery bar, we handle voltage later
    lv_bar_set_value(battery_bar, 0, LV_ANIM_OFF);
  } else {
    lv_bar_set_value(battery_bar, 0, LV_ANIM_OFF);
    setLabelText(battery_label, "NO DATA");
    setLabelTextColor(battery_label, LVGL_RED);
  }

  // Update left voltage (cell voltage)
  if (voltage_left_label != NULL) {  // Check object exists
    // Re-align only on source change: lv_obj_align() invalidates the label
    // every call even when the resulting position is identical.
    if (bmsConnected) {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.2fv", lowestCellV);
        setLabelText(voltage_left_label, buffer);

        // Always use black text for better readability
        setLabelTextColor(voltage_left_label, LVGL_BLACK);
        if (lastVoltLeftMode != 0) {
          // Restore default position when BMS is connected
          lv_obj_align(voltage_left_label, LV_ALIGN_TOP_LEFT, 3, 12);
          lastVoltLeftMode = 0;
        }
    } else if (escConnected) {
        setLabelTextColor(voltage_left_label, LVGL_BLACK);
        setLabelText(voltage_left_label, "NO\nBMS");
        if (lastVoltLeftMode != 1) {
          // Move up by 10 pixels when showing NO BMS
          lv_obj_align(voltage_left_label, LV_ALIGN_TOP_LEFT, 3, 2);
          lastVoltLeftMode = 1;
        }
    } else {
        setLabelText(voltage_left_label, "");
        lastVoltLeftMode = 2;
    }
  }

  // Update right voltage (total voltage)
  // if esc connected, show esc voltage in center of screen
  if (voltage_right_label != NULL && battery_label != NULL) {  // Check objects exist
    if (bmsConnected) {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.0fv", totalVolts);
        setLabelText(voltage_right_label, buffer);
        // Ensure battery label shows percentage if BMS is connected
        if (batteryPercent >= 0) {
           char batt_buffer[10];
           snprintf(batt_buffer, sizeof(batt_buffer), "%d%%", (int)batteryPercent);
           setLabelText(battery_label, batt_buffer);
           setLabelTextColor(battery_label, LVGL_BLACK);
        }

    } else if (escConnected) {
        setLabelText(voltage_right_label, "");

        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.1fv", totalVolts);
        setLabelText(battery_label, buffer);
        setLabelTextColor(battery_label, LVGL_BLACK);
    } else {
        setLabelText(voltage_right_label, "");
    }
  }

  // Update power display with individual character positions.
  // Compute the desired text for every position first, then diff-apply: the
  // old clear-then-set pattern invalidated all 4 labels twice per frame.
  if (power_char_labels[0] != NULL) {  // Check if power display is initialized
    const char* power_texts[4] = {"", "", "", ""};
    char power_tens_buf[12];
    char power_ones_buf[12];
    char power_tenths_buf[12];

    if (bmsConnected || escConnected) {
        float kWatts = unifiedBatteryData.power;

        // Treat tiny magnitudes as zero and clamp negatives to zero for display
        if (kWatts > -0.05f && kWatts < 0.05f) {
          kWatts = 0.0f;
        }
        if (kWatts < 0.0f) {
          kWatts = 0.0f;
        }

        // Carry-safe rounding to one decimal place (e.g., 9.95 -> 10.0)
        int tenths_total = static_cast<int>(kWatts * 10.0f + 0.5f);
        int whole_part = tenths_total / 10;
        int decimal_part = tenths_total % 10;

        // Extract individual digits
        int tens = whole_part / 10;
        int ones = whole_part % 10;

        // Tens digit (only show if >= 10kW)
        if (tens > 0) {
          snprintf(power_tens_buf, sizeof(power_tens_buf), "%d", tens);
          power_texts[0] = power_tens_buf;
        }

        // Ones digit (always show)
        snprintf(power_ones_buf, sizeof(power_ones_buf), "%d", ones);
        power_texts[1] = power_ones_buf;

        // Decimal point
        power_texts[2] = ".";

        // Tenths digit
        snprintf(power_tenths_buf, sizeof(power_tenths_buf), "%d", decimal_part);
        power_texts[3] = power_tenths_buf;

        // Unit label is static and already set to "kW"
    }
    // else: all positions clear when no data; unit label stays visible

    for (int i = 0; i < 4; i++) {
      setLabelText(power_char_labels[i], power_texts[i]);
    }
  }

  // Update performance mode - Re-added this section
  if (perf_mode_label != NULL) {  // Check object exists
      setLabelText(perf_mode_label, deviceData.performance_mode == 0 ? "CHILL " : "SPORT");
  }

  // Update armed time
  if (armed_time_label != NULL) {  // Check object exists
    const unsigned int nowMillis = millis();
    static unsigned int _lastArmedMillis = 0;  // Renamed to avoid conflict
    if (armed) _lastArmedMillis = nowMillis;
    // Calculate session time only if armedStartMillis is valid (not 0)
    const int sessionSeconds = (armedStartMillis > 0) ? ((_lastArmedMillis - armedStartMillis) / 1000) : 0;
    char timeBuffer[10];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);
    setLabelText(armed_time_label, timeBuffer);
  }

  // Update altitude - compute the desired text for all 7 character positions
  // first, then diff-apply so unchanged digits are not redrawn. (The old
  // clear-then-set pattern invalidated all 7 labels twice per frame.)
  const char* altitude_texts[7] = {"", "", "", "", "", "", ""};
  char altitude_digit_buffers[7][12];

  if (altitude == __FLT_MIN__) {
    // Show error in first few positions
    altitude_texts[0] = "E";
    altitude_texts[1] = "R";
    altitude_texts[2] = "R";
  } else {
    if (deviceData.metric_alt) {  // Display meters if metric altitude is enabled
      // Explicitly handle small negative values to avoid "-0.0"
      float display_altitude = altitude;
      if (display_altitude > -0.05f && display_altitude < 0.0f) {
          display_altitude = 0.0f;
      }

      // Handle negative sign and convert to positive for digit extraction
      bool isNegative = (display_altitude < 0);
      float abs_altitude = abs(display_altitude);

      // Convert to integer parts for individual digit extraction
      int whole_part = static_cast<int>(abs_altitude);
      int decimal_part = static_cast<int>(round((abs_altitude - whole_part) * 10));

      // Extract individual digits
      int thousands = whole_part / 1000;
      int hundreds = (whole_part / 100) % 10;
      int tens = (whole_part / 10) % 10;
      int ones = whole_part % 10;

      // Determine the leftmost digit position needed
      int leftmost_pos = 3;  // ones position (always needed)
      if (whole_part >= 10) leftmost_pos = 2;      // tens
      if (whole_part >= 100) leftmost_pos = 1;     // hundreds
      if (whole_part >= 1000) leftmost_pos = 0;    // thousands

      // Place negative sign if needed (one position left of leftmost digit)
      if (isNegative && leftmost_pos > 0) {
        altitude_texts[leftmost_pos - 1] = "-";
      }

      // Thousands digit
      if (thousands > 0) {
        snprintf(altitude_digit_buffers[0], sizeof(altitude_digit_buffers[0]), "%d", thousands);
        altitude_texts[0] = altitude_digit_buffers[0];
      }

      // Hundreds digit
      if (thousands > 0 || hundreds > 0) {
        snprintf(altitude_digit_buffers[1], sizeof(altitude_digit_buffers[1]), "%d", hundreds);
        altitude_texts[1] = altitude_digit_buffers[1];
      }

      // Tens digit
      if (whole_part >= 10) {
        snprintf(altitude_digit_buffers[2], sizeof(altitude_digit_buffers[2]), "%d", tens);
        altitude_texts[2] = altitude_digit_buffers[2];
      }

      // Ones digit (always show)
      snprintf(altitude_digit_buffers[3], sizeof(altitude_digit_buffers[3]), "%d", ones);
      altitude_texts[3] = altitude_digit_buffers[3];

      // Decimal point
      altitude_texts[4] = ".";

      // Tenths digit
      snprintf(altitude_digit_buffers[5], sizeof(altitude_digit_buffers[5]), "%d", decimal_part);
      altitude_texts[5] = altitude_digit_buffers[5];

      // Adjust width of position 4 back to narrow for meters (decimal point).
      // lv_obj_set_size is internally change-detected, so this is free when
      // the unit mode hasn't changed.
      lv_obj_set_size(altitude_char_labels[4], 8, 30);  // decimal_width=8, char_height=30

      // Unit
      altitude_texts[6] = "m";
    } else {
      // For feet - no decimal point needed
      float feet_float = altitude * 3.28084f;
      bool isNegative = (feet_float < 0);
      int feet = static_cast<int>(round(abs(feet_float)));

      // Extract individual digits for up to 99,999 feet
      int ten_thousands = feet / 10000;
      int thousands = (feet / 1000) % 10;
      int hundreds = (feet / 100) % 10;
      int tens = (feet / 10) % 10;
      int ones = feet % 10;

      // Adjust width of position 4 for feet (should be normal width, not narrow)
      lv_obj_set_size(altitude_char_labels[4], 17, 30);  // char_width=19, char_height=30

      // Determine the leftmost digit position needed
      int leftmost_pos = 4;  // ones position (always needed for feet)
      if (feet >= 10) leftmost_pos = 3;        // tens
      if (feet >= 100) leftmost_pos = 2;       // hundreds
      if (feet >= 1000) leftmost_pos = 1;      // thousands
      if (feet >= 10000) leftmost_pos = 0;     // ten-thousands

      // Place negative sign if needed (one position left of leftmost digit)
      if (isNegative && leftmost_pos > 0) {
        altitude_texts[leftmost_pos - 1] = "-";
      }

      // Ten-thousands digit in position 0
      if (ten_thousands > 0) {
        snprintf(altitude_digit_buffers[0], sizeof(altitude_digit_buffers[0]), "%d", ten_thousands);
        altitude_texts[0] = altitude_digit_buffers[0];
      }

      // Thousands digit in position 1
      if (ten_thousands > 0 || thousands > 0) {
        snprintf(altitude_digit_buffers[1], sizeof(altitude_digit_buffers[1]), "%d", thousands);
        altitude_texts[1] = altitude_digit_buffers[1];
      }

      // Hundreds digit in position 2
      if (ten_thousands > 0 || thousands > 0 || hundreds > 0) {
        snprintf(altitude_digit_buffers[2], sizeof(altitude_digit_buffers[2]), "%d", hundreds);
        altitude_texts[2] = altitude_digit_buffers[2];
      }

      // Tens digit in position 3
      if (feet >= 10) {
        snprintf(altitude_digit_buffers[3], sizeof(altitude_digit_buffers[3]), "%d", tens);
        altitude_texts[3] = altitude_digit_buffers[3];
      }

      // Ones digit in position 4 (always show) - now has normal width
      snprintf(altitude_digit_buffers[4], sizeof(altitude_digit_buffers[4]), "%d", ones);
      altitude_texts[4] = altitude_digit_buffers[4];

      // Feet unit in position 6 (same as meters, with proper spacing)
      altitude_texts[6] = "f";
    }
  }

  for (int i = 0; i < 7; i++) {
    setLabelText(altitude_char_labels[i], altitude_texts[i]);
  }

  // Update temperature labels.
  // Tile background styles (normal/warning/critical) are applied only on
  // level transitions: add/remove_style invalidates the tile on every call,
  // which used to redraw active warning tiles every single frame.
  // -- Battery Temperature --
  if (batt_temp_label != NULL && batt_letter_label != NULL) {  // Check labels exist
    uint8_t battTempLevel = 0;  // 0 = normal/no data, 1 = warning, 2 = critical
    if (bmsTelemetry.bmsState == TelemetryState::CONNECTED && hasValidBatteryTemp) {
      char buffer[12];
      snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(batteryTemp));
      setLabelText(batt_temp_label, buffer);
      if (batteryTemp >= bmsCellTempThresholds.critHigh) {
        battTempLevel = 2;
      } else if (batteryTemp >= bmsCellTempThresholds.warnHigh) {
        battTempLevel = 1;
      }
    } else {
      // No valid cell probe connected: show "-" instead of a fake low reading.
      setLabelText(batt_temp_label, "-");
    }

    if (battTempLevel != lastBattTempLevel) {
      lv_obj_remove_style(batt_temp_bg, &style_warning, 0);
      lv_obj_remove_style(batt_temp_bg, &style_critical, 0);
      if (battTempLevel == 2) {
        lv_obj_add_style(batt_temp_bg, &style_critical, 0);
        lv_obj_remove_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else if (battTempLevel == 1) {
        lv_obj_add_style(batt_temp_bg, &style_warning, 0);
        lv_obj_remove_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
      lastBattTempLevel = battTempLevel;
    }
  }

  // -- ESC Temperature --
  if (esc_temp_label != NULL && esc_letter_label != NULL) {  // Check labels exist
    uint8_t escTempLevel = 0;
    if (escTelemetry.escState == TelemetryState::CONNECTED && !isnan(escTemp)) {
      char buffer[12];
      snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(escTemp));
      setLabelText(esc_temp_label, buffer);
      if (escTemp >= escTempThresholds->critHigh) {
        escTempLevel = 2;
      } else if (escTemp >= escTempThresholds->warnHigh) {
        escTempLevel = 1;
      }
    } else {
      setLabelText(esc_temp_label, "-");
    }

    if (escTempLevel != lastEscTempLevel) {
      lv_obj_remove_style(esc_temp_bg, &style_warning, 0);
      lv_obj_remove_style(esc_temp_bg, &style_critical, 0);
      if (escTempLevel == 2) {
        lv_obj_add_style(esc_temp_bg, &style_critical, 0);
        lv_obj_remove_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else if (escTempLevel == 1) {
        lv_obj_add_style(esc_temp_bg, &style_warning, 0);
        lv_obj_remove_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
      lastEscTempLevel = escTempLevel;
    }
  }

  // -- Motor Temperature --
  if (motor_temp_label != NULL && motor_letter_label != NULL) {  // Check labels exist
    uint8_t motorTempLevel = 0;
    // Show motor temp only for a valid numeric reading while ESC is connected.
    if (escTelemetry.escState == TelemetryState::CONNECTED && !isnan(motorTemp)) {
      char buffer[12];
      snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(motorTemp));
      setLabelText(motor_temp_label, buffer);
      if (motorTemp >= motorTempThresholds.critHigh) {
        motorTempLevel = 2;
      } else if (motorTemp >= motorTempThresholds.warnHigh) {
        motorTempLevel = 1;
      }
    } else {
      setLabelText(motor_temp_label, "-");
    }

    if (motorTempLevel != lastMotorTempLevel) {
      lv_obj_remove_style(motor_temp_bg, &style_warning, 0);
      lv_obj_remove_style(motor_temp_bg, &style_critical, 0);
      if (motorTempLevel == 2) {
        lv_obj_add_style(motor_temp_bg, &style_critical, 0);
        lv_obj_remove_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else if (motorTempLevel == 1) {
        lv_obj_add_style(motor_temp_bg, &style_warning, 0);
        lv_obj_remove_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
      lastMotorTempLevel = motorTempLevel;
    }
  }

  // Update armed indicator
  if (armed) {
    // Set background to CYAN when armed, regardless of cruise state
    setBgColor(arm_indicator, LVGL_CYAN, LV_PART_MAIN);
    lv_obj_remove_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);
  }

  // Update Cruise Control Icon Visibility (conditional)
  // Only update based on `cruising` state if not currently flashing
  if (!isFlashingCruiseIcon) {
    if (cruising) {
      lv_obj_remove_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
    }
    // Restore original color after flashing is done
    setImageRecolor(cruise_icon_img, original_cruise_icon_color);
  }

  // Update Charging Icon Visibility - only when BMS is connected and reports charging
  if (charging_icon_img != NULL) {  // Check object exists
    bool showChargingIcon = (bmsTelemetry.bmsState == TelemetryState::CONNECTED) && bmsTelemetry.is_charging;
    if (showChargingIcon) {
      lv_obj_remove_flag(charging_icon_img, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(charging_icon_img, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // Update Arm Fail Warning Icon Visibility (conditional)
  // Only hide if not currently flashing
  if (!isFlashingArmFailIcon && arm_fail_warning_icon_img != NULL) {
    lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
    // Ensure color is reset if flashing ended abruptly elsewhere (though cb should handle it)
    setImageRecolor(arm_fail_warning_icon_img, original_arm_fail_icon_color);
  }

  // Keep the BLE icon synced from the UI task so missed callback updates recover.
  // Pairing mode flash takes priority over the solid-connected state - the
  // flashing Bluetooth symbol communicates "ready to pair" for the entire 60s
  // window, even after the phone has connected. Once the window ends the icon
  // settles solid (if still connected) or hides.
  if (ble_pairing_icon != NULL) {
    if (isBLEPairingModeActive()) {
      if (!isFlashingBLEPairingIcon) {
        isFlashingBLEPairingIcon = true;
        lv_obj_remove_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
        ble_pairing_flash_timer = lv_timer_create(ble_pairing_flash_timer_cb, 500, NULL);
        if (ble_pairing_flash_timer == NULL) {
          isFlashingBLEPairingIcon = false;
          lv_obj_add_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
          USBSerial.println("Error: Failed to create BLE pairing flash timer!");
        }
      }
    } else if (deviceConnected) {
      if (ble_pairing_flash_timer != NULL) {
        lv_timer_del(ble_pairing_flash_timer);
        ble_pairing_flash_timer = NULL;
      }
      lv_obj_remove_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
      isFlashingBLEPairingIcon = false;
    } else {
      if (ble_pairing_flash_timer != NULL) {
        lv_timer_del(ble_pairing_flash_timer);
        ble_pairing_flash_timer = NULL;
      }
      lv_obj_add_flag(ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
      isFlashingBLEPairingIcon = false;
    }
  }

  // Update climb rate indicator
  static float lastAltitude = 0.0f;
  static uint32_t lastAltitudeTime = 0;
  uint32_t currentTime = millis();

  if (currentTime - lastAltitudeTime > 200) {  // Update climb rate every 200ms
    float climbRate = 0.0f;
    if (lastAltitudeTime > 0) {  // Skip first calculation
      float altitudeChange = altitude - lastAltitude;
      float timeChange = (currentTime - lastAltitudeTime) / 1000.0f;  // Convert to seconds
      if (timeChange > 0) {
        climbRate = altitudeChange / timeChange;  // m/s
      }
    }
    updateClimbRateIndicator(climbRate);
    lastAltitude = altitude;
    lastAltitudeTime = currentTime;
  }
}

// Test function with simulated data for development/testing
void updateLvglMainScreenWithTestData(const STR_DEVICE_DATA_140_V1& deviceData) {
  // Create simulated telemetry data
  static uint32_t testStartTime = millis();
  uint32_t elapsed = millis() - testStartTime;

  // Generate dynamic test values that change over time
  float time_factor = (elapsed / 1000.0f);  // Convert to seconds

  // Simulated ESC telemetry
  STR_ESC_TELEMETRY_140 testEscTelemetry = {};
  testEscTelemetry.escState = TelemetryState::CONNECTED;
  testEscTelemetry.cap_temp = 55.0f + sin(time_factor * 0.1f) * 55.0f;  // 0-110°C range
  testEscTelemetry.motor_temp = 55.0f + sin(time_factor * 0.15f) * 55.0f;  // 0-110°C range

  // Simulated BMS telemetry
  STR_BMS_TELEMETRY_140 testBmsTelemetry = {};
  testBmsTelemetry.bmsState = TelemetryState::CONNECTED;
  testBmsTelemetry.lowest_cell_voltage = 3.35f + sin(time_factor * 0.2f) * 0.85f;  // 2.5-4.2V range
  testBmsTelemetry.highest_temperature = 37.5f + sin(time_factor * 0.08f) * 42.5f;  // -5 to 80°C range
  testBmsTelemetry.is_charging = (int(time_factor / 10) % 2 == 0);  // Toggle every 10 seconds

  // Simulated unified battery data
  UnifiedBatteryData testUnifiedBatteryData = {};
  testUnifiedBatteryData.soc = 50.0f + sin(time_factor * 0.05f) * 50.0f;  // 0-100% range
  testUnifiedBatteryData.volts = 80.0f + sin(time_factor * 0.1f) * 20.0f;  // 60-100V range
  testUnifiedBatteryData.power = 10.0f + sin(time_factor * 0.3f) * 10.0f;  // 0-20kW range

  // Simulated altitude with realistic variation
  float testAltitude = 1250.0f + sin(time_factor * 0.02f) * 500.0f + cos(time_factor * 0.1f) * 50.0f;  // Varying altitude

  // Simulated armed/cruise states
  bool testArmed = (int(time_factor / 15) % 2 == 1);  // Toggle every 15 seconds
  bool testCruising = testArmed && (int(time_factor / 8) % 2 == 1);  // Toggle every 8 seconds when armed

  // Simulated armed start time
  static unsigned int testArmedStartMillis = millis();
  if (!testArmed) {
    testArmedStartMillis = millis();  // Reset when not armed
  }

  // Call the main update function with test data
  updateLvglMainScreen(
    deviceData,
    testEscTelemetry,
    testBmsTelemetry,
    testUnifiedBatteryData,
    testAltitude,
    testArmed,
    testCruising,
    testArmedStartMillis);
}
