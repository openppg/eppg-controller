#include "../../../inc/sp140/lvgl/lvgl_updates.h"
#include "../../../inc/sp140/esp32s3-config.h"
#include "../../../inc/sp140/monitor_config.h"  // For direct threshold access
#include "../../../inc/sp140/globals.h"
#include "../../../inc/sp140/vibration_pwm.h"
#include "../../../inc/sp140/shared-config.h"

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

// Timer callback declarations
static void cruise_flash_timer_cb(lv_timer_t* timer);
static void arm_fail_flash_timer_cb(lv_timer_t* timer);
static void critical_border_flash_timer_cb(lv_timer_t* timer);

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
    lv_obj_clear_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
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
    lv_obj_clear_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);

    // Create the timer (250ms interval for on/off cycle)
    cruise_flash_timer = lv_timer_create(cruise_flash_timer_cb, 250, NULL);
    if (cruise_flash_timer == NULL) {
      // Failed to create timer, reset state
      isFlashingCruiseIcon = false;
      lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide it again
      lv_obj_set_style_img_recolor(cruise_icon_img, original_cruise_icon_color, LV_PART_MAIN);  // Restore color on failure
      USBSerial.println("Error: Failed to create cruise flash timer!");
    } else {
      // Set icon to red for flashing
      lv_obj_set_style_img_recolor(cruise_icon_img, LVGL_RED, LV_PART_MAIN);
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
    lv_obj_clear_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
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
    lv_obj_set_style_img_recolor(arm_fail_warning_icon_img, original_arm_fail_icon_color, LV_PART_MAIN);
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
    lv_obj_clear_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);

    // Set icon to red for flashing
    lv_obj_set_style_img_recolor(arm_fail_warning_icon_img, LVGL_RED, LV_PART_MAIN);

    // Create the timer (250ms interval for on/off cycle)
    arm_fail_flash_timer = lv_timer_create(arm_fail_flash_timer_cb, 250, NULL);
    if (arm_fail_flash_timer == NULL) {
      // Failed to create timer, reset state
      isFlashingArmFailIcon = false;
      lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide it again
      lv_obj_set_style_img_recolor(arm_fail_warning_icon_img, original_arm_fail_icon_color, LV_PART_MAIN);  // Restore color
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
      lv_obj_invalidate(lv_scr_act());
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
    lv_refr_now(lv_disp_get_default());
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
      lv_obj_invalidate(lv_scr_act());
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
    lv_refr_now(lv_disp_get_default());
  }
}

void stopCriticalBorderFlashDirect() {
  if (critical_border_flash_timer != NULL) {
    lv_timer_del(critical_border_flash_timer);
    critical_border_flash_timer = NULL;
  }
  if (critical_border != NULL) {
    lv_obj_set_style_border_opa(critical_border, LV_OPA_0, LV_PART_MAIN);
    lv_obj_invalidate(lv_scr_act());  // Ensure clean removal of border
    // Force immediate refresh for clean stop
    lv_refr_now(lv_disp_get_default());
  }
  isFlashingCriticalBorder = false;
}

// Update the climb rate indicator
void updateClimbRateIndicator(float climbRate) {
  // Clamp climb rate to displayable range (-0.6 to +0.6 m/s)
  if (climbRate > 0.6f) climbRate = 0.6f;
  if (climbRate < -0.6f) climbRate = -0.6f;

  // Reset all sections to transparent
  for (int i = 0; i < 12; i++) {
    if (climb_rate_fill_sections[i] != NULL) {
      lv_obj_set_style_bg_opa(climb_rate_fill_sections[i], LV_OPA_0, LV_PART_MAIN);
    }
  }

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

  if (climbRate >= 0.15f) {
    // Positive climb rate - fill sections above center line
    int sectionsToFill = (int)(climbRate / 0.3f);
    if (sectionsToFill > 6) sectionsToFill = 6;

    for (int i = 0; i < sectionsToFill; i++) {
      int sectionIndex = 5 - i;  // Section 5 is closest to center line, going up to 0
      if (climb_rate_fill_sections[sectionIndex] != NULL) {
        lv_obj_set_style_bg_opa(climb_rate_fill_sections[sectionIndex], LV_OPA_100, LV_PART_MAIN);
        lv_obj_set_style_bg_color(climb_rate_fill_sections[sectionIndex], positive_colors[i], LV_PART_MAIN);
      }
    }
  } else if (climbRate <= -0.15f) {
    // Negative climb rate - fill sections below center line
    int sectionsToFill = (int)(-climbRate / 0.3f);
    if (sectionsToFill > 6) sectionsToFill = 6;

    for (int i = 0; i < sectionsToFill; i++) {
      int sectionIndex = 6 + i;  // Section 6 is closest to center line, going down to 11
      if (climb_rate_fill_sections[sectionIndex] != NULL) {
        lv_obj_set_style_bg_opa(climb_rate_fill_sections[sectionIndex], LV_OPA_100, LV_PART_MAIN);
        lv_obj_set_style_bg_color(climb_rate_fill_sections[sectionIndex], negative_colors[i], LV_PART_MAIN);
      }
    }
  }
  // If climb rate is between -0.15 and +0.15, no sections are filled (neutral)
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
  // Calculate highest cell temperature from T1-T4 only (excluding MOSFET and balance temps)
  float batteryTemp = bmsTelemetry.t1_temperature;
  if (bmsTelemetry.t2_temperature > batteryTemp) batteryTemp = bmsTelemetry.t2_temperature;
  if (bmsTelemetry.t3_temperature > batteryTemp) batteryTemp = bmsTelemetry.t3_temperature;
  if (bmsTelemetry.t4_temperature > batteryTemp) batteryTemp = bmsTelemetry.t4_temperature;
  float escTemp = escTelemetry.cap_temp;
  float motorTemp = escTelemetry.motor_temp;
  // Check if BMS or ESC is connected
  bool bmsConnected = (bmsTelemetry.bmsState == TelemetryState::CONNECTED);
  bool escConnected = (escTelemetry.escState == TelemetryState::CONNECTED);

  // Check if spinner exists before styling it
  if (spinner != NULL) {
    lv_obj_set_style_arc_color(spinner, darkMode ? lv_color_make(100, 100, 100) : lv_color_make(230, 230, 230), LV_PART_MAIN);
    lv_obj_set_style_arc_color(spinner, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
  }

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

    lv_obj_set_style_bg_color(battery_bar, batteryColor, LV_PART_INDICATOR);

    // Update battery percentage text
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%d%%", (int)batteryPercent);
    lv_label_set_text(battery_label, buffer);
    lv_obj_set_style_text_color(battery_label, LVGL_BLACK, 0);
  } else if (escConnected) {
    // clear the battery bar, we handle voltage later
    lv_bar_set_value(battery_bar, 0, LV_ANIM_OFF);
  } else {
    lv_bar_set_value(battery_bar, 0, LV_ANIM_OFF);
    lv_label_set_text(battery_label, "NO DATA");
    lv_obj_set_style_text_color(battery_label, LVGL_RED, 0);
  }

  // Update left voltage (cell voltage)
  if (voltage_left_label != NULL) {  // Check object exists
    if (bmsConnected) {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.2fv", lowestCellV);
        lv_label_set_text(voltage_left_label, buffer);

        // Always use black text for better readability
        lv_obj_set_style_text_color(voltage_left_label, LVGL_BLACK, 0);
    } else if (escConnected) {
        lv_obj_set_style_text_color(voltage_left_label, LVGL_BLACK, 0);
        lv_label_set_text(voltage_left_label, "NO\nBMS");
    } else {
        lv_label_set_text(voltage_left_label, "");
    }
  }

  // Update right voltage (total voltage)
  // if esc connected, show esc voltage in center of screen
  if (voltage_right_label != NULL && battery_label != NULL) {  // Check objects exist
    if (bmsConnected) {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.0fv", totalVolts);
        lv_label_set_text(voltage_right_label, buffer);
        // Ensure battery label shows percentage if BMS is connected
        if (batteryPercent >= 0) {
           char batt_buffer[10];
           snprintf(batt_buffer, sizeof(batt_buffer), "%d%%", (int)batteryPercent);
           lv_label_set_text(battery_label, batt_buffer);
           lv_obj_set_style_text_color(battery_label, LVGL_BLACK, 0);
        }

    } else if (escConnected) {
        lv_label_set_text(voltage_right_label, "");

        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.1fv", totalVolts);
        lv_label_set_text(battery_label, buffer);
        lv_obj_set_style_text_color(battery_label, LVGL_BLACK, 0);
    } else {
        lv_label_set_text(voltage_right_label, "");
    }
  }

  // Update power display with individual character positions
  if (power_char_labels[0] != NULL) {  // Check if power display is initialized
    if (bmsConnected || escConnected) {
        float kWatts = unifiedBatteryData.power;

        // Treat tiny magnitudes as zero and clamp negatives to zero for display
        if (kWatts > -0.05f && kWatts < 0.05f) {
          kWatts = 0.0f;
        }
        if (kWatts < 0.0f) {
          kWatts = 0.0f;
        }

        // Clear all positions first
        for (int i = 0; i < 4; i++) {
          lv_label_set_text(power_char_labels[i], "");
        }

        // Carry-safe rounding to one decimal place (e.g., 9.95 -> 10.0)
        int tenths_total = static_cast<int>(kWatts * 10.0f + 0.5f);
        int whole_part = tenths_total / 10;
        int decimal_part = tenths_total % 10;

        // Extract individual digits
        int tens = whole_part / 10;
        int ones = whole_part % 10;

        // Populate character positions using static buffers
        static char power_digit_buffers[4][2];

        // Tens digit (only show if >= 10kW)
        if (tens > 0) {
          snprintf(power_digit_buffers[0], sizeof(power_digit_buffers[0]), "%d", tens);
          lv_label_set_text(power_char_labels[0], power_digit_buffers[0]);
        }

        // Ones digit (always show)
        snprintf(power_digit_buffers[1], sizeof(power_digit_buffers[1]), "%d", ones);
        lv_label_set_text(power_char_labels[1], power_digit_buffers[1]);

        // Decimal point
        lv_label_set_text(power_char_labels[2], ".");

        // Tenths digit
        snprintf(power_digit_buffers[3], sizeof(power_digit_buffers[3]), "%d", decimal_part);
        lv_label_set_text(power_char_labels[3], power_digit_buffers[3]);

        // Unit label is static and already set to "kW"
    } else {
        // Clear all positions when no data
        for (int i = 0; i < 4; i++) {
          lv_label_set_text(power_char_labels[i], "");
        }
        // Keep unit label visible even when no data
    }
  }

  // Update performance mode - Re-added this section
  if (perf_mode_label != NULL) {  // Check object exists
      lv_label_set_text(perf_mode_label, deviceData.performance_mode == 0 ? "CHILL " : "SPORT");
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
    lv_label_set_text(armed_time_label, timeBuffer);
  }

  // Update altitude - populate individual character positions
  if (altitude == __FLT_MIN__) {
    // Show error in first few positions
    lv_label_set_text(altitude_char_labels[0], "E");
    lv_label_set_text(altitude_char_labels[1], "R");
    lv_label_set_text(altitude_char_labels[2], "R");
    lv_label_set_text(altitude_char_labels[3], "");
    lv_label_set_text(altitude_char_labels[4], "");
    lv_label_set_text(altitude_char_labels[5], "");
    lv_label_set_text(altitude_char_labels[6], "");
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

      // Populate each character position using static character buffers
      static char digit_buffers[7][2];  // Static buffers for single digits

      // Clear all positions first
      for (int i = 0; i < 7; i++) {
        lv_label_set_text(altitude_char_labels[i], "");
      }

      // Determine the leftmost digit position needed
      int leftmost_pos = 3;  // ones position (always needed)
      if (whole_part >= 10) leftmost_pos = 2;      // tens
      if (whole_part >= 100) leftmost_pos = 1;     // hundreds
      if (whole_part >= 1000) leftmost_pos = 0;    // thousands

      // Place negative sign if needed (one position left of leftmost digit)
      if (isNegative && leftmost_pos > 0) {
        lv_label_set_text(altitude_char_labels[leftmost_pos - 1], "-");
      }

      // Thousands digit
      if (thousands > 0) {
        snprintf(digit_buffers[0], sizeof(digit_buffers[0]), "%d", thousands);
        lv_label_set_text(altitude_char_labels[0], digit_buffers[0]);
      }

      // Hundreds digit
      if (thousands > 0 || hundreds > 0) {
        snprintf(digit_buffers[1], sizeof(digit_buffers[1]), "%d", hundreds);
        lv_label_set_text(altitude_char_labels[1], digit_buffers[1]);
      }

      // Tens digit
      if (whole_part >= 10) {
        snprintf(digit_buffers[2], sizeof(digit_buffers[2]), "%d", tens);
        lv_label_set_text(altitude_char_labels[2], digit_buffers[2]);
      }

      // Ones digit (always show)
      snprintf(digit_buffers[3], sizeof(digit_buffers[3]), "%d", ones);
      lv_label_set_text(altitude_char_labels[3], digit_buffers[3]);

      // Decimal point
      lv_label_set_text(altitude_char_labels[4], ".");

      // Tenths digit
      snprintf(digit_buffers[5], sizeof(digit_buffers[5]), "%d", decimal_part);
      lv_label_set_text(altitude_char_labels[5], digit_buffers[5]);

      // Adjust width of position 4 back to narrow for meters (decimal point)
      lv_obj_set_size(altitude_char_labels[4], 8, 24);  // decimal_width=8, char_height=24

      // Unit
      lv_label_set_text(altitude_char_labels[6], "m");
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

      static char digit_buffers_ft[7][2];  // Static buffers for feet

      // Adjust width of position 4 for feet (should be normal width, not narrow)
      lv_obj_set_size(altitude_char_labels[4], 17, 24);  // char_width=19, char_height=24

      // Clear all positions first
      for (int i = 0; i < 7; i++) {
        lv_label_set_text(altitude_char_labels[i], "");
      }

      // Determine the leftmost digit position needed
      int leftmost_pos = 4;  // ones position (always needed for feet)
      if (feet >= 10) leftmost_pos = 3;        // tens
      if (feet >= 100) leftmost_pos = 2;       // hundreds
      if (feet >= 1000) leftmost_pos = 1;      // thousands
      if (feet >= 10000) leftmost_pos = 0;     // ten-thousands

      // Place negative sign if needed (one position left of leftmost digit)
      if (isNegative && leftmost_pos > 0) {
        lv_label_set_text(altitude_char_labels[leftmost_pos - 1], "-");
      }

      // Ten-thousands digit in position 0
      if (ten_thousands > 0) {
        snprintf(digit_buffers_ft[0], sizeof(digit_buffers_ft[0]), "%d", ten_thousands);
        lv_label_set_text(altitude_char_labels[0], digit_buffers_ft[0]);
      }

      // Thousands digit in position 1
      if (ten_thousands > 0 || thousands > 0) {
        snprintf(digit_buffers_ft[1], sizeof(digit_buffers_ft[1]), "%d", thousands);
        lv_label_set_text(altitude_char_labels[1], digit_buffers_ft[1]);
      }

      // Hundreds digit in position 2
      if (ten_thousands > 0 || thousands > 0 || hundreds > 0) {
        snprintf(digit_buffers_ft[2], sizeof(digit_buffers_ft[2]), "%d", hundreds);
        lv_label_set_text(altitude_char_labels[2], digit_buffers_ft[2]);
      }

      // Tens digit in position 3
      if (feet >= 10) {
        snprintf(digit_buffers_ft[3], sizeof(digit_buffers_ft[3]), "%d", tens);
        lv_label_set_text(altitude_char_labels[3], digit_buffers_ft[3]);
      }

      // Ones digit in position 4 (always show) - now has normal width
      snprintf(digit_buffers_ft[4], sizeof(digit_buffers_ft[4]), "%d", ones);
      lv_label_set_text(altitude_char_labels[4], digit_buffers_ft[4]);

      // Feet unit in position 6 (same as meters, with proper spacing)
      lv_label_set_text(altitude_char_labels[6], "f");
    }
  }

  // Update temperature labels
  // -- Battery Temperature --
  if (batt_temp_label != NULL && batt_letter_label != NULL) {  // Check labels exist
    lv_obj_remove_style(batt_temp_bg, &style_warning, 0);
    lv_obj_remove_style(batt_temp_bg, &style_critical, 0);

    if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
      lv_label_set_text_fmt(batt_temp_label, "%d", static_cast<int>(batteryTemp));
      if (batteryTemp >= bmsCellTempThresholds.critHigh) {
        lv_obj_add_style(batt_temp_bg, &style_critical, 0);
        lv_obj_clear_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else if (batteryTemp >= bmsCellTempThresholds.warnHigh) {
        lv_obj_add_style(batt_temp_bg, &style_warning, 0);
        lv_obj_clear_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
    } else {
      lv_label_set_text(batt_temp_label, "-");
      lv_obj_add_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // -- ESC Temperature --
  if (esc_temp_label != NULL && esc_letter_label != NULL) {  // Check labels exist
    lv_obj_remove_style(esc_temp_bg, &style_warning, 0);
    lv_obj_remove_style(esc_temp_bg, &style_critical, 0);

    if (escTelemetry.escState == TelemetryState::CONNECTED) {
      lv_label_set_text_fmt(esc_temp_label, "%d", static_cast<int>(escTemp));

      if (escTemp >= escMosTempThresholds.critHigh) {
        lv_obj_add_style(esc_temp_bg, &style_critical, 0);
        lv_obj_clear_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else if (escTemp >= escMosTempThresholds.warnHigh) {
        lv_obj_add_style(esc_temp_bg, &style_warning, 0);
        lv_obj_clear_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
    } else {
      lv_label_set_text(esc_temp_label, "-");
      lv_obj_add_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // -- Motor Temperature --
  if (motor_temp_label != NULL && motor_letter_label != NULL) {  // Check labels exist
    lv_obj_remove_style(motor_temp_bg, &style_warning, 0);
    lv_obj_remove_style(motor_temp_bg, &style_critical, 0);

    if (escTelemetry.escState == TelemetryState::CONNECTED && motorTemp > -20.0f) {
      lv_label_set_text_fmt(motor_temp_label, "%d", static_cast<int>(motorTemp));

      if (motorTemp >= motorTempThresholds.critHigh) {
        lv_obj_add_style(motor_temp_bg, &style_critical, 0);
        lv_obj_clear_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else if (motorTemp >= motorTempThresholds.warnHigh) {
        lv_obj_add_style(motor_temp_bg, &style_warning, 0);
        lv_obj_clear_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
    } else {
      lv_label_set_text(motor_temp_label, "-");
      lv_obj_add_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // Update armed indicator
  if (armed) {
    // Set background to CYAN when armed, regardless of cruise state
    lv_obj_set_style_bg_color(arm_indicator, LVGL_CYAN, LV_PART_MAIN);
    lv_obj_clear_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);
  }

  // Update Cruise Control Icon Visibility (conditional)
  // Only update based on `cruising` state if not currently flashing
  if (!isFlashingCruiseIcon) {
    if (cruising) {
      lv_obj_clear_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
    }
    // Restore original color after flashing is done
    lv_obj_set_style_img_recolor(cruise_icon_img, original_cruise_icon_color, LV_PART_MAIN);
  }

  // Update Charging Icon Visibility - only when BMS is connected and reports charging
  if (charging_icon_img != NULL) {  // Check object exists
    bool showChargingIcon = (bmsTelemetry.bmsState == TelemetryState::CONNECTED) && bmsTelemetry.is_charging;
    if (showChargingIcon) {
      lv_obj_clear_flag(charging_icon_img, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(charging_icon_img, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // Update Arm Fail Warning Icon Visibility (conditional)
  // Only hide if not currently flashing
  if (!isFlashingArmFailIcon && arm_fail_warning_icon_img != NULL) {
    lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
    // Ensure color is reset if flashing ended abruptly elsewhere (though cb should handle it)
    lv_obj_set_style_img_recolor(arm_fail_warning_icon_img, original_arm_fail_icon_color, LV_PART_MAIN);
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
