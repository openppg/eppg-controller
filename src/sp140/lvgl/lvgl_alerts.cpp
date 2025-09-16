#include "../../../inc/sp140/lvgl/lvgl_alerts.h"

// Alert counter UI objects - definitions
lv_obj_t* warning_counter_circle = NULL;
lv_obj_t* warning_counter_label = NULL;
lv_obj_t* critical_counter_circle = NULL;
lv_obj_t* critical_counter_label = NULL;

// Alert text carousel objects - definitions
lv_obj_t* alert_text_label = NULL;
lv_timer_t* alert_cycle_timer = NULL;

// Snapshot state shared between timer and external updater
AlertSnapshot currentSnap{};
uint8_t currentIdx = 0;

void loadAlertSnapshot(const AlertSnapshot& snap) {
  currentSnap = snap;
  currentIdx = 0;
  if (alert_text_label == NULL) return;
  if (snap.count == 0) {
    lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  const char* txt = sensorIDToAbbreviation(static_cast<SensorID>(snap.ids[0]));
  lv_label_set_text(alert_text_label, txt);
  lv_obj_set_style_text_color(alert_text_label, snap.criticalMode ? lv_color_make(255, 0, 0) : lv_color_make(255, 165, 0), 0);
  lv_obj_clear_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
}

// ----------------- Alert Counter UI -----------------
void setupAlertCounterUI(bool darkMode) {
  if (main_screen == NULL) return;

  const int CIRCLE_SIZE = 18;  // smaller circle
  // Warning circle (orange)
  if (warning_counter_circle == NULL) {
    warning_counter_circle = lv_led_create(main_screen);
    lv_obj_set_size(warning_counter_circle, CIRCLE_SIZE, CIRCLE_SIZE);
    // Align relative to altitude area (using first altitude character as reference)
    if (altitude_char_labels[0]) {
      lv_obj_align_to(warning_counter_circle, altitude_char_labels[0], LV_ALIGN_OUT_TOP_LEFT, 1, -2);
    } else {
      lv_obj_align(warning_counter_circle, LV_ALIGN_TOP_LEFT, 6, 75);
    }
    lv_led_set_color(warning_counter_circle, LVGL_ORANGE);
    lv_led_on(warning_counter_circle);
    // Remove bloom/glow effect - make it a solid flat circle
    lv_obj_set_style_bg_grad_dir(warning_counter_circle, LV_GRAD_DIR_NONE, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(warning_counter_circle, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(warning_counter_circle, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_add_flag(warning_counter_circle, LV_OBJ_FLAG_HIDDEN);

    warning_counter_label = lv_label_create(warning_counter_circle);
    lv_obj_set_style_text_font(warning_counter_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(warning_counter_label, LVGL_BLACK, 0);
    lv_obj_align(warning_counter_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(warning_counter_label, "0");
  }

  // Critical circle (red)
  if (critical_counter_circle == NULL) {
    critical_counter_circle = lv_led_create(main_screen);
    lv_obj_set_size(critical_counter_circle, CIRCLE_SIZE, CIRCLE_SIZE);
    if (warning_counter_circle) {
      lv_obj_align_to(critical_counter_circle, warning_counter_circle, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, 0);
    } else {
      lv_obj_align(critical_counter_circle, LV_ALIGN_TOP_LEFT, 35, 75);
    }
    lv_led_set_color(critical_counter_circle, LVGL_RED);
    lv_led_on(critical_counter_circle);
    // Remove bloom/glow effect - make it a solid flat circle
    lv_obj_set_style_bg_grad_dir(critical_counter_circle, LV_GRAD_DIR_NONE, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(critical_counter_circle, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(critical_counter_circle, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_add_flag(critical_counter_circle, LV_OBJ_FLAG_HIDDEN);

    critical_counter_label = lv_label_create(critical_counter_circle);
    lv_obj_set_style_text_font(critical_counter_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(critical_counter_label, LVGL_WHITE, 0);
    lv_obj_align(critical_counter_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(critical_counter_label, "0");
  }

  // Alert text label (to the right of circles)
  if (alert_text_label == NULL) {
    alert_text_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(alert_text_label, LVGL_ORANGE, 0);
    if (critical_counter_circle) {
      lv_obj_align_to(alert_text_label, critical_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    } else {
      lv_obj_align(alert_text_label, LV_ALIGN_TOP_LEFT, 60, 75);
    }
    lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  }

  // Timer removed; rotation is now driven by aggregator task via queue.
}

void updateAlertCounterDisplay(const AlertCounts& counts) {
  if (!warning_counter_circle || !critical_counter_circle) return;

  // Warning
  if (counts.warningCount > 0) {
    char buf[4];
    if (counts.warningCount > 9) {
      snprintf(buf, sizeof(buf), "9+");
    } else {
      snprintf(buf, sizeof(buf), "%u", counts.warningCount);
    }
    lv_label_set_text(warning_counter_label, buf);
    lv_obj_clear_flag(warning_counter_circle, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(warning_counter_circle, LV_OBJ_FLAG_HIDDEN);
  }

    // Critical
  if (counts.criticalCount > 0) {
    char buf[4];
    if (counts.criticalCount > 9) {
      snprintf(buf, sizeof(buf), "9+");
    } else {
      snprintf(buf, sizeof(buf), "%u", counts.criticalCount);
    }
    lv_label_set_text(critical_counter_label, buf);
    lv_obj_clear_flag(critical_counter_circle, LV_OBJ_FLAG_HIDDEN);

    // Simple positioning: align with alert text if no warnings, otherwise right of warning circle
    if (counts.warningCount == 0) {
      // No warnings: position to align with alert text (avoid red border)
      lv_obj_align(critical_counter_circle, LV_ALIGN_TOP_LEFT, 10, 75);
    } else {
      // With warnings: position to the right of the warning circle, align bottoms
      if (warning_counter_circle) {
        lv_obj_align_to(critical_counter_circle, warning_counter_circle, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, 0);
      }
    }
  } else {
    lv_obj_add_flag(critical_counter_circle, LV_OBJ_FLAG_HIDDEN);
  }
}

// Public helpers to control alert text externally
void lv_showAlertText(SensorID id, bool critical) {
  if (alert_text_label == NULL) return;
  const char* txt = sensorIDToAbbreviation(id);
  lv_label_set_text(alert_text_label, txt);
}

// New function that accepts alert level for dynamic abbreviations
void lv_showAlertTextWithLevel(SensorID id, AlertLevel level, bool critical) {
  if (alert_text_label == NULL) return;
  const char* txt = sensorIDToAbbreviationWithLevel(id, level);
  lv_label_set_text(alert_text_label, txt);
  // Use larger font for critical alerts, smaller for warnings
  if (critical) {
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_18, 0);
    // Hide altitude while showing critical alert
    setAltitudeVisibility(false);
    // Place alert label near altitude readout area
    if (altitude_char_labels[0]) {
      lv_obj_set_pos(alert_text_label, lv_obj_get_x(altitude_char_labels[0]) + 5, lv_obj_get_y(altitude_char_labels[0]));
    }
  } else {
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_14, 0);
    // Ensure altitude visible during warnings
    setAltitudeVisibility(true);
    // Re-align warning text next to circles
    if (warning_counter_circle) {
      lv_obj_align_to(alert_text_label, warning_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    }
    // Make warning text darker and slightly larger for readability
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_16, 0);
    // Dark orange for better readability over light background
    lv_obj_set_style_text_color(alert_text_label, lv_color_make(200, 100, 0), 0);
  }
  if (critical) {
    lv_obj_set_style_text_color(alert_text_label, lv_color_make(255, 0, 0), 0);
  }
  lv_obj_clear_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
}

void lv_hideAlertText() {
  if (alert_text_label == NULL) return;
  lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  // Restore altitude visibility when no critical alert
  setAltitudeVisibility(true);
}
