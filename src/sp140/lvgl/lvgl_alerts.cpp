#include "../../../inc/sp140/lvgl/lvgl_alerts.h"

// Alert counter UI objects - definitions
lv_obj_t* warning_counter_circle = NULL;
lv_obj_t* warning_counter_label = NULL;
lv_obj_t* critical_counter_circle = NULL;
lv_obj_t* critical_counter_label = NULL;

// Alert text carousel objects - definitions
lv_obj_t* alert_text_label = NULL;       // For warning text
lv_obj_t* critical_text_label = NULL;    // For critical text (separate so both can show)
lv_timer_t* alert_cycle_timer = NULL;

// Snapshot state shared between timer and external updater
AlertSnapshot currentSnap{};
uint8_t currentIdx = 0;

void loadAlertSnapshot(const AlertSnapshot& snap) {
  currentSnap = snap;
  currentIdx = 0;
  if (snap.count == 0) {
    if (alert_text_label != NULL) {
      lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (critical_text_label != NULL) {
      lv_obj_add_flag(critical_text_label, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }
  const char* txt = sensorIDToAbbreviation(static_cast<SensorID>(snap.ids[0]));
  if (snap.criticalMode) {
    if (critical_text_label == NULL) return;
    lv_label_set_text(critical_text_label, txt);
    lv_obj_set_style_text_color(critical_text_label, lv_color_make(255, 0, 0), 0);
    lv_obj_clear_flag(critical_text_label, LV_OBJ_FLAG_HIDDEN);
  } else {
    if (alert_text_label == NULL) return;
    lv_label_set_text(alert_text_label, txt);
    lv_obj_set_style_text_color(alert_text_label, lv_color_make(255, 165, 0), 0);
    lv_obj_clear_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  }
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

  // Warning text label (to the right of warning circle)
  if (alert_text_label == NULL) {
    alert_text_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(alert_text_label, LVGL_ORANGE, 0);
    if (warning_counter_circle) {
      lv_obj_align_to(alert_text_label, warning_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    } else {
      lv_obj_align(alert_text_label, LV_ALIGN_TOP_LEFT, 60, 75);
    }
    lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  }

  // Critical text label (in altitude area, to the right of critical circle)
  if (critical_text_label == NULL) {
    critical_text_label = lv_label_create(main_screen);
    lv_obj_set_style_text_font(critical_text_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(critical_text_label, lv_color_make(255, 0, 0), 0);
    // Initially positioned in altitude area
    if (altitude_char_labels[0]) {
      lv_obj_set_pos(critical_text_label, lv_obj_get_x(altitude_char_labels[0]) + 25, lv_obj_get_y(altitude_char_labels[0]));
    } else {
      lv_obj_align(critical_text_label, LV_ALIGN_TOP_LEFT, 30, 100);
    }
    lv_obj_add_flag(critical_text_label, LV_OBJ_FLAG_HIDDEN);
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
  const char* txt = sensorIDToAbbreviationWithLevel(id, level);

  if (critical) {
    // Use critical_text_label for critical alerts - shows on TOP row (above altitude)
    if (critical_text_label == NULL) return;
    lv_label_set_text(critical_text_label, txt);
    lv_obj_set_style_text_font(critical_text_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(critical_text_label, lv_color_make(255, 0, 0), 0);
    // Keep altitude visible when only critical is showing
    // (altitude will be hidden by warning if both are showing)
    setAltitudeVisibility(true);
    // Position critical circle and text in the top row (above altitude)
    if (critical_counter_circle && altitude_char_labels[0]) {
      // Position in the top row area (above altitude)
      lv_obj_align_to(critical_counter_circle, altitude_char_labels[0], LV_ALIGN_OUT_TOP_LEFT, 1, -2);
      // Place critical text to the right of the red bubble
      lv_obj_align_to(critical_text_label, critical_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    }
    lv_obj_clear_flag(critical_text_label, LV_OBJ_FLAG_HIDDEN);
  } else {
    // Use alert_text_label for warning alerts
    if (alert_text_label == NULL) return;
    lv_label_set_text(alert_text_label, txt);
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_18, 0);
    // Dark orange for better readability
    lv_obj_set_style_text_color(alert_text_label, lv_color_make(200, 100, 0), 0);

    // Check if critical is also showing
    bool criticalShowing = (critical_text_label != NULL &&
                            !lv_obj_has_flag(critical_text_label, LV_OBJ_FLAG_HIDDEN));

    if (criticalShowing) {
      // Critical is showing on top, so warning goes to altitude area (bottom)
      // Hide altitude since warning takes that space
      setAltitudeVisibility(false);
      if (warning_counter_circle && critical_counter_circle && altitude_char_labels[0]) {
        lv_coord_t altY = lv_obj_get_y(altitude_char_labels[0]);
        // Align warning bubble X with critical bubble X so texts line up
        lv_coord_t critX = lv_obj_get_x(critical_counter_circle);
        // Place orange bubble aligned with critical bubble
        lv_obj_set_pos(warning_counter_circle, critX, altY + 2);
        // Place warning text to the right of the orange bubble
        lv_obj_align_to(alert_text_label, warning_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
      }
    } else {
      // No critical showing, so warning goes to top row (altitude stays visible)
      setAltitudeVisibility(true);
      if (warning_counter_circle && altitude_char_labels[0]) {
        // Position in the top row area (above altitude)
        lv_obj_align_to(warning_counter_circle, altitude_char_labels[0], LV_ALIGN_OUT_TOP_LEFT, 1, -2);
        // Place warning text to the right of the orange bubble
        lv_obj_align_to(alert_text_label, warning_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
      }
    }
    lv_obj_clear_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  }
}

void lv_hideAlertText() {
  lv_hideWarningText();
  lv_hideCriticalText();
}

void lv_hideWarningText() {
  if (alert_text_label != NULL) {
    lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  }
  // Restore warning circle to its original position (above altitude)
  if (warning_counter_circle && altitude_char_labels[0]) {
    lv_obj_align_to(warning_counter_circle, altitude_char_labels[0], LV_ALIGN_OUT_TOP_LEFT, 1, -2);
  }
  // Warning doesn't control altitude visibility anymore - only critical does
}

void lv_hideCriticalText() {
  if (critical_text_label != NULL) {
    lv_obj_add_flag(critical_text_label, LV_OBJ_FLAG_HIDDEN);
  }
  // Restore critical circle to its normal position (next to warning circle)
  if (critical_counter_circle && warning_counter_circle) {
    lv_obj_align_to(critical_counter_circle, warning_counter_circle, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, 0);
  }
  // Restore altitude visibility when critical is hidden
  setAltitudeVisibility(true);
}
