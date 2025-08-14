#ifndef LVGL_ALERTS_H
#define LVGL_ALERTS_H

#include <lvgl.h>
#include "../simple_monitor.h"
#include "../alert_display.h"
#include "lvgl_main_screen.h"

// Alert counter UI objects
extern lv_obj_t* warning_counter_circle;
extern lv_obj_t* warning_counter_label;
extern lv_obj_t* critical_counter_circle;
extern lv_obj_t* critical_counter_label;

// Alert text carousel objects
extern lv_obj_t* alert_text_label;
extern lv_timer_t* alert_cycle_timer;

// Snapshot state shared between timer and external updater
extern AlertSnapshot currentSnap;
extern uint8_t currentIdx;

// Functions
void setupAlertCounterUI(bool darkMode);
void updateAlertCounterDisplay(const AlertCounts& counts);
void loadAlertSnapshot(const AlertSnapshot& snap);

// Public helpers to control alert text externally
void lv_showAlertText(SensorID id, bool critical);
void lv_showAlertTextWithLevel(SensorID id, AlertLevel level, bool critical);
void lv_hideAlertText();

#endif // LVGL_ALERTS_H
