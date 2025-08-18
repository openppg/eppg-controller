#ifndef LVGL_MAIN_SCREEN_H
#define LVGL_MAIN_SCREEN_H

#include <lvgl.h>
#include "lvgl_core.h"

// Screen page enumeration
enum ScreenPage {
  MAIN_SCREEN
};

// Colors for LVGL - matching ST7735 colors as much as possible
#define LVGL_BLACK lv_color_black()
#define LVGL_WHITE lv_color_white()
#define LVGL_GREEN lv_color_make(0, 255, 0)
#define LVGL_YELLOW lv_color_make(255, 255, 0)
#define LVGL_RED lv_color_make(255, 0, 0)
#define LVGL_BLUE lv_color_make(0, 0, 255)
#define LVGL_ORANGE lv_color_make(255, 165, 0)
#define LVGL_CYAN lv_color_make(0, 255, 255)
#define LVGL_PURPLE lv_color_make(128, 0, 128)
#define LVGL_GRAY lv_color_make(128, 128, 128)


// LVGL styles for temperature warnings and critical states
extern lv_style_t style_warning;
extern lv_style_t style_critical;

// Main screen objects
extern lv_obj_t* main_screen;
extern lv_obj_t* battery_bar;
extern lv_obj_t* battery_label;
extern lv_obj_t* voltage_left_label;
extern lv_obj_t* voltage_right_label;
extern lv_obj_t* power_char_labels[4];  // Individual character position labels for power
extern lv_obj_t* power_unit_label;  // Separate label for "kW" unit text
extern lv_obj_t* power_bar;
extern lv_obj_t* perf_mode_label;
extern lv_obj_t* armed_time_label;
extern lv_obj_t* altitude_char_labels[7];  // Individual character position labels
extern lv_obj_t* batt_temp_label;
extern lv_obj_t* esc_temp_label;
extern lv_obj_t* motor_temp_label;
extern lv_obj_t* arm_indicator;
extern lv_obj_t* spinner;       // For the spinning animation
extern lv_obj_t* spinner_overlay;  // Overlay for the spinner
extern lv_obj_t* batt_letter_label;  // Letter label for Battery temp
extern lv_obj_t* esc_letter_label;  // Letter label for ESC temp
extern lv_obj_t* motor_letter_label;  // Letter label for Motor temp
extern lv_obj_t* batt_temp_bg;  // Background rectangle for Battery temp section
extern lv_obj_t* esc_temp_bg;  // Background rectangle for ESC temp section
extern lv_obj_t* motor_temp_bg;  // Background rectangle for Motor temp section

// Icon objects
extern lv_obj_t* cruise_icon_img;  // Cruise control icon image object
extern lv_obj_t* charging_icon_img;  // Charging icon image object
extern lv_obj_t* arm_fail_warning_icon_img;  // Arm fail warning icon

// Climb rate indicator objects
extern lv_obj_t* climb_rate_divider_lines[13];
extern lv_obj_t* climb_rate_fill_sections[12];

// Critical border (used by flash animations)
extern lv_obj_t* critical_border;

// Loading overlay functions
void showLoadingOverlay();
void hideLoadingOverlay();

// Helper functions
void setAltitudeVisibility(bool visible);
lv_obj_t* createTempBackground(lv_obj_t* parent, int x, int y, int width, int height);

// Main screen setup function
void setupMainScreen(bool darkMode);

#endif // LVGL_MAIN_SCREEN_H
