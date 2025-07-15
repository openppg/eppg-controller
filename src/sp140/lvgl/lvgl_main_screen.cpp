#include "../../../inc/sp140/lvgl/lvgl_main_screen.h"
#include "../../../inc/sp140/esp32s3-config.h"

#include "../../assets/img/cruise-control-340255-30.c"  // Cruise control icon  // NOLINT(build/include)
#include "../../assets/img/energy-539741-26.c"  // Charging icon  // NOLINT(build/include)
#include "../../assets/img/warning_2135850_30.c"  // Warning icon  // NOLINT(build/include)
#include "../../assets/fonts/roboto_mono_16.c"  // Monospace font for numbers  // NOLINT(build/include)

// External font declaration
extern const lv_font_t roboto_mono_16;

// Main screen objects - definitions
lv_obj_t* main_screen = NULL;
lv_obj_t* battery_bar = NULL;
lv_obj_t* battery_label = NULL;
lv_obj_t* voltage_left_label = NULL;
lv_obj_t* voltage_right_label = NULL;
lv_obj_t* power_char_labels[4] = {NULL};  // Individual character position labels for power: [tens][ones][.][tenths]
lv_obj_t* power_unit_label = NULL;  // Separate label for "kW" unit text
lv_obj_t* power_bar = NULL;
lv_obj_t* perf_mode_label = NULL;
lv_obj_t* armed_time_label = NULL;
lv_obj_t* altitude_char_labels[7] = {NULL};  // Individual character position labels
lv_obj_t* batt_temp_label = NULL;
lv_obj_t* esc_temp_label = NULL;
lv_obj_t* motor_temp_label = NULL;
lv_obj_t* arm_indicator = NULL;
lv_obj_t* spinner = NULL;       // For the spinning animation
lv_obj_t* spinner_overlay = NULL;  // Overlay for the spinner
lv_obj_t* batt_letter_label = NULL;  // Letter label for Battery temp
lv_obj_t* esc_letter_label = NULL;  // Letter label for ESC temp
lv_obj_t* motor_letter_label = NULL;  // Letter label for Motor temp
lv_obj_t* batt_temp_bg = NULL;  // Background rectangle for Battery temp section
lv_obj_t* esc_temp_bg = NULL;  // Background rectangle for ESC temp section
lv_obj_t* motor_temp_bg = NULL;  // Background rectangle for Motor temp section

// Icon objects
lv_obj_t* cruise_icon_img = NULL;  // Cruise control icon image object
lv_obj_t* charging_icon_img = NULL;  // Charging icon image object
lv_obj_t* arm_fail_warning_icon_img = NULL;  // Arm fail warning icon

// Climb rate indicator horizontal divider lines (13 lines total)
lv_obj_t* climb_rate_divider_lines[13] = {NULL};

// Climb rate fill sections (12 sections between the 13 lines)
lv_obj_t* climb_rate_fill_sections[12] = {NULL};

// Critical border (used by flash animations)
lv_obj_t* critical_border = NULL;

// Helper function to hide/show all altitude character labels
void setAltitudeVisibility(bool visible) {
  for (int i = 0; i < 7; i++) {
    if (altitude_char_labels[i]) {
      if (visible) {
        lv_obj_clear_flag(altitude_char_labels[i], LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(altitude_char_labels[i], LV_OBJ_FLAG_HIDDEN);
      }
    }
  }
}

// Helper function to create temperature background rectangles
lv_obj_t* createTempBackground(lv_obj_t* parent, int x, int y, int width, int height) {
  lv_obj_t* bg = lv_obj_create(parent);
  lv_obj_set_size(bg, width, height);
  lv_obj_set_pos(bg, x, y);
  lv_obj_set_style_border_width(bg, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(bg, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bg, LV_OPA_0, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(bg, 0, LV_PART_MAIN);
  lv_obj_set_style_outline_width(bg, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bg, 0, LV_PART_MAIN);
  lv_obj_set_style_border_opa(bg, LV_OPA_0, LV_PART_MAIN);
  lv_obj_set_style_outline_opa(bg, LV_OPA_0, LV_PART_MAIN);
  lv_obj_set_style_shadow_opa(bg, LV_OPA_0, LV_PART_MAIN);
  lv_obj_move_background(bg);
  lv_obj_add_flag(bg, LV_OBJ_FLAG_HIDDEN);
  return bg;
}

// Function to show the loading overlay
void showLoadingOverlay() {
  if (spinner_overlay != NULL) {
    lv_obj_clear_flag(spinner_overlay, LV_OBJ_FLAG_HIDDEN);
  }
}

// Function to hide the loading overlay
void hideLoadingOverlay() {
  if (spinner_overlay != NULL) {
    lv_obj_add_flag(spinner_overlay, LV_OBJ_FLAG_HIDDEN);
  }
}

// Setup the main screen layout once
void setupMainScreen(bool darkMode) {
  if (main_screen != NULL) {
    lv_obj_del(main_screen);
  }

  // Create main screen
  main_screen = lv_obj_create(NULL);

  // Disable scrollbars
  lv_obj_clear_flag(main_screen, LV_OBJ_FLAG_SCROLLABLE);

  // Set theme based on dark/light mode
  lv_obj_set_style_bg_color(main_screen,
                          darkMode ? LVGL_BLACK : LVGL_WHITE,
                          LV_PART_MAIN);

  // Top section - battery status
  battery_bar = lv_bar_create(main_screen);
  lv_obj_set_size(battery_bar, SCREEN_WIDTH, 37);
  lv_obj_set_pos(battery_bar, 0, 0);
  lv_bar_set_range(battery_bar, 0, 100);
  lv_obj_set_style_bg_color(battery_bar,
                          darkMode ? LVGL_BLACK : LVGL_WHITE,
                          LV_PART_MAIN);
  // Remove rounded corners from battery bar
  lv_obj_set_style_radius(battery_bar, 0, LV_PART_MAIN);  // Background part
  lv_obj_set_style_radius(battery_bar, 0, LV_PART_INDICATOR);  // Indicator part

  // Battery percentage label
  battery_label = lv_label_create(main_screen);
  lv_obj_align(battery_label, LV_ALIGN_TOP_MID, 0, 3);  // Move up for better vertical centering in battery bar
  lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_28, 0);  // Large font for prominent percentage display
  lv_obj_set_style_text_color(battery_label, LVGL_BLACK, 0);
  // Center-align battery percentage since it's in the middle
  lv_obj_set_style_text_align(battery_label, LV_TEXT_ALIGN_CENTER, 0);

  // Left voltage label
  voltage_left_label = lv_label_create(main_screen);
  lv_obj_align(voltage_left_label, LV_ALIGN_TOP_LEFT, 3, 12);  // Adjust Y position slightly for smaller font
  lv_obj_set_style_text_font(voltage_left_label, &lv_font_montserrat_12, 0);  // Much smaller font for voltage
  lv_obj_set_style_text_color(voltage_left_label,
                            darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Right voltage label
  voltage_right_label = lv_label_create(main_screen);
  lv_obj_align(voltage_right_label, LV_ALIGN_TOP_RIGHT, -3, 12);  // Adjust Y position slightly for smaller font
  lv_obj_set_style_text_font(voltage_right_label, &lv_font_montserrat_12, 0);  // Much smaller font for voltage
  lv_obj_set_style_text_color(voltage_right_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  // Right-align right voltage so numbers grow from right to left
  lv_obj_set_style_text_align(voltage_right_label, LV_TEXT_ALIGN_RIGHT, 0);

  // Middle section - power display with individual character positions
  // Layout: [tens][ones][.][tenths] kW  (4 positions for numbers + decimal)
  int power_char_width = 16;      // Character width spacing
  int power_decimal_width = 8;    // Decimal point width
  int power_char_height = 28;     // Character height
  int power_start_x = -1;         // Start position from left (shifted left by 6px total)
  int power_start_y = 54 - 14;    // Y position (middle section, offset up by 14 total - shifted up 4px more)

  // Position array for each character (X offsets)
  int power_char_positions[4] = {
    power_start_x + 0 * power_char_width,           // Position 0: tens digit
    power_start_x + 1 * power_char_width,           // Position 1: ones digit
    power_start_x + 2 * power_char_width,           // Position 2: decimal point
    power_start_x + 2 * power_char_width + power_decimal_width  // Position 3: tenths digit
  };

  for (int i = 0; i < 4; i++) {
    power_char_labels[i] = lv_label_create(main_screen);

    // Use larger font for numbers, smaller for decimal point
    lv_obj_set_style_text_font(power_char_labels[i], &lv_font_montserrat_24, 0);  // Large font for power numbers
    lv_obj_set_style_text_color(power_char_labels[i], darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
    lv_obj_set_style_text_align(power_char_labels[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_all(power_char_labels[i], 0, 0);  // No padding

    // Set width based on position
    int width = (i == 2) ? power_decimal_width : power_char_width;  // Decimal point gets narrow width
    lv_obj_set_size(power_char_labels[i], width, power_char_height);
    lv_obj_set_pos(power_char_labels[i], power_char_positions[i], power_start_y);

    // Initialize with empty text
    lv_label_set_text(power_char_labels[i], "");
  }

  // Power unit label ("kW" text only) - position after the numbers
  power_unit_label = lv_label_create(main_screen);
  lv_obj_align(power_unit_label, LV_ALIGN_LEFT_MID, power_start_x + 4 * power_char_width + power_decimal_width + 2 - 20 + 3, -7);  // Position shifted left 20px, down 3px, then right 3px
  lv_obj_set_style_text_font(power_unit_label, &lv_font_montserrat_10, 0);  // Even smaller font for unit
  lv_obj_set_style_text_color(power_unit_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_label_set_text(power_unit_label, "kW");  // Static unit text

  // Performance mode label
  perf_mode_label = lv_label_create(main_screen);
  lv_obj_align(perf_mode_label, LV_ALIGN_RIGHT_MID, -14, -17);
  lv_obj_set_style_text_font(perf_mode_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(perf_mode_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  // Ensure text within the label is centered
  lv_obj_set_style_text_align(perf_mode_label, LV_TEXT_ALIGN_CENTER, 0);

  // Armed time label - adjust position now that there's no bluetooth icon
  armed_time_label = lv_label_create(main_screen);
  lv_obj_align(armed_time_label, LV_ALIGN_RIGHT_MID, -15, -3);
  lv_obj_set_style_text_font(armed_time_label, &lv_font_montserrat_14, 0);  // Use Montserrat font (has colon)
  lv_obj_set_style_text_color(armed_time_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  // Right-align time so numbers grow from right to left
  lv_obj_set_style_text_align(armed_time_label, LV_TEXT_ALIGN_RIGHT, 0);

  // Bottom section - altitude and temperatures
  // Create individual character position labels for fixed positioning
  // Layout: [thousands][hundreds][tens][ones][.][tenths][m]  (7 positions total)
  int char_width = 19;      // Slightly tighter character width spacing
  int decimal_width = 8;    // Slightly wider decimal point for proportional spacing
  int char_height = 24;     // Larger character height
  int unit_width = 12;      // Narrower width for unit character to reduce buffer

  // Calculate total width needed and position from the right
  int total_width = 5 * char_width + decimal_width + unit_width;  // 5 full chars + decimal + narrower unit
  int start_x = 120 - total_width - 5;  // Start 5px from right edge (moved left 3px more)
  int start_y = 128 - 5 - char_height;  // Y position

  // Position array for each character (X offsets)
  int char_positions[7] = {
    start_x + 0 * char_width,           // Position 0: thousands digit
    start_x + 1 * char_width,           // Position 1: hundreds digit
    start_x + 2 * char_width,           // Position 2: tens digit
    start_x + 3 * char_width,           // Position 3: ones digit
    start_x + 4 * char_width,           // Position 4: decimal point (will adjust width)
    start_x + 4 * char_width + decimal_width,  // Position 5: tenths digit
    start_x + 5 * char_width + decimal_width       // Position 6: unit letter (moved 5px right total)
  };

  for (int i = 0; i < 7; i++) {
    altitude_char_labels[i] = lv_label_create(main_screen);

    // Use smaller font for unit character (position 6), larger font for others
    if (i == 6) {
      lv_obj_set_style_text_font(altitude_char_labels[i], &lv_font_montserrat_12, 0);  // Even smaller font for unit character
    } else {
      lv_obj_set_style_text_font(altitude_char_labels[i], &lv_font_montserrat_28, 0);  // Two font sizes larger for numbers
    }

    lv_obj_set_style_text_color(altitude_char_labels[i], darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
    lv_obj_set_style_text_align(altitude_char_labels[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_all(altitude_char_labels[i], 0, 0);  // No padding

    // Apply comprehensive clean styling to prevent any background artifacts
    lv_obj_set_style_bg_opa(altitude_char_labels[i], LV_OPA_0, 0);  // Transparent background
    lv_obj_set_style_border_width(altitude_char_labels[i], 0, 0);  // No border
    lv_obj_set_style_outline_width(altitude_char_labels[i], 0, 0);  // No outline
    lv_obj_set_style_shadow_width(altitude_char_labels[i], 0, 0);  // No shadow
    lv_obj_set_style_border_opa(altitude_char_labels[i], LV_OPA_0, 0);  // Transparent border
    lv_obj_set_style_outline_opa(altitude_char_labels[i], LV_OPA_0, 0);  // Transparent outline
    lv_obj_set_style_shadow_opa(altitude_char_labels[i], LV_OPA_0, 0);  // Transparent shadow
    lv_obj_set_style_bg_color(altitude_char_labels[i], lv_color_hex(0x000000), 0);  // Explicit transparent background

    // Set width based on position and display mode
    int width = char_width;  // Default to normal width
    if (i == 4) {
      // Position 4 gets narrow width only when displaying meters (with decimal point)
      // For feet, position 4 should get normal width since it holds a digit
      width = decimal_width;  // This will be overridden dynamically during updates
    } else if (i == 6) {
      // Position 6 (unit character) gets narrower width to reduce buffer
      width = unit_width;
    }
    lv_obj_set_size(altitude_char_labels[i], width, char_height);

    // Adjust Y position for unit character to align with baseline
    int y_pos = start_y;
    if (i == 6) {
      y_pos += 11;  // Move smaller unit character down 11 pixels to align with baseline of 28pt font
    }
    lv_obj_set_pos(altitude_char_labels[i], char_positions[i], y_pos);

    // Initialize with empty text
    lv_label_set_text(altitude_char_labels[i], "");
  }

  // Create temperature background rectangles first (behind text)
  // Battery temperature background (top section: Y=70 to Y=89, X=117 to X=148)
  batt_temp_bg = createTempBackground(main_screen, 117, 70, 31, 19);

  // ESC temperature background (middle section: Y=89 to Y=109, X=117 to X=148)
  esc_temp_bg = createTempBackground(main_screen, 117, 89, 31, 20);

  // Motor temperature background (bottom section: Y=109 to Y=128, X=117 to X=148)
  motor_temp_bg = createTempBackground(main_screen, 117, 109, 31, 19);

  // Create temperature labels - adjust positions to align with divider lines
  batt_temp_label = lv_label_create(main_screen);
  // Align bottom-right, adjust Y offset for spacing, move left to avoid new section
  lv_obj_align(batt_temp_label, LV_ALIGN_BOTTOM_RIGHT, -11, -39);
  lv_obj_set_style_text_font(batt_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(batt_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(batt_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_border_width(batt_temp_label, 0, 0);  // No border
  lv_obj_set_style_outline_width(batt_temp_label, 0, 0);  // No outline
  lv_obj_set_style_shadow_width(batt_temp_label, 0, 0);  // No shadow
  lv_obj_set_style_bg_color(batt_temp_label, lv_color_hex(0x000000), 0);  // Explicit transparent background
  lv_obj_set_style_pad_left(batt_temp_label, 2, 0);
  lv_obj_set_style_pad_right(batt_temp_label, 2, 0);
  lv_obj_set_style_pad_top(batt_temp_label, 1, 0);
  lv_obj_set_style_pad_bottom(batt_temp_label, 1, 0);
  lv_obj_set_style_text_align(batt_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values

  // Create letter label for B
  batt_letter_label = lv_label_create(main_screen);
  // Align bottom-left, moved left 2px and down 7px total, then up 1px
  lv_obj_align(batt_letter_label, LV_ALIGN_BOTTOM_LEFT, 116, -41);
  lv_obj_set_style_text_font(batt_letter_label, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(batt_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(batt_letter_label, LV_OPA_0, 0);  // Transparent background
  lv_obj_set_style_border_width(batt_letter_label, 0, 0);  // No border
  lv_obj_set_style_shadow_width(batt_letter_label, 0, 0);  // No shadow
  lv_obj_set_style_pad_left(batt_letter_label, 2, 0);
  lv_obj_set_style_pad_right(batt_letter_label, 2, 0);
  lv_obj_set_style_pad_top(batt_letter_label, 1, 0);
  lv_obj_set_style_pad_bottom(batt_letter_label, 1, 0);
  lv_label_set_text(batt_letter_label, "B");

  esc_temp_label = lv_label_create(main_screen);
  // Align bottom-right, adjust Y offset for spacing, move left to avoid new section
  lv_obj_align(esc_temp_label, LV_ALIGN_BOTTOM_RIGHT, -11, -20);
  lv_obj_set_style_text_font(esc_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(esc_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(esc_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_border_width(esc_temp_label, 0, 0);  // No border
  lv_obj_set_style_outline_width(esc_temp_label, 0, 0);  // No outline
  lv_obj_set_style_shadow_width(esc_temp_label, 0, 0);  // No shadow
  lv_obj_set_style_bg_color(esc_temp_label, lv_color_hex(0x000000), 0);  // Explicit transparent background
  lv_obj_set_style_pad_left(esc_temp_label, 2, 0);
  lv_obj_set_style_pad_right(esc_temp_label, 2, 0);
  lv_obj_set_style_pad_top(esc_temp_label, 1, 0);
  lv_obj_set_style_pad_bottom(esc_temp_label, 1, 0);
  lv_obj_set_style_text_align(esc_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values

  // Create letter label for E
  esc_letter_label = lv_label_create(main_screen);
  // Align bottom-left, moved left 2px and down 7px total, then up 1px
  lv_obj_align(esc_letter_label, LV_ALIGN_BOTTOM_LEFT, 116, -22);
  lv_obj_set_style_text_font(esc_letter_label, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(esc_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(esc_letter_label, LV_OPA_0, 0);  // Transparent background
  lv_obj_set_style_border_width(esc_letter_label, 0, 0);  // No border
  lv_obj_set_style_shadow_width(esc_letter_label, 0, 0);  // No shadow
  lv_obj_set_style_pad_left(esc_letter_label, 2, 0);
  lv_obj_set_style_pad_right(esc_letter_label, 2, 0);
  lv_obj_set_style_pad_top(esc_letter_label, 1, 0);
  lv_obj_set_style_pad_bottom(esc_letter_label, 1, 0);
  lv_label_set_text(esc_letter_label, "E");

  motor_temp_label = lv_label_create(main_screen);
  // Align bottom-right, adjust Y offset for spacing, move left to avoid new section
  lv_obj_align(motor_temp_label, LV_ALIGN_BOTTOM_RIGHT, -11, 0);
  lv_obj_set_style_text_font(motor_temp_label, &lv_font_montserrat_12, 0);

  // Set explicit size to prevent auto-sizing that could overlap with altitude
  lv_obj_set_size(motor_temp_label, 25, 18);  // Fixed width/height to contain temperature numbers
  lv_obj_add_flag(motor_temp_label, LV_OBJ_FLAG_IGNORE_LAYOUT);  // Prevent layout effects
  lv_obj_set_style_text_color(motor_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(motor_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_border_width(motor_temp_label, 0, 0);  // No border
  lv_obj_set_style_outline_width(motor_temp_label, 0, 0);  // No outline
  lv_obj_set_style_shadow_width(motor_temp_label, 0, 0);  // No shadow
  lv_obj_set_style_bg_color(motor_temp_label, lv_color_hex(0x000000), 0);  // Explicit transparent background
  lv_obj_set_style_border_opa(motor_temp_label, LV_OPA_0, 0);  // Transparent border
  lv_obj_set_style_outline_opa(motor_temp_label, LV_OPA_0, 0);  // Transparent outline
  lv_obj_set_style_shadow_opa(motor_temp_label, LV_OPA_0, 0);  // Transparent shadow
  lv_obj_set_style_pad_left(motor_temp_label, 2, 0);
  lv_obj_set_style_pad_right(motor_temp_label, 2, 0);
  lv_obj_set_style_pad_top(motor_temp_label, 1, 0);
  lv_obj_set_style_pad_bottom(motor_temp_label, 1, 0);
  lv_obj_set_style_text_align(motor_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values

  // Create letter label for M
  motor_letter_label = lv_label_create(main_screen);
  // Align bottom-left, moved left 2px and down 7px total, then down 1px more
  lv_obj_align(motor_letter_label, LV_ALIGN_BOTTOM_LEFT, 116, -1);
  lv_obj_set_style_text_font(motor_letter_label, &lv_font_montserrat_10, 0);

  // Set explicit size to prevent any sizing issues
  lv_obj_set_size(motor_letter_label, 12, 15);  // Fixed size for single letter
  lv_obj_add_flag(motor_letter_label, LV_OBJ_FLAG_IGNORE_LAYOUT);  // Prevent layout effects
  lv_obj_set_style_text_color(motor_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(motor_letter_label, LV_OPA_0, 0);  // Transparent background
  lv_obj_set_style_border_width(motor_letter_label, 0, 0);  // No border
  lv_obj_set_style_shadow_width(motor_letter_label, 0, 0);  // No shadow
  lv_obj_set_style_outline_width(motor_letter_label, 0, 0);  // No outline
  lv_obj_set_style_border_opa(motor_letter_label, LV_OPA_0, 0);  // Transparent border
  lv_obj_set_style_outline_opa(motor_letter_label, LV_OPA_0, 0);  // Transparent outline
  lv_obj_set_style_shadow_opa(motor_letter_label, LV_OPA_0, 0);  // Transparent shadow
  lv_obj_set_style_bg_color(motor_letter_label, lv_color_hex(0x000000), 0);  // Explicit transparent background
  lv_obj_set_style_pad_left(motor_letter_label, 2, 0);
  lv_obj_set_style_pad_right(motor_letter_label, 2, 0);
  lv_obj_set_style_pad_top(motor_letter_label, 1, 0);
  lv_obj_set_style_pad_bottom(motor_letter_label, 1, 0);
  lv_label_set_text(motor_letter_label, "M");

  // Draw divider lines
  // Create horizontal line between top and middle sections
  lv_obj_t* h_line1 = lv_line_create(main_screen);
  static lv_point_t h_line1_points[] = {{0, 37}, {SCREEN_WIDTH, 37}};
  lv_line_set_points(h_line1, h_line1_points, 2);
  lv_obj_set_style_line_color(h_line1,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line1, 1, LV_PART_MAIN);

  // Create horizontal line between middle and bottom sections (stop at new section boundary)
  lv_obj_t* h_line2 = lv_line_create(main_screen);
  static lv_point_t h_line2_points[] = {{0, 70}, {148, 70}};
  lv_line_set_points(h_line2, h_line2_points, 2);
  lv_obj_set_style_line_color(h_line2,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line2, 1, LV_PART_MAIN);

  // Create vertical line in middle section (shifted left by 6 pixels)
  lv_obj_t* v_line1 = lv_line_create(main_screen);
  static lv_point_t v_line1_points[] = {{102, 37}, {102, 70}};
  lv_line_set_points(v_line1, v_line1_points, 2);
  lv_obj_set_style_line_color(v_line1,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line1, 1, LV_PART_MAIN);

  // Create vertical line in bottom section (moved left 3px)
  lv_obj_t* v_line2 = lv_line_create(main_screen);
  static lv_point_t v_line2_points[] = {{117, 70}, {117, 128}};
  lv_line_set_points(v_line2, v_line2_points, 2);
  lv_obj_set_style_line_color(v_line2,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line2, 1, LV_PART_MAIN);

  // Create vertical line for new far-right section (12px from right edge)
  lv_obj_t* v_line3 = lv_line_create(main_screen);
  static lv_point_t v_line3_points[] = {{148, 37}, {148, 128}};
  lv_line_set_points(v_line3, v_line3_points, 2);
  lv_obj_set_style_line_color(v_line3,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line3, 1, LV_PART_MAIN);

  // Create horizontal dividers for temperature section
  // Line between B and E at Y=89 (start from moved vertical line)
  lv_obj_t* h_line3 = lv_line_create(main_screen);
  static lv_point_t h_line3_points[] = {{117, 89}, {148, 89}};
  lv_line_set_points(h_line3, h_line3_points, 2);
  lv_obj_set_style_line_color(h_line3,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line3, 1, LV_PART_MAIN);

  // Line between E and M at Y=109 (start from moved vertical line)
  lv_obj_t* h_line4 = lv_line_create(main_screen);
  static lv_point_t h_line4_points[] = {{117, 109}, {148, 109}};
  lv_line_set_points(h_line4, h_line4_points, 2);
  lv_obj_set_style_line_color(h_line4,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line4, 1, LV_PART_MAIN);

  // Create arm indicator (initially hidden)
  arm_indicator = lv_obj_create(main_screen);
  lv_obj_set_size(arm_indicator, 46, 33);
  lv_obj_set_pos(arm_indicator, 102, 37);
  lv_obj_set_style_border_width(arm_indicator, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(arm_indicator, 0, LV_PART_MAIN);  // Ensure sharp corners
  // Move to background and hide initially
  lv_obj_move_background(arm_indicator);
  lv_obj_add_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);

  // Create cruise control icon (initially hidden)
  cruise_icon_img = lv_img_create(main_screen);
  lv_img_set_src(cruise_icon_img, &cruise_control_340255_30);  // Use the new 30x30 image descriptor

  // Set icon color using recoloring based on theme
  lv_color_t icon_color;
  if (darkMode) {
    icon_color = lv_color_white();  // White icon on dark background
  } else {
    icon_color = lv_color_black();  // Black icon on light background
  }
  lv_obj_set_style_img_recolor(cruise_icon_img, icon_color, LV_PART_MAIN);
  lv_obj_set_style_img_recolor_opa(cruise_icon_img, LV_OPA_COVER, LV_PART_MAIN);  // Make icon fully opaque

  lv_obj_align(cruise_icon_img, LV_ALIGN_CENTER, 5, -11);  // Align center, offset right 5, up 11
  lv_obj_move_foreground(cruise_icon_img);  // Ensure icon is on the top layer
  lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide initially

  // Create charging icon (initially hidden)
  charging_icon_img = lv_img_create(main_screen);
  lv_img_set_src(charging_icon_img, &energy_539741_26);
  lv_obj_align_to(charging_icon_img, battery_label, LV_ALIGN_OUT_RIGHT_MID, 3, 0);  // Align to right of battery label

  // Set charging icon color based on theme
  lv_obj_set_style_img_recolor(charging_icon_img, icon_color, LV_PART_MAIN);  // Use same icon_color as cruise
  lv_obj_set_style_img_recolor_opa(charging_icon_img, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_move_foreground(charging_icon_img);  // Ensure icon is on top
  lv_obj_add_flag(charging_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide initially

  // Create arm fail warning icon (initially hidden)
  arm_fail_warning_icon_img = lv_img_create(main_screen);
  lv_img_set_src(arm_fail_warning_icon_img, &warning_2135850_30);
  // Align in the same position as the cruise icon
  lv_obj_align(arm_fail_warning_icon_img, LV_ALIGN_CENTER, 12, -9);

  // Set icon color based on theme (using the same logic as cruise icon)
  lv_obj_set_style_img_recolor(arm_fail_warning_icon_img, icon_color, LV_PART_MAIN);
  lv_obj_set_style_img_recolor_opa(arm_fail_warning_icon_img, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_move_foreground(arm_fail_warning_icon_img);  // Ensure icon is on top
  lv_obj_add_flag(arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);  // Hide initially

  // Create semi-transparent overlay for the spinner
  spinner_overlay = lv_obj_create(main_screen);
  lv_obj_set_size(spinner_overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_set_pos(spinner_overlay, 0, 0);
  lv_obj_set_style_bg_color(spinner_overlay, darkMode ? LVGL_BLACK : LVGL_WHITE, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(spinner_overlay, LV_OPA_70, LV_PART_MAIN);  // 70% opacity
  lv_obj_set_style_border_width(spinner_overlay, 0, LV_PART_MAIN);

  // Create spinning animation at the top center - now place on top of overlay
  spinner = lv_spinner_create(spinner_overlay, 1000, 60);  // 1000ms period, 60 arcade width
  lv_obj_set_size(spinner, 80, 80);  // Even larger spinner for visibility
  lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 0);  // Position at center of screen

  // Make the spinner more visible with strong colors and good contrast
  lv_obj_set_style_arc_width(spinner, 8, LV_PART_INDICATOR);  // Thicker arc for indicator
  lv_obj_set_style_arc_width(spinner, 8, LV_PART_MAIN);  // Thicker background arc

  // Apply proper shading and gradients for a more attractive appearance
  lv_obj_set_style_arc_rounded(spinner, true, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(spinner, true, LV_PART_MAIN);

  // Set spinner colors with better contrast
  lv_obj_set_style_arc_color(spinner, darkMode ? lv_color_make(100, 100, 100) : lv_color_make(230, 230, 230), LV_PART_MAIN);
  lv_obj_set_style_arc_color(spinner, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);

  // Hide the overlay by default
  lv_obj_add_flag(spinner_overlay, LV_OBJ_FLAG_HIDDEN);

  // Create climb rate indicator horizontal divider lines in the far-right section
  // Section spans from y=37 to y=128 (91 pixels), with 13 horizontal divider lines
  const int climb_section_start_y = 37;
  const int climb_section_end_y = 128;

  for (int i = 0; i < 13; i++) {
    // Calculate evenly spaced positions for 13 lines
    int y_pos = climb_section_start_y + (i * 7);  // 7 pixels between each line

    // Create horizontal divider line running from section start to screen edge
    climb_rate_divider_lines[i] = lv_line_create(main_screen);
    static lv_point_t line_points[13][2];  // Static array for all line points
    line_points[i][0].x = 148;  // Start at section boundary
    line_points[i][0].y = y_pos;
    line_points[i][1].x = 160;  // End at screen edge
    line_points[i][1].y = y_pos;

    lv_line_set_points(climb_rate_divider_lines[i], line_points[i], 2);

    // Make the center line (line 6) special: 3 pixels wide and black
    if (i == 6) {
      lv_obj_set_style_line_color(climb_rate_divider_lines[i], LVGL_BLACK, LV_PART_MAIN);
      lv_obj_set_style_line_width(climb_rate_divider_lines[i], 3, LV_PART_MAIN);
    } else {
      lv_obj_set_style_line_color(climb_rate_divider_lines[i], LVGL_GRAY, LV_PART_MAIN);
      lv_obj_set_style_line_width(climb_rate_divider_lines[i], 1, LV_PART_MAIN);
    }
  }

  // Create fill sections between the horizontal divider lines
  for (int i = 0; i < 12; i++) {
    // Calculate position between lines
    int y_start = climb_section_start_y + (i * 7);
    int y_end = climb_section_start_y + ((i + 1) * 7);
    int section_height = y_end - y_start;

    climb_rate_fill_sections[i] = lv_obj_create(main_screen);
    lv_obj_set_size(climb_rate_fill_sections[i], 12, section_height);  // Full width of section
    lv_obj_set_pos(climb_rate_fill_sections[i], 148, y_start);

    // Set styling - initially transparent
    lv_obj_set_style_border_width(climb_rate_fill_sections[i], 0, LV_PART_MAIN);
    lv_obj_set_style_radius(climb_rate_fill_sections[i], 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(climb_rate_fill_sections[i], LV_OPA_0, LV_PART_MAIN);

    // Move fill sections behind the divider lines
    lv_obj_move_background(climb_rate_fill_sections[i]);
  }

  // Create critical alert border (initially hidden) - moved from updates file
  if (critical_border == NULL) {
    critical_border = lv_obj_create(main_screen);
    lv_obj_set_size(critical_border, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(critical_border, 0, 0);
    lv_obj_set_style_border_width(critical_border, 4, LV_PART_MAIN);
    lv_obj_set_style_border_color(critical_border, LVGL_RED, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(critical_border, LV_OPA_0, LV_PART_MAIN);  // Transparent background
    lv_obj_set_style_radius(critical_border, 0, LV_PART_MAIN);  // Sharp corners
    lv_obj_add_flag(critical_border, LV_OBJ_FLAG_HIDDEN);  // Initially hidden
    // Move border to front so it's visible over all other elements
    lv_obj_move_foreground(critical_border);
  }

  // Load the screen
  lv_scr_load(main_screen);
}
