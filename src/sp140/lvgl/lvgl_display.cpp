#include "../../../inc/sp140/lvgl/lvgl_display.h"
#include "../../../inc/version.h"
#include "../../../inc/sp140/structs.h"
#include "../../../inc/sp140/bms.h"
#include <Adafruit_ST7735.h>
#include <vector>
#include <string>
#include <map>
#include <utility>
#include "../../../inc/sp140/simple_monitor.h"  // for sensor ID helpers
#include "../../../inc/sp140/alert_display.h"

// Forward declarations for alert counter UI
void setupAlertCounterUI(bool darkMode);
void updateAlertCounterDisplay(const AlertCounts& counts);

#include "../../assets/img/cruise-control-340255-30.c"  // Cruise control icon  // NOLINT(build/include)
#include "../../assets/img/energy-539741-26.c"  // Charging icon  // NOLINT(build/include)
#include "../../assets/img/warning_2135850_30.c"  // Warning icon  // NOLINT(build/include)
#include "../../assets/fonts/roboto_mono_16.c"  // Monospace font for numbers  // NOLINT(build/include)


// External font declaration
extern const lv_font_t roboto_mono_16;

// Display dimensions
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

// LVGL buffer size - optimize for our display
// Use 1/4 of the screen size to balance memory usage and performance
#define LVGL_BUFFER_SIZE (SCREEN_WIDTH * (SCREEN_HEIGHT / 4))

// LVGL refresh time in ms - match the config file setting
#define LVGL_REFRESH_TIME 40


#define BATT_TEMP_WARNING 45.0f
#define BATT_TEMP_CRITICAL 55.0f
#define ESC_TEMP_WARNING 90.0f
#define ESC_TEMP_CRITICAL 100.0f
#define MOTOR_TEMP_WARNING 90.0f
#define MOTOR_TEMP_CRITICAL 110.0f

#define BATT_CRITICAL_SOC_THRESHOLD 5.0f
#define BATT_WARNING_SOC_THRESHOLD 20.0f
#define CELL_VOLTAGE_WARNING 3.2f
#define CELL_VOLTAGE_CRITICAL 2.95f

// Notification System Definitions
/*
#define NOTIFICATION_LATCH_MS 2000 // Minimum time to display a notification
#define NOTIFICATION_CYCLE_MS 2000 // Time between cycling to the *next* notification

struct NotificationInfo {
  const char* message;
  lv_color_t color;
  bool is_error;  // true for error (red), false for warning (orange)
  uint32_t last_active_time;  // Last time the condition for this notification was met
  // Removed display_start_time as it's managed separately now
};
*/

// Define notification message strings
/*
const char* const WARN_LOW_CELL = "BAT-LC";
const char* const CRIT_LOW_CELL = "BAT-LC";
// ... existing code ...
const char* const ERR_BMS_TIMEOUT = "BMS-NC";
const char* const ERR_ESC_TIMEOUT = "ESC-NC";
// Add more message strings as needed...
*/

// Global variables
static lv_disp_drv_t disp_drv;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LVGL_BUFFER_SIZE];
static Adafruit_ST7735* tft_driver = nullptr;

// The last time LVGL was updated
static uint32_t lvgl_last_update = 0;

// Main screen objects
lv_obj_t* main_screen = NULL;
static lv_obj_t* battery_bar = NULL;
static lv_obj_t* battery_label = NULL;
static lv_obj_t* voltage_left_label = NULL;
static lv_obj_t* voltage_right_label = NULL;
static lv_obj_t* power_char_labels[4] = {NULL};  // Individual character position labels for power: [tens][ones][.][tenths]
static lv_obj_t* power_unit_label = NULL;  // Separate label for "kW" unit text
static lv_obj_t* power_bar = NULL;
static lv_obj_t* perf_mode_label = NULL;
static lv_obj_t* armed_time_label = NULL;
static lv_obj_t* altitude_char_labels[7] = {NULL};  // Individual character position labels
static lv_obj_t* batt_temp_label = NULL;
static lv_obj_t* esc_temp_label = NULL;
static lv_obj_t* motor_temp_label = NULL;
static lv_obj_t* arm_indicator = NULL;
static lv_obj_t* spinner = NULL;       // For the spinning animation
static lv_obj_t* spinner_overlay = NULL;  // Overlay for the spinner
static lv_obj_t* batt_letter_label = NULL;  // Letter label for Battery temp
static lv_obj_t* esc_letter_label = NULL;  // Letter label for ESC temp
static lv_obj_t* motor_letter_label = NULL;  // Letter label for Motor temp
static lv_obj_t* batt_temp_bg = NULL;  // Background rectangle for Battery temp section
static lv_obj_t* esc_temp_bg = NULL;  // Background rectangle for ESC temp section
static lv_obj_t* motor_temp_bg = NULL;  // Background rectangle for Motor temp section

// Alert counter UI objects
static lv_obj_t* warning_counter_circle = NULL;
static lv_obj_t* warning_counter_label = NULL;
static lv_obj_t* critical_counter_circle = NULL;
static lv_obj_t* critical_counter_label = NULL;

// Alert text carousel objects
static lv_obj_t* alert_text_label = NULL;
static lv_timer_t* alert_cycle_timer = NULL;
// Snapshot state shared between timer and external updater
static AlertSnapshot currentSnap{};
static uint8_t currentIdx = 0;

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
  lv_obj_set_style_text_color(alert_text_label, snap.criticalMode ? lv_color_make(255,0,0) : lv_color_make(255,165,0), 0);
  lv_obj_clear_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
}
// static lv_obj_t* warning_label = NULL;  // New label for warnings/errors // REMOVED
lv_obj_t* cruise_icon_img = NULL;  // Cruise control icon image object
static lv_obj_t* charging_icon_img = NULL;  // Charging icon image object
static lv_obj_t* arm_fail_warning_icon_img = NULL;  // Arm fail warning icon

// Climb rate indicator horizontal divider lines (13 lines total)
static lv_obj_t* climb_rate_divider_lines[13] = {NULL};

// Climb rate fill sections (12 sections between the 13 lines)
static lv_obj_t* climb_rate_fill_sections[12] = {NULL};

// Notification Counter Objects
/* // REMOVED
static lv_obj_t* error_counter_circle = NULL;
static lv_obj_t* error_counter_label = NULL;
static lv_obj_t* warning_counter_circle = NULL;
static lv_obj_t* warning_counter_label = NULL;
*/

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

// Function forward declarations
void setupMainScreen(bool darkMode);
void updateClimbRateIndicator(float climbRate);
void updateLvglMainScreenWithTestData(const STR_DEVICE_DATA_140_V1& deviceData);


// --- Cruise Icon Flashing ---
static lv_timer_t* cruise_flash_timer = NULL;
static int cruise_flash_count = 0;
static bool isFlashingCruiseIcon = false;
static void cruise_flash_timer_cb(lv_timer_t* timer);  // Forward declaration
static lv_color_t original_cruise_icon_color;  // To restore color after flashing
// --- End Cruise Icon Flashing ---

// --- Arm Fail Icon Flashing ---
static lv_timer_t* arm_fail_flash_timer = NULL;
static int arm_fail_flash_count = 0;
static bool isFlashingArmFailIcon = false;
static void arm_fail_flash_timer_cb(lv_timer_t* timer);  // Forward declaration
static lv_color_t original_arm_fail_icon_color;  // To restore color after flashing
// --- End Arm Fail Icon Flashing ---

// --- Critical Alert Border Flashing ---
static lv_obj_t* critical_border = NULL;
static lv_timer_t* critical_border_flash_timer = NULL;
static bool isFlashingCriticalBorder = false;
static void critical_border_flash_timer_cb(lv_timer_t* timer);  // Forward declaration
// --- End Critical Alert Border Flashing ---

void setupLvglBuffer() {
  // Initialize LVGL library
  USBSerial.println("Initializing LVGL");
  lv_init();
  USBSerial.println("LVGL initialized");

  // Setup buffer for LVGL
  USBSerial.println("Setting up LVGL buffer");
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUFFER_SIZE);
  USBSerial.println("LVGL buffer initialized");
}

void setupLvglDisplay(const STR_DEVICE_DATA_140_V1& deviceData, int8_t dc_pin, int8_t rst_pin, SPIClass* spi) {
  USBSerial.println("Setting up LVGL display");

  // Create the TFT driver instance if not already created
  if (tft_driver == nullptr) {
    tft_driver = new Adafruit_ST7735(spi, displayCS, dc_pin, rst_pin);

    // Initialize the display
    tft_driver->initR(INITR_BLACKTAB);
    tft_driver->setRotation(deviceData.screen_rotation);
    tft_driver->fillScreen(ST77XX_BLACK);
  }

  // Initialize LVGL buffer
  setupLvglBuffer();

  // Initialize display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);

  // Set display driver properties
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.draw_buf = &draw_buf;

  // Register the display driver
  lv_disp_drv_register(&disp_drv);

  // Set LVGL default theme - using default font
  lv_theme_t* theme = lv_theme_default_init(
    lv_disp_get_default(),                // Display
    lv_palette_main(LV_PALETTE_BLUE),     // Primary color
    lv_palette_main(LV_PALETTE_AMBER),    // Secondary color
    deviceData.theme == 1,                // Dark mode
    LV_FONT_DEFAULT);                     // Default font

  lv_disp_set_theme(lv_disp_get_default(), theme);
}

// Optimize the flush callback to minimize SPI transfers
void lvgl_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  // Make sure display CS is selected
  digitalWrite(displayCS, LOW);

  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  // Set drawing window
  tft_driver->startWrite();
  tft_driver->setAddrWindow(area->x1, area->y1, w, h);

  // Push colors - using DMA if available
  uint32_t len = w * h;
  tft_driver->writePixels((uint16_t*)color_p, len);
  tft_driver->endWrite();

  // Deselect display CS when done
  digitalWrite(displayCS, HIGH);

  // Indicate to LVGL that flush is done
  lv_disp_flush_ready(disp);
}

// LVGL tick handler - to be called from timer or in main loop
void lv_tick_handler() {
  static uint32_t last_tick = 0;
  uint32_t current_ms = millis();

  if (current_ms - last_tick > 5) {  // 5ms tick rate for LVGL
    lv_tick_inc(current_ms - last_tick);
    last_tick = current_ms;
  }
}

// Update LVGL - call this regularly
void updateLvgl() {
  uint32_t current_ms = millis();

  // Update LVGL at the defined refresh rate
  if (current_ms - lvgl_last_update > LVGL_REFRESH_TIME) {
    lv_tick_handler();
    lv_task_handler();
    lvgl_last_update = current_ms;
  }
}

void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  USBSerial.println("Displaying LVGL splash screen");

  // Make sure display CS is selected
  digitalWrite(displayCS, LOW);

  // Create a new screen for the splash
  lv_obj_t* splash_screen = lv_obj_create(NULL);
  lv_scr_load(splash_screen);

  // Disable scrollbars
  lv_obj_clear_flag(splash_screen, LV_OBJ_FLAG_SCROLLABLE);

  // Set background color based on theme
  lv_obj_set_style_bg_color(splash_screen,
                           deviceData.theme == 1 ? lv_color_black() : lv_color_white(),
                           LV_PART_MAIN);

  // OpenPPG title
  lv_obj_t* title_label = lv_label_create(splash_screen);
  lv_label_set_text(title_label, "OpenPPG");
  // Use a larger font for the title
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title_label,
                             deviceData.theme == 1 ? lv_color_white() : lv_color_black(),
                             0);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 15);

  // Create animation for title
  lv_anim_t title_anim;
  lv_anim_init(&title_anim);
  lv_anim_set_var(&title_anim, title_label);
  lv_anim_set_values(&title_anim, 0, 255);
  lv_anim_set_time(&title_anim, 500);
  lv_anim_set_exec_cb(&title_anim, [](void* var, int32_t value) {
    lv_obj_set_style_opa((lv_obj_t*)var, value, 0);
  });
  lv_anim_start(&title_anim);

  // Version label
  lv_obj_t* version_label = lv_label_create(splash_screen);
  char version_str[10];
  snprintf(version_str, sizeof(version_str), "v%d.%d", VERSION_MAJOR, VERSION_MINOR);
  lv_label_set_text(version_label, version_str);
  lv_obj_set_style_text_font(version_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(version_label,
                             deviceData.theme == 1 ? lv_color_white() : lv_color_black(),
                             0);
  lv_obj_align(version_label, LV_ALIGN_CENTER, 0, 0);

  // Time used label
  lv_obj_t* time_label = lv_label_create(splash_screen);
  char time_str[10];
  const int hours = deviceData.armed_time / 60;
  const int minutes = deviceData.armed_time % 60;
  snprintf(time_str, sizeof(time_str), "%02d:%02d", hours, minutes);
  lv_label_set_text(time_label, time_str);
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(time_label,
                             deviceData.theme == 1 ? lv_color_white() : lv_color_black(),
                             0);
  lv_obj_align(time_label, LV_ALIGN_BOTTOM_MID, 0, -20);

  // Process LVGL for the duration
  uint32_t start_time = millis();
  while (millis() - start_time < duration) {
    updateLvgl();
    delay(LVGL_REFRESH_TIME);
  }

  // Deselect display CS when done
  digitalWrite(displayCS, HIGH);
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



  // Warning/Error message label
  /* // REMOVED
  warning_label = lv_label_create(main_screen);
  // Position above altitude area (which is now on the right)
  lv_obj_set_width(warning_label, 115);  // Limit width
  // Align warning label to the top-left of the bottom section
  lv_obj_align(warning_label, LV_ALIGN_TOP_LEFT, 5, 78);  // Position below HLine2
  lv_obj_set_style_text_font(warning_label, &lv_font_montserrat_12, 0);  // Smaller font for warnings
  lv_label_set_long_mode(warning_label, LV_LABEL_LONG_WRAP);  // Allow wrapping
  lv_label_set_text(warning_label, "");  // Initially empty
  lv_obj_add_flag(warning_label, LV_OBJ_FLAG_HIDDEN);  // Hide initially
  */

  // Create temperature background rectangles first (behind text)
  // Battery temperature background (top section: Y=70 to Y=89, X=117 to X=148)
  batt_temp_bg = lv_obj_create(main_screen);
  lv_obj_set_size(batt_temp_bg, 31, 19);  // Width: 148-117=31, Height: 89-70=19
  lv_obj_set_pos(batt_temp_bg, 117, 70);
  lv_obj_set_style_border_width(batt_temp_bg, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(batt_temp_bg, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(batt_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Initially transparent
  lv_obj_set_style_shadow_width(batt_temp_bg, 0, LV_PART_MAIN);  // No shadow
  lv_obj_set_style_outline_width(batt_temp_bg, 0, LV_PART_MAIN);  // No outline
  lv_obj_set_style_pad_all(batt_temp_bg, 0, LV_PART_MAIN);  // No padding
  lv_obj_set_style_border_opa(batt_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent border
  lv_obj_set_style_outline_opa(batt_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent outline
  lv_obj_set_style_shadow_opa(batt_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent shadow
  lv_obj_move_background(batt_temp_bg);  // Move to background layer
  lv_obj_add_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);  // Hide by default, show only when highlighting

  // ESC temperature background (middle section: Y=89 to Y=109, X=117 to X=148)
  esc_temp_bg = lv_obj_create(main_screen);
  lv_obj_set_size(esc_temp_bg, 31, 20);  // Width: 148-117=31, Height: 109-89=20
  lv_obj_set_pos(esc_temp_bg, 117, 89);
  lv_obj_set_style_border_width(esc_temp_bg, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(esc_temp_bg, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(esc_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Initially transparent
  lv_obj_set_style_shadow_width(esc_temp_bg, 0, LV_PART_MAIN);  // No shadow
  lv_obj_set_style_outline_width(esc_temp_bg, 0, LV_PART_MAIN);  // No outline
  lv_obj_set_style_pad_all(esc_temp_bg, 0, LV_PART_MAIN);  // No padding
  lv_obj_set_style_border_opa(esc_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent border
  lv_obj_set_style_outline_opa(esc_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent outline
  lv_obj_set_style_shadow_opa(esc_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent shadow
  lv_obj_move_background(esc_temp_bg);  // Move to background layer
  lv_obj_add_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);  // Hide by default, show only when highlighting

  // Motor temperature background (bottom section: Y=109 to Y=128, X=117 to X=148)
  motor_temp_bg = lv_obj_create(main_screen);
  lv_obj_set_size(motor_temp_bg, 31, 19);  // Width: 148-117=31, Height: 128-109=19
  lv_obj_set_pos(motor_temp_bg, 117, 109);
  lv_obj_set_style_border_width(motor_temp_bg, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(motor_temp_bg, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(motor_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Initially transparent
  lv_obj_set_style_shadow_width(motor_temp_bg, 0, LV_PART_MAIN);  // No shadow
  lv_obj_set_style_outline_width(motor_temp_bg, 0, LV_PART_MAIN);  // No outline
  lv_obj_set_style_pad_all(motor_temp_bg, 0, LV_PART_MAIN);  // No padding
  lv_obj_set_style_border_opa(motor_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent border
  lv_obj_set_style_outline_opa(motor_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent outline
  lv_obj_set_style_shadow_opa(motor_temp_bg, LV_OPA_0, LV_PART_MAIN);  // Transparent shadow
  lv_obj_move_background(motor_temp_bg);  // Move to background layer
  lv_obj_add_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);  // Hide by default, show only when highlighting

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

  // Ensure warning_label is drawn on top of lines and other elements in its area
  /* // REMOVED
  if (warning_label != NULL) {
      lv_obj_move_foreground(warning_label);
  }
  */

  // Create critical alert border (initially hidden)
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
  // lv_img_set_src(cruise_icon_img, &noun_cruise_control_340255_1);  // Use the old 40x40 image descriptor
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

  // Store the original color for restoring after flash
  original_cruise_icon_color = icon_color;

  // lv_obj_align(cruise_icon_img, LV_ALIGN_CENTER, 12, -12);  // Align center, then offset right 12, up 12
  lv_obj_align(cruise_icon_img, LV_ALIGN_CENTER, 5, -11);  // Align center, offset right 5, up 11 (moved left 3px)
  lv_obj_move_foreground(cruise_icon_img);  // Ensure icon is on the top layer
  // Don't hide it initially for debugging
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

  // Store the original color for restoring after flash
  original_arm_fail_icon_color = icon_color;

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

   // Load the screen
  lv_scr_load(main_screen);

  // Create alert counter UI elements
  setupAlertCounterUI(darkMode);
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
      lv_obj_align_to(warning_counter_circle, altitude_char_labels[0], LV_ALIGN_OUT_TOP_LEFT, 0, -2);
    } else {
      lv_obj_align(warning_counter_circle, LV_ALIGN_TOP_LEFT, 5, 75);
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
      lv_obj_align_to(critical_counter_circle, warning_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
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
      strcpy(buf, "9+");
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
      strcpy(buf, "9+");
    } else {
      snprintf(buf, sizeof(buf), "%u", counts.criticalCount);
    }
    lv_label_set_text(critical_counter_label, buf);
    lv_obj_clear_flag(critical_counter_circle, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(critical_counter_circle, LV_OBJ_FLAG_HIDDEN);
  }
}

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
// --- End Cruise Icon Flashing Implementation ---

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
// --- End Arm Fail Icon Flashing Implementation ---

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

  if (climbRate >= 0.05f) {
    // Positive climb rate - fill sections above center line
    int sectionsToFill = (int)(climbRate / 0.1f);
    if (sectionsToFill > 6) sectionsToFill = 6;

    for (int i = 0; i < sectionsToFill; i++) {
      int sectionIndex = 5 - i;  // Section 5 is closest to center line, going up to 0
      if (climb_rate_fill_sections[sectionIndex] != NULL) {
        lv_obj_set_style_bg_opa(climb_rate_fill_sections[sectionIndex], LV_OPA_100, LV_PART_MAIN);
        lv_obj_set_style_bg_color(climb_rate_fill_sections[sectionIndex], positive_colors[i], LV_PART_MAIN);
      }
    }
  } else if (climbRate <= -0.05f) {
    // Negative climb rate - fill sections below center line
    int sectionsToFill = (int)(-climbRate / 0.1f);
    if (sectionsToFill > 6) sectionsToFill = 6;

    for (int i = 0; i < sectionsToFill; i++) {
      int sectionIndex = 6 + i;  // Section 6 is closest to center line, going down to 11
      if (climb_rate_fill_sections[sectionIndex] != NULL) {
        lv_obj_set_style_bg_opa(climb_rate_fill_sections[sectionIndex], LV_OPA_100, LV_PART_MAIN);
        lv_obj_set_style_bg_color(climb_rate_fill_sections[sectionIndex], negative_colors[i], LV_PART_MAIN);
      }
    }
  }
  // If climb rate is between -0.05 and +0.05, no sections are filled (neutral)
}

// --- Critical Alert Border Flashing Implementation ---
static void critical_border_flash_timer_cb(lv_timer_t* timer) {
  // This callback runs within the LVGL task handler, which is already protected by lvglMutex

  if (critical_border == NULL) {
    // Safety check
    if (critical_border_flash_timer != NULL) {
      lv_timer_del(critical_border_flash_timer);
      critical_border_flash_timer = NULL;
    }
    isFlashingCriticalBorder = false;
    return;
  }

  // Toggle visibility
  if (lv_obj_has_flag(critical_border, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_clear_flag(critical_border, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(critical_border, LV_OBJ_FLAG_HIDDEN);
  }
}

void startCriticalBorderFlash() {
  // This function can be called from other tasks, so protect with mutex
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {  // Use a timeout
    if (critical_border == NULL) {
      xSemaphoreGive(lvglMutex);
      return;  // Can't flash if border doesn't exist
    }

    // If a flash timer is already running, delete it first
    if (critical_border_flash_timer != NULL) {
      lv_timer_del(critical_border_flash_timer);
      critical_border_flash_timer = NULL;
    }

    // Reset state and start flashing
    isFlashingCriticalBorder = true;

    // Start with the border visible
    lv_obj_clear_flag(critical_border, LV_OBJ_FLAG_HIDDEN);

    // Create the timer (500ms interval for on/off cycle - matches vibration rate)
    critical_border_flash_timer = lv_timer_create(critical_border_flash_timer_cb, 500, NULL);
    if (critical_border_flash_timer == NULL) {
      // Failed to create timer, reset state
      isFlashingCriticalBorder = false;
      lv_obj_add_flag(critical_border, LV_OBJ_FLAG_HIDDEN);  // Hide it again
      USBSerial.println("Error: Failed to create critical border flash timer!");
    }

    xSemaphoreGive(lvglMutex);
  } else {
     USBSerial.println("Warning: Failed to acquire LVGL mutex for startCriticalBorderFlash");
  }
}

void stopCriticalBorderFlash() {
  // This function can be called from other tasks, so protect with mutex
  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(50)) == pdTRUE) {  // Use a timeout
    if (critical_border_flash_timer != NULL) {
      lv_timer_del(critical_border_flash_timer);
      critical_border_flash_timer = NULL;
    }

    isFlashingCriticalBorder = false;

    // Hide the border
    if (critical_border != NULL) {
      lv_obj_add_flag(critical_border, LV_OBJ_FLAG_HIDDEN);
    }

    xSemaphoreGive(lvglMutex);
  } else {
     USBSerial.println("Warning: Failed to acquire LVGL mutex for stopCriticalBorderFlash");
  }
}

bool isCriticalBorderFlashing() {
  return isFlashingCriticalBorder;
}
// --- End Critical Alert Border Flashing Implementation ---

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
    for (int i = 0; i < 7; i++) {
      if (altitude_char_labels[i]) {
        lv_obj_add_flag(altitude_char_labels[i], LV_OBJ_FLAG_HIDDEN);
      }
    }
    // Move alert label to altitude position (using first altitude character as reference)
    if (altitude_char_labels[0]) {
      lv_obj_set_pos(alert_text_label, lv_obj_get_x(altitude_char_labels[0]), lv_obj_get_y(altitude_char_labels[0]));
    }
  } else {
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_14, 0);
    // Ensure altitude visible during warnings
    for (int i = 0; i < 7; i++) {
      if (altitude_char_labels[i]) {
        lv_obj_clear_flag(altitude_char_labels[i], LV_OBJ_FLAG_HIDDEN);
      }
    }
    // Re-align warning text next to circles
    if (warning_counter_circle) {
      lv_obj_align_to(alert_text_label, warning_counter_circle, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    }
    // Make warning text darker and slightly larger for readability
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_16, 0);
    // Dark orange for better readability over light background
    lv_obj_set_style_text_color(alert_text_label, lv_color_make(200,100,0), 0);
  }
  if (critical) {
    lv_obj_set_style_text_color(alert_text_label, lv_color_make(255,0,0), 0);
  }
  lv_obj_clear_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
}

void lv_hideAlertText() {
  if (alert_text_label == NULL) return;
  lv_obj_add_flag(alert_text_label, LV_OBJ_FLAG_HIDDEN);
  // Restore altitude visibility when no critical alert
  for (int i = 0; i < 7; i++) {
    if (altitude_char_labels[i]) {
      lv_obj_clear_flag(altitude_char_labels[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

void updateLvglMainScreen(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
) {
  // Make sure display CS is selected before any LVGL calls that might draw
  digitalWrite(displayCS, LOW);

  bool darkMode = (deviceData.theme == 1);
  float batteryPercent = unifiedBatteryData.soc;
  float totalVolts = unifiedBatteryData.volts;
  float lowestCellV = bmsTelemetry.lowest_cell_voltage;
  float batteryTemp = bmsTelemetry.highest_temperature;
  float escTemp = escTelemetry.cap_temp;
  float motorTemp = escTelemetry.motor_temp;
  // Check if BMS or ESC is connected
  bool bmsConnected = (bmsTelemetry.bmsState == TelemetryState::CONNECTED);
  bool escConnected = (escTelemetry.escState == TelemetryState::CONNECTED);


  // Assume main_screen is already created and loaded by setup()
  // We might need a separate function later if we want to dynamically change themes
  // For now, just ensure spinner colors match the current theme state potentially set at boot
  // Check if spinner exists before styling it
  if (spinner != NULL) {
    lv_obj_set_style_arc_color(spinner, darkMode ? lv_color_make(100, 100, 100) : lv_color_make(230, 230, 230), LV_PART_MAIN);
    lv_obj_set_style_arc_color(spinner, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
  }

  // Update battery bar and percentage
  if (bmsConnected && batteryPercent > 0) {
    lv_bar_set_value(battery_bar, (int)batteryPercent, LV_ANIM_OFF);

    // Set color based on percentage
    lv_color_t batteryColor = LVGL_RED;
    if (batteryPercent >= BATT_WARNING_SOC_THRESHOLD) {
      batteryColor = LVGL_GREEN;
    } else if (batteryPercent >= BATT_CRITICAL_SOC_THRESHOLD) {
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
        lv_label_set_text(voltage_left_label, "No BMS");
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
        if (batteryPercent > 0) {
           char batt_buffer[10];
           snprintf(batt_buffer, sizeof(batt_buffer), "%d%%", (int)batteryPercent);
           lv_label_set_text(battery_label, batt_buffer);
           lv_obj_set_style_text_color(battery_label, LVGL_BLACK, 0);
        } else {
           // If SOC is 0, maybe show voltage instead or clear? Let's clear for now.
           lv_label_set_text(battery_label, "-");
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

        // Clear all positions first
        for (int i = 0; i < 4; i++) {
          lv_label_set_text(power_char_labels[i], "");
        }

        // Format power to one decimal place (e.g., 12.5)
        int whole_part = static_cast<int>(kWatts);
        int decimal_part = static_cast<int>(round((kWatts - whole_part) * 10));

        // Extract individual digits
        int tens = whole_part / 10;
        int ones = whole_part % 10;

        // Populate character positions using static buffers
        static char power_digit_buffers[4][2];

        // Tens digit (only show if >= 10kW)
        if (tens > 0) {
          snprintf(power_digit_buffers[0], 2, "%d", tens);
          lv_label_set_text(power_char_labels[0], power_digit_buffers[0]);
        }

        // Ones digit (always show)
        snprintf(power_digit_buffers[1], 2, "%d", ones);
        lv_label_set_text(power_char_labels[1], power_digit_buffers[1]);

        // Decimal point
        lv_label_set_text(power_char_labels[2], ".");

        // Tenths digit
        snprintf(power_digit_buffers[3], 2, "%d", decimal_part);
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
      int leftmost_pos = 3; // ones position (always needed)
      if (whole_part >= 10) leftmost_pos = 2;      // tens
      if (whole_part >= 100) leftmost_pos = 1;     // hundreds
      if (whole_part >= 1000) leftmost_pos = 0;    // thousands

      // Place negative sign if needed (one position left of leftmost digit)
      if (isNegative && leftmost_pos > 0) {
        lv_label_set_text(altitude_char_labels[leftmost_pos - 1], "-");
      }

      // Thousands digit
      if (thousands > 0) {
        snprintf(digit_buffers[0], 2, "%d", thousands);
        lv_label_set_text(altitude_char_labels[0], digit_buffers[0]);
      }

      // Hundreds digit
      if (thousands > 0 || hundreds > 0) {
        snprintf(digit_buffers[1], 2, "%d", hundreds);
        lv_label_set_text(altitude_char_labels[1], digit_buffers[1]);
      }

      // Tens digit
      if (whole_part >= 10) {
        snprintf(digit_buffers[2], 2, "%d", tens);
        lv_label_set_text(altitude_char_labels[2], digit_buffers[2]);
      }

      // Ones digit (always show)
      snprintf(digit_buffers[3], 2, "%d", ones);
      lv_label_set_text(altitude_char_labels[3], digit_buffers[3]);

      // Decimal point
      lv_label_set_text(altitude_char_labels[4], ".");

      // Tenths digit
      snprintf(digit_buffers[5], 2, "%d", decimal_part);
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
      int leftmost_pos = 4; // ones position (always needed for feet)
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
        snprintf(digit_buffers_ft[0], 2, "%d", ten_thousands);
        lv_label_set_text(altitude_char_labels[0], digit_buffers_ft[0]);
      }

      // Thousands digit in position 1
      if (ten_thousands > 0 || thousands > 0) {
        snprintf(digit_buffers_ft[1], 2, "%d", thousands);
        lv_label_set_text(altitude_char_labels[1], digit_buffers_ft[1]);
      }

      // Hundreds digit in position 2
      if (ten_thousands > 0 || thousands > 0 || hundreds > 0) {
        snprintf(digit_buffers_ft[2], 2, "%d", hundreds);
        lv_label_set_text(altitude_char_labels[2], digit_buffers_ft[2]);
      }

      // Tens digit in position 3
      if (feet >= 10) {
        snprintf(digit_buffers_ft[3], 2, "%d", tens);
        lv_label_set_text(altitude_char_labels[3], digit_buffers_ft[3]);
      }

      // Ones digit in position 4 (always show) - now has normal width
      snprintf(digit_buffers_ft[4], 2, "%d", ones);
      lv_label_set_text(altitude_char_labels[4], digit_buffers_ft[4]);

      // Feet unit in position 6 (same as meters, with proper spacing)
      lv_label_set_text(altitude_char_labels[6], "f");
    }
  }

  // Update temperature labels
  // -- Battery Temperature --
  if (batt_temp_label != NULL && batt_letter_label != NULL) {  // Check labels exist
    lv_color_t bg_color = LVGL_BLACK;  // Default, will be overridden if opaque
    lv_color_t text_color = darkMode ? LVGL_WHITE : LVGL_BLACK;
    lv_opa_t bg_opacity = LV_OPA_0;  // Default transparent

    if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
      lv_label_set_text_fmt(batt_temp_label, "%d", static_cast<int>(batteryTemp));

      if (batteryTemp >= BATT_TEMP_CRITICAL) {
        bg_color = LVGL_RED;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      } else if (batteryTemp >= BATT_TEMP_WARNING) {
        bg_color = LVGL_YELLOW;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      }
    } else {
      lv_label_set_text(batt_temp_label, "-");
    }

    lv_obj_set_style_bg_opa(batt_temp_label, LV_OPA_0, 0);  // Keep label background transparent
    lv_obj_set_style_bg_opa(batt_letter_label, LV_OPA_0, 0);  // Keep letter background transparent
    lv_obj_set_style_text_color(batt_temp_label, text_color, 0);
    lv_obj_set_style_text_color(batt_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);  // Keep letter normal color

    // Apply highlighting to the full background rectangle
    if (batt_temp_bg != NULL) {
      if (bg_opacity > 0) {
        // Show background and set color when highlighting is needed
        lv_obj_clear_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(batt_temp_bg, bg_opacity, LV_PART_MAIN);
        lv_obj_set_style_bg_color(batt_temp_bg, bg_color, LV_PART_MAIN);
      } else {
        // Hide background when no highlighting is needed
        lv_obj_add_flag(batt_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
    }
  }

  // -- ESC Temperature --
  if (esc_temp_label != NULL && esc_letter_label != NULL) {  // Check labels exist
    lv_color_t bg_color = LVGL_BLACK;
    lv_color_t text_color = darkMode ? LVGL_WHITE : LVGL_BLACK;
    lv_opa_t bg_opacity = LV_OPA_0;

    if (escTelemetry.escState == TelemetryState::CONNECTED) {
      lv_label_set_text_fmt(esc_temp_label, "%d", static_cast<int>(escTemp));

      if (escTemp >= ESC_TEMP_CRITICAL) {
        bg_color = LVGL_RED;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      } else if (escTemp >= ESC_TEMP_WARNING) {
        bg_color = LVGL_YELLOW;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      }
    } else {
      lv_label_set_text(esc_temp_label, "-");
    }

    lv_obj_set_style_bg_opa(esc_temp_label, LV_OPA_0, 0);  // Keep label background transparent
    lv_obj_set_style_bg_opa(esc_letter_label, LV_OPA_0, 0);  // Keep letter background transparent
    lv_obj_set_style_text_color(esc_temp_label, text_color, 0);
    lv_obj_set_style_text_color(esc_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);  // Keep letter normal color

    // Apply highlighting to the full background rectangle
    if (esc_temp_bg != NULL) {
      if (bg_opacity > 0) {
        // Show background and set color when highlighting is needed
        lv_obj_clear_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(esc_temp_bg, bg_opacity, LV_PART_MAIN);
        lv_obj_set_style_bg_color(esc_temp_bg, bg_color, LV_PART_MAIN);
      } else {
        // Hide background when no highlighting is needed
        lv_obj_add_flag(esc_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
    }
  }

  // -- Motor Temperature --
  if (motor_temp_label != NULL && motor_letter_label != NULL) {  // Check labels exist
    lv_color_t bg_color = LVGL_BLACK;
    lv_color_t text_color = darkMode ? LVGL_WHITE : LVGL_BLACK;
    lv_opa_t bg_opacity = LV_OPA_0;

    if (escTelemetry.escState == TelemetryState::CONNECTED && motorTemp > -20.0f) {
      lv_label_set_text_fmt(motor_temp_label, "%d", static_cast<int>(motorTemp));

      if (motorTemp >= MOTOR_TEMP_CRITICAL) {
        bg_color = LVGL_RED;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      } else if (motorTemp >= MOTOR_TEMP_WARNING) {
        bg_color = LVGL_YELLOW;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      }
    } else {
      lv_label_set_text(motor_temp_label, "-");
    }

    lv_obj_set_style_bg_opa(motor_temp_label, LV_OPA_0, 0);  // Keep label background transparent
    lv_obj_set_style_bg_opa(motor_letter_label, LV_OPA_0, 0);  // Keep letter background transparent
    lv_obj_set_style_text_color(motor_temp_label, text_color, 0);
    lv_obj_set_style_text_color(motor_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);  // Keep letter normal color

    // Apply highlighting to the full background rectangle
    if (motor_temp_bg != NULL) {
      if (bg_opacity > 0) {
        // Show background and set color when highlighting is needed
        lv_obj_clear_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(motor_temp_bg, bg_opacity, LV_PART_MAIN);
        lv_obj_set_style_bg_color(motor_temp_bg, bg_color, LV_PART_MAIN);
      } else {
        // Hide background when no highlighting is needed
        lv_obj_add_flag(motor_temp_bg, LV_OBJ_FLAG_HIDDEN);
      }
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

  // Update Charging Icon Visibility
  if (charging_icon_img != NULL) {  // Check object exists
    if (bmsTelemetry.is_charging) {  // Check the charging flag
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

  if (currentTime - lastAltitudeTime > 500) {  // Update climb rate every 500ms
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

  // Deselect display CS when done drawing
  digitalWrite(displayCS, HIGH);
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
    testArmedStartMillis
  );
}
