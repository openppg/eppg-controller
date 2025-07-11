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
static lv_obj_t* power_label = NULL;
static lv_obj_t* power_bar = NULL;
static lv_obj_t* perf_mode_label = NULL;
static lv_obj_t* armed_time_label = NULL;
static lv_obj_t* altitude_label = NULL;
static lv_obj_t* batt_temp_label = NULL;
static lv_obj_t* esc_temp_label = NULL;
static lv_obj_t* motor_temp_label = NULL;
static lv_obj_t* arm_indicator = NULL;
static lv_obj_t* spinner = NULL;       // For the spinning animation
static lv_obj_t* spinner_overlay = NULL;  // Overlay for the spinner
static lv_obj_t* batt_letter_label = NULL;  // Letter label for Battery temp
static lv_obj_t* esc_letter_label = NULL;  // Letter label for ESC temp
static lv_obj_t* motor_letter_label = NULL;  // Letter label for Motor temp
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
  lv_obj_align(battery_label, LV_ALIGN_TOP_MID, 0, 5);
  lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(battery_label, LVGL_BLACK, 0);

  // Left voltage label
  voltage_left_label = lv_label_create(main_screen);
  lv_obj_align(voltage_left_label, LV_ALIGN_TOP_LEFT, 3, 10);
  lv_obj_set_style_text_font(voltage_left_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(voltage_left_label,
                            darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Right voltage label
  voltage_right_label = lv_label_create(main_screen);
  lv_obj_align(voltage_right_label, LV_ALIGN_TOP_RIGHT, -3, 10);
  lv_obj_set_style_text_font(voltage_right_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(voltage_right_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Middle section - power display
  power_label = lv_label_create(main_screen);
  lv_obj_align(power_label, LV_ALIGN_LEFT_MID, 5, -10);
  lv_obj_set_style_text_font(power_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(power_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);


  // Performance mode label
  perf_mode_label = lv_label_create(main_screen);
  lv_obj_align(perf_mode_label, LV_ALIGN_RIGHT_MID, -5, -17);
  lv_obj_set_style_text_font(perf_mode_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(perf_mode_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  // Ensure text within the label is centered
  lv_obj_set_style_text_align(perf_mode_label, LV_TEXT_ALIGN_CENTER, 0);

  // Armed time label - adjust position now that there's no bluetooth icon
  armed_time_label = lv_label_create(main_screen);
  lv_obj_align(armed_time_label, LV_ALIGN_RIGHT_MID, -6, -3);
  lv_obj_set_style_text_font(armed_time_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(armed_time_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Bottom section - altitude and temperatures
  altitude_label = lv_label_create(main_screen);
  // Use a larger font for altitude
  lv_obj_set_style_text_font(altitude_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(altitude_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  // Align altitude label to the bottom-right corner of its section (left of vertical line)
  lv_obj_align(altitude_label, LV_ALIGN_BOTTOM_LEFT, 5, -5);  // Position slightly from bottom-left

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

  // Create temperature labels - adjust positions to align with divider lines
  batt_temp_label = lv_label_create(main_screen);
  // Align bottom-right, adjust Y offset for spacing
  lv_obj_align(batt_temp_label, LV_ALIGN_BOTTOM_RIGHT, -1, -39);
  lv_obj_set_style_text_font(batt_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(batt_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(batt_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_pad_left(batt_temp_label, 3, 0);
  lv_obj_set_style_pad_right(batt_temp_label, 3, 0);
  lv_obj_set_style_pad_top(batt_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(batt_temp_label, 2, 0);
  lv_obj_set_style_text_align(batt_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values

  // Create letter label for B
  batt_letter_label = lv_label_create(main_screen);
  // Align bottom-left, adjust Y offset for spacing
  lv_obj_align(batt_letter_label, LV_ALIGN_BOTTOM_LEFT, 122, -39);
  lv_obj_set_style_text_font(batt_letter_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(batt_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_pad_left(batt_letter_label, 3, 0);
  lv_obj_set_style_pad_right(batt_letter_label, 3, 0);
  lv_obj_set_style_pad_top(batt_letter_label, 2, 0);
  lv_obj_set_style_pad_bottom(batt_letter_label, 2, 0);
  lv_label_set_text(batt_letter_label, "B");

  esc_temp_label = lv_label_create(main_screen);
  // Align bottom-right, adjust Y offset for spacing
  lv_obj_align(esc_temp_label, LV_ALIGN_BOTTOM_RIGHT, -1, -20);
  lv_obj_set_style_text_font(esc_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(esc_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(esc_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_pad_left(esc_temp_label, 3, 0);
  lv_obj_set_style_pad_right(esc_temp_label, 3, 0);
  lv_obj_set_style_pad_top(esc_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(esc_temp_label, 2, 0);
  lv_obj_set_style_text_align(esc_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values

  // Create letter label for E
  esc_letter_label = lv_label_create(main_screen);
  // Align bottom-left, adjust Y offset for spacing
  lv_obj_align(esc_letter_label, LV_ALIGN_BOTTOM_LEFT, 122, -20);
  lv_obj_set_style_text_font(esc_letter_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(esc_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_pad_left(esc_letter_label, 3, 0);
  lv_obj_set_style_pad_right(esc_letter_label, 3, 0);
  lv_obj_set_style_pad_top(esc_letter_label, 2, 0);
  lv_obj_set_style_pad_bottom(esc_letter_label, 2, 0);
  lv_label_set_text(esc_letter_label, "E");

  motor_temp_label = lv_label_create(main_screen);
  // Align bottom-right, adjust Y offset for spacing
  lv_obj_align(motor_temp_label, LV_ALIGN_BOTTOM_RIGHT, -1, 0);
  lv_obj_set_style_text_font(motor_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(motor_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(motor_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_pad_left(motor_temp_label, 3, 0);
  lv_obj_set_style_pad_right(motor_temp_label, 3, 0);
  lv_obj_set_style_pad_top(motor_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(motor_temp_label, 2, 0);
  lv_obj_set_style_text_align(motor_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values

  // Create letter label for M
  motor_letter_label = lv_label_create(main_screen);
  // Align bottom-left, adjust Y offset for spacing
  lv_obj_align(motor_letter_label, LV_ALIGN_BOTTOM_LEFT, 122, 0);
  lv_obj_set_style_text_font(motor_letter_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(motor_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_pad_left(motor_letter_label, 3, 0);
  lv_obj_set_style_pad_right(motor_letter_label, 3, 0);
  lv_obj_set_style_pad_top(motor_letter_label, 2, 0);
  lv_obj_set_style_pad_bottom(motor_letter_label, 2, 0);
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

  // Create horizontal line between middle and bottom sections
  lv_obj_t* h_line2 = lv_line_create(main_screen);
  static lv_point_t h_line2_points[] = {{0, 70}, {SCREEN_WIDTH, 70}};
  lv_line_set_points(h_line2, h_line2_points, 2);
  lv_obj_set_style_line_color(h_line2,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line2, 1, LV_PART_MAIN);

  // Create vertical line in middle section
  lv_obj_t* v_line1 = lv_line_create(main_screen);
  static lv_point_t v_line1_points[] = {{108, 37}, {108, 70}};
  lv_line_set_points(v_line1, v_line1_points, 2);
  lv_obj_set_style_line_color(v_line1,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line1, 1, LV_PART_MAIN);

  // Create vertical line in bottom section
  lv_obj_t* v_line2 = lv_line_create(main_screen);
  static lv_point_t v_line2_points[] = {{120, 70}, {120, 128}};
  lv_line_set_points(v_line2, v_line2_points, 2);
  lv_obj_set_style_line_color(v_line2,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line2, 1, LV_PART_MAIN);

  // Create horizontal dividers for temperature section
  // Line between B and E at Y=89
  lv_obj_t* h_line3 = lv_line_create(main_screen);
  static lv_point_t h_line3_points[] = {{120, 89}, {SCREEN_WIDTH, 89}};
  lv_line_set_points(h_line3, h_line3_points, 2);
  lv_obj_set_style_line_color(h_line3,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line3, 1, LV_PART_MAIN);

  // Line between E and M at Y=109
  lv_obj_t* h_line4 = lv_line_create(main_screen);
  static lv_point_t h_line4_points[] = {{120, 109}, {SCREEN_WIDTH, 109}};
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

  // Create arm indicator (initially hidden)
  arm_indicator = lv_obj_create(main_screen);
  lv_obj_set_size(arm_indicator, 52, 33);
  lv_obj_set_pos(arm_indicator, 108, 37);
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
  lv_obj_align(cruise_icon_img, LV_ALIGN_CENTER, 12, -9);  // Align center, offset right 12, up 9 (down 3 from -12)
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
    // Align relative to altitude label (created earlier)
    if (altitude_label) {
      lv_obj_align_to(warning_counter_circle, altitude_label, LV_ALIGN_OUT_TOP_LEFT, 0, -2);
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

// Public helpers to control alert text externally
void lv_showAlertText(SensorID id, bool critical) {
  if (alert_text_label == NULL) return;
  const char* txt = sensorIDToAbbreviation(id);
  lv_label_set_text(alert_text_label, txt);
  // Use larger font for critical alerts, smaller for warnings
  if (critical) {
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_18, 0);
    // Hide altitude while showing critical alert
    if (altitude_label) {
      lv_obj_add_flag(altitude_label, LV_OBJ_FLAG_HIDDEN);
      // Move alert label to altitude position
      lv_obj_set_pos(alert_text_label, lv_obj_get_x(altitude_label), lv_obj_get_y(altitude_label));
    }
  } else {
    lv_obj_set_style_text_font(alert_text_label, &lv_font_montserrat_14, 0);
    // Ensure altitude visible during warnings
    if (altitude_label) {
      lv_obj_clear_flag(altitude_label, LV_OBJ_FLAG_HIDDEN);
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
  if (altitude_label) lv_obj_clear_flag(altitude_label, LV_OBJ_FLAG_HIDDEN);
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

        // Set color based on voltage
        if (lowestCellV <= CELL_VOLTAGE_CRITICAL && lowestCellV > 0) {  // Add > 0 check
        lv_obj_set_style_text_color(voltage_left_label, LVGL_RED, 0);
        } else if (lowestCellV <= CELL_VOLTAGE_WARNING && lowestCellV > 0) {  // Add > 0 check
        lv_obj_set_style_text_color(voltage_left_label, LVGL_ORANGE, 0);
        } else {
        lv_obj_set_style_text_color(voltage_left_label,
                                    darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
        }
    } else if (escConnected) {
        lv_obj_set_style_text_color(voltage_left_label, LVGL_RED, 0);
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

  // Update power display
  if (power_label != NULL) {  // Check objects exist
    if (bmsConnected || escConnected) {
        char buffer[10];
        float kWatts = unifiedBatteryData.power;
        snprintf(buffer, sizeof(buffer), kWatts < 1.0 ? "%.2f kW" : "%.1f kW", kWatts);
        lv_label_set_text(power_label, buffer);
    } else {
        lv_label_set_text(power_label, "");
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

  // Update altitude
  char altBuffer[15];
  if (altitude == __FLT_MIN__) {
    lv_label_set_text(altitude_label, "ERR");
    lv_obj_set_style_text_color(altitude_label, LVGL_RED, 0);
  } else {
    if (deviceData.metric_alt) {
      // Explicitly handle small negative values to avoid "-0.0"
      float display_altitude = altitude;
      if (display_altitude > -0.05f && display_altitude < 0.0f) {
          display_altitude = 0.0f;
      }
      snprintf(altBuffer, sizeof(altBuffer), "%.1f m", display_altitude);  // Use corrected value
      lv_label_set_text(altitude_label, altBuffer);
    } else {
      snprintf(altBuffer, sizeof(altBuffer), "%d f", static_cast<int>(round(altitude * 3.28084)));  // Format number with unit
      lv_label_set_text(altitude_label, altBuffer);
    }
    // Set color for the combined label
    lv_obj_set_style_text_color(altitude_label,
                              darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
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
        text_color = LVGL_WHITE;
        bg_opacity = LV_OPA_100;
      } else if (batteryTemp >= BATT_TEMP_WARNING) {
        bg_color = LVGL_ORANGE;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      }
    } else {
      lv_label_set_text(batt_temp_label, "-");
    }

    lv_obj_set_style_bg_opa(batt_temp_label, bg_opacity, 0);
    lv_obj_set_style_bg_opa(batt_letter_label, bg_opacity, 0);
    lv_obj_set_style_text_color(batt_temp_label, text_color, 0);
    lv_obj_set_style_text_color(batt_letter_label, text_color, 0);
    if (bg_opacity > 0) {
      lv_obj_set_style_bg_color(batt_temp_label, bg_color, LV_PART_MAIN);
      lv_obj_set_style_bg_color(batt_letter_label, bg_color, LV_PART_MAIN);
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
        text_color = LVGL_WHITE;
        bg_opacity = LV_OPA_100;
      } else if (escTemp >= ESC_TEMP_WARNING) {
        bg_color = LVGL_ORANGE;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      }
    } else {
      lv_label_set_text(esc_temp_label, "-");
    }

    lv_obj_set_style_bg_opa(esc_temp_label, bg_opacity, 0);
    lv_obj_set_style_bg_opa(esc_letter_label, bg_opacity, 0);
    lv_obj_set_style_text_color(esc_temp_label, text_color, 0);
    lv_obj_set_style_text_color(esc_letter_label, text_color, 0);
    if (bg_opacity > 0) {
      lv_obj_set_style_bg_color(esc_temp_label, bg_color, LV_PART_MAIN);
      lv_obj_set_style_bg_color(esc_letter_label, bg_color, LV_PART_MAIN);
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
        text_color = LVGL_WHITE;
        bg_opacity = LV_OPA_100;
      } else if (motorTemp >= MOTOR_TEMP_WARNING) {
        bg_color = LVGL_ORANGE;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      }
    } else {
      lv_label_set_text(motor_temp_label, "-");
    }

    lv_obj_set_style_bg_opa(motor_temp_label, bg_opacity, 0);
    lv_obj_set_style_bg_opa(motor_letter_label, bg_opacity, 0);
    lv_obj_set_style_text_color(motor_temp_label, text_color, 0);
    lv_obj_set_style_text_color(motor_letter_label, text_color, 0);
    if (bg_opacity > 0) {
      lv_obj_set_style_bg_color(motor_temp_label, bg_color, LV_PART_MAIN);
      lv_obj_set_style_bg_color(motor_letter_label, bg_color, LV_PART_MAIN);
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

  // Deselect display CS when done drawing
  digitalWrite(displayCS, HIGH);
}
