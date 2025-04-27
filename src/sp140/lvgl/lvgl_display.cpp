#include "../../../inc/sp140/lvgl/lvgl_display.h"
#include "../../../inc/version.h"
#include "../../../inc/sp140/structs.h"
#include "../../../inc/sp140/bms.h"
#include <Adafruit_ST7735.h>
#include <vector>
#include <string>
#include <map>
#include <utility> // For std::pair

// Include the generated C file for the cruise icon
#include "../../assets/img/cruise-control-340255-30.c" // New 30x30 icon

// Display dimensions
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

// LVGL buffer size - optimize for our display
// Use 1/4 of the screen size to balance memory usage and performance
#define LVGL_BUFFER_SIZE (SCREEN_WIDTH * (SCREEN_HEIGHT / 4))

// LVGL refresh time in ms - match the config file setting
#define LVGL_REFRESH_TIME 40

// Constants from the original display.h
#define TEMP_WARNING_THRESHOLD 55.0f
#define TEMP_CRITICAL_THRESHOLD 70.0f
#define BATTERY_LOW_THRESHOLD 5.0f // Note: These percentage thresholds seem unused currently
#define BATTERY_MEDIUM_THRESHOLD 20.0f
#define CELL_VOLTAGE_WARNING 3.6f
#define CELL_VOLTAGE_CRITICAL 3.5f

// Notification System Definitions
#define NOTIFICATION_LATCH_MS 2000 // Minimum time to display a notification
#define NOTIFICATION_CYCLE_MS 2000 // Time between cycling to the *next* notification

struct NotificationInfo {
  const char* message;
  lv_color_t color;
  bool is_error; // true for error (red), false for warning (orange)
  uint32_t last_active_time; // Last time the condition for this notification was met
  // Removed display_start_time as it's managed separately now
};

// Define notification message strings
const char* const WARN_LOW_CELL = "BAT-LC";
const char* const CRIT_LOW_CELL = "BAT-LC";
const char* const WARN_LOW_PACK = "BAT-LV";
const char* const CRIT_LOW_PACK = "BAT-LV";
const char* const WARN_HIGH_PACK = "BAT-HV";
const char* const CRIT_HIGH_PACK = "BAT-HV";
const char* const WARN_CELL_DIFF = "BAT-DV";
const char* const CRIT_CELL_DIFF = "BAT-DV";
const char* const WARN_BATT_TEMP = "BAT-HT";
const char* const CRIT_BATT_TEMP = "BAT-HT";
const char* const WARN_ESC_TEMP = "ESC-HT";
const char* const CRIT_ESC_TEMP = "ESC-HT";
const char* const WARN_MOTOR_TEMP = "MOT-HT";
const char* const CRIT_MOTOR_TEMP = "MOT-HT";
const char* const ERR_BMS_TIMEOUT = "BMS-NC";
const char* const ERR_ESC_TIMEOUT = "ESC-NC";
// Add more message strings as needed...

// Global variables
static lv_disp_drv_t disp_drv;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LVGL_BUFFER_SIZE];
static Adafruit_ST7735* tft_driver = nullptr;

// The last time LVGL was updated
static uint32_t lvgl_last_update = 0;

// Main screen objects
static lv_obj_t* main_screen = NULL;
static lv_obj_t* battery_bar = NULL;
static lv_obj_t* battery_label = NULL;
static lv_obj_t* voltage_left_label = NULL;
static lv_obj_t* voltage_right_label = NULL;
static lv_obj_t* power_label = NULL;
static lv_obj_t* power_bar = NULL;
static lv_obj_t* perf_mode_label = NULL;
static lv_obj_t* armed_time_label = NULL;
static lv_obj_t* altitude_label = NULL;
static lv_obj_t* altitude_unit_label = NULL;
static lv_obj_t* batt_temp_label = NULL;
static lv_obj_t* esc_temp_label = NULL;
static lv_obj_t* motor_temp_label = NULL;
static lv_obj_t* arm_indicator = NULL;
static lv_obj_t* spinner = NULL;       // For the spinning animation
static lv_obj_t* spinner_overlay = NULL; // Overlay for the spinner
static lv_obj_t* warning_label = NULL; // New label for warnings/errors
lv_obj_t* cruise_icon_img = NULL; // Cruise control icon image object

// Notification Counter Objects
static lv_obj_t* error_counter_circle = NULL;
static lv_obj_t* error_counter_label = NULL;
static lv_obj_t* warning_counter_circle = NULL;
static lv_obj_t* warning_counter_label = NULL;

// Create a static array of temperature labels for easy access
static lv_obj_t* temp_labels[3];
static lv_obj_t* temp_letter_labels[3];  // For B, E, M letters

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

struct DisplayedNotification {
  const char* message = nullptr;
  lv_color_t color = LVGL_BLACK; // Default color
  bool visible = false;
};

// Function forward declarations
void setupMainScreen(bool darkMode);
// Forward declaration for the new notification management function
static DisplayedNotification manageDisplayNotifications(
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry
);

// Helper function to manage notifications (keep static, pass map by reference)
static void updateOrAddNotification(std::map<std::string, NotificationInfo>& notifications, const std::string& key, bool isActive, const char* msg, lv_color_t color, bool isError) {
    uint32_t current_time = millis();
    auto it = notifications.find(key);

    if (isActive) {
        if (it != notifications.end()) {
            // Update last active time if it exists
            it->second.last_active_time = current_time;
            // Ensure message/color/error status are up-to-date if they can change
            it->second.message = msg;
            it->second.color = color;
            it->second.is_error = isError;
        } else {
            // Add new notification
            notifications[key] = {msg, color, isError, current_time};
        }
    }
    // If !isActive, we only update the last_active_time if the entry exists.
    // The filtering logic in manageDisplayNotifications handles removal.
    // It's important *not* to remove inactive entries here, as manageDisplayNotifications
    // needs to know the last_active_time to enforce the latch.
    else if (it != notifications.end()) {
       // Condition is not active, but keep the entry for now so the latch can work.
       // We don't update last_active_time here if isActive is false.
    }
}


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
    tft_driver = new Adafruit_ST7735(
      spi,
      displayCS,
      dc_pin,
      rst_pin
    );

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
    LV_FONT_DEFAULT                       // Default font
  );
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

  // Initialize the main screen
  USBSerial.println("Switching from splash to main screen");
  setupMainScreen(deviceData.theme == 1);
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

  // Power bar
  power_bar = lv_bar_create(main_screen);
  lv_obj_set_size(power_bar, 85, 8);
  lv_obj_align(power_bar, LV_ALIGN_LEFT_MID, 5, 8);
  lv_bar_set_range(power_bar, 0, 20);  // 0-20kW
  lv_obj_set_style_bg_color(power_bar,
                           darkMode ? LVGL_BLACK : LVGL_WHITE,
                           LV_PART_MAIN);
  lv_obj_set_style_bg_color(power_bar, LVGL_GREEN, LV_PART_INDICATOR);
  // Remove rounded corners from power bar
  lv_obj_set_style_radius(power_bar, 0, LV_PART_MAIN);  // Background part
  lv_obj_set_style_radius(power_bar, 0, LV_PART_INDICATOR);  // Indicator part

  // Performance mode label
  perf_mode_label = lv_label_create(main_screen);
  lv_obj_align(perf_mode_label, LV_ALIGN_RIGHT_MID, -6, -17);
  lv_obj_set_style_text_font(perf_mode_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(perf_mode_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

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

  // Warning/Error message label
  warning_label = lv_label_create(main_screen);
  // Position above altitude area (which is now on the right)
  lv_obj_set_width(warning_label, 115);  // Limit width
  // Align warning label to the top-left of the bottom section
  lv_obj_align(warning_label, LV_ALIGN_TOP_LEFT, 5, 78);  // Position below HLine2
  lv_obj_set_style_text_font(warning_label, &lv_font_montserrat_12, 0);  // Smaller font for warnings
  lv_label_set_long_mode(warning_label, LV_LABEL_LONG_WRAP); // Allow wrapping
  lv_label_set_text(warning_label, ""); // Initially empty
  lv_obj_add_flag(warning_label, LV_OBJ_FLAG_HIDDEN); // Hide initially

  // Altitude unit label (e.g., "m" or "ft")
  altitude_unit_label = lv_label_create(main_screen);
  // Align unit label just left of the vertical line (x=120) at the bottom
  lv_obj_align(altitude_unit_label, LV_ALIGN_BOTTOM_RIGHT, -(SCREEN_WIDTH - 120 + 5), -5); // x=-45, y=-5
  lv_obj_set_style_text_font(altitude_unit_label, &lv_font_montserrat_16, 0); // Use a smaller font
  lv_obj_set_style_text_color(altitude_unit_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Align altitude value label to the left of the unit label
  lv_obj_align_to(altitude_label, altitude_unit_label, LV_ALIGN_OUT_LEFT_BOTTOM, -1, 0); // Align bottom edges, reduce padding to 1px

  // Create temperature labels - adjust positions to align with divider lines
  batt_temp_label = lv_label_create(main_screen);
  lv_obj_align(batt_temp_label, LV_ALIGN_BOTTOM_RIGHT, -5, -35);  // Between top and middle line
  lv_obj_set_style_text_font(batt_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(batt_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(batt_temp_label, LV_OPA_0, 0); // Initially transparent
  lv_obj_set_style_pad_left(batt_temp_label, 3, 0); // Add padding for when background is visible
  lv_obj_set_style_pad_right(batt_temp_label, 3, 0);
  lv_obj_set_style_pad_top(batt_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(batt_temp_label, 2, 0);
  lv_obj_set_style_text_align(batt_temp_label, LV_TEXT_ALIGN_RIGHT, 0); // Right align temperature values
  temp_labels[0] = batt_temp_label;

  // Create letter label for B
  lv_obj_t* batt_letter_label = lv_label_create(main_screen);
  lv_obj_align(batt_letter_label, LV_ALIGN_BOTTOM_LEFT, 122, -35);  // Just right of vertical line
  lv_obj_set_style_text_font(batt_letter_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(batt_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_pad_left(batt_letter_label, 3, 0); // Add padding for when background is visible
  lv_obj_set_style_pad_right(batt_letter_label, 3, 0);
  lv_obj_set_style_pad_top(batt_letter_label, 2, 0);
  lv_obj_set_style_pad_bottom(batt_letter_label, 2, 0);
  lv_label_set_text(batt_letter_label, "B");
  temp_letter_labels[0] = batt_letter_label;

  esc_temp_label = lv_label_create(main_screen);
  lv_obj_align(esc_temp_label, LV_ALIGN_BOTTOM_RIGHT, -5, -16);  // Adjusted to match E label
  lv_obj_set_style_text_font(esc_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(esc_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(esc_temp_label, LV_OPA_0, 0);  // Initially transparent
  lv_obj_set_style_pad_left(esc_temp_label, 3, 0);  // Add padding for when background is visible
  lv_obj_set_style_pad_right(esc_temp_label, 3, 0);
  lv_obj_set_style_pad_top(esc_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(esc_temp_label, 2, 0);
  lv_obj_set_style_text_align(esc_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values
  temp_labels[1] = esc_temp_label;

  // Create letter label for E
  lv_obj_t* esc_letter_label = lv_label_create(main_screen);
  lv_obj_align(esc_letter_label, LV_ALIGN_BOTTOM_LEFT, 122, -16);  // Just right of vertical line
  lv_obj_set_style_text_font(esc_letter_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(esc_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_pad_left(esc_letter_label, 3, 0); // Add padding for when background is visible
  lv_obj_set_style_pad_right(esc_letter_label, 3, 0);
  lv_obj_set_style_pad_top(esc_letter_label, 2, 0);
  lv_obj_set_style_pad_bottom(esc_letter_label, 2, 0);
  lv_label_set_text(esc_letter_label, "E");
  temp_letter_labels[1] = esc_letter_label;

  motor_temp_label = lv_label_create(main_screen);
  lv_obj_align(motor_temp_label, LV_ALIGN_BOTTOM_RIGHT, -5, 0);  // Adjusted to match M label
  lv_obj_set_style_text_font(motor_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(motor_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(motor_temp_label, LV_OPA_0, 0); // Initially transparent
  lv_obj_set_style_pad_left(motor_temp_label, 3, 0);  // Add padding for when background is visible
  lv_obj_set_style_pad_right(motor_temp_label, 3, 0);
  lv_obj_set_style_pad_top(motor_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(motor_temp_label, 2, 0);
  lv_obj_set_style_text_align(motor_temp_label, LV_TEXT_ALIGN_RIGHT, 0);  // Right align temperature values
  temp_labels[2] = motor_temp_label;

  // Create letter label for M
  lv_obj_t* motor_letter_label = lv_label_create(main_screen);
  lv_obj_align(motor_letter_label, LV_ALIGN_BOTTOM_LEFT, 122, 0);
  lv_obj_set_style_text_font(motor_letter_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(motor_letter_label, darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_pad_left(motor_letter_label, 3, 0);  // Add padding for when background is visible
  lv_obj_set_style_pad_right(motor_letter_label, 3, 0);
  lv_obj_set_style_pad_top(motor_letter_label, 2, 0);
  lv_obj_set_style_pad_bottom(motor_letter_label, 2, 0);
  lv_label_set_text(motor_letter_label, "M");
  temp_letter_labels[2] = motor_letter_label;

  // --- Notification Counters ---
  const int counter_size = 16; // Made smaller

  // Error Counter (Red Circle)
  error_counter_circle = lv_obj_create(main_screen);
  lv_obj_set_size(error_counter_circle, counter_size, counter_size);
  lv_obj_set_style_radius(error_counter_circle, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(error_counter_circle, LVGL_RED, 0);
  lv_obj_set_style_border_width(error_counter_circle, 0, 0);

  error_counter_label = lv_label_create(error_counter_circle);
  lv_label_set_text(error_counter_label, "0");
  lv_obj_set_style_text_color(error_counter_label, LVGL_WHITE, 0);
  lv_obj_set_style_text_font(error_counter_label, &lv_font_montserrat_12, 0);
  lv_obj_center(error_counter_label);

  // Warning Counter (Orange Circle)
  warning_counter_circle = lv_obj_create(main_screen);
  lv_obj_set_size(warning_counter_circle, counter_size, counter_size);
  // Align warning counter to bottom right corner, offset slightly
  lv_obj_align(warning_counter_circle, LV_ALIGN_BOTTOM_RIGHT, -25, -35); // Adjusted: left 20px, up 10px
  lv_obj_set_style_radius(warning_counter_circle, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(warning_counter_circle, LVGL_ORANGE, 0);
  lv_obj_set_style_border_width(warning_counter_circle, 0, 0);

  warning_counter_label = lv_label_create(warning_counter_circle);
  lv_label_set_text(warning_counter_label, "0");
  lv_obj_set_style_text_color(warning_counter_label, LVGL_BLACK, 0); // Black text for orange
  lv_obj_set_style_text_font(warning_counter_label, &lv_font_montserrat_12, 0);
  lv_obj_center(warning_counter_label);

  // Align error counter to the left of the warning counter
  lv_obj_align_to(error_counter_circle, warning_counter_circle, LV_ALIGN_OUT_LEFT_MID, -3, 0);

  // Hide counters initially
  lv_obj_add_flag(error_counter_circle, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(warning_counter_circle, LV_OBJ_FLAG_HIDDEN);
  // --- End Notification Counters ---

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
  static lv_point_t v_line1_points[] = {{110, 37}, {110, 70}};
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
  lv_obj_t* h_line3 = lv_line_create(main_screen);
  static lv_point_t h_line3_points[] = {{120, 94}, {SCREEN_WIDTH, 94}};
  lv_line_set_points(h_line3, h_line3_points, 2);
  lv_obj_set_style_line_color(h_line3,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line3, 1, LV_PART_MAIN);

  lv_obj_t* h_line4 = lv_line_create(main_screen);
  static lv_point_t h_line4_points[] = {{120, 111}, {SCREEN_WIDTH, 111}};
  lv_line_set_points(h_line4, h_line4_points, 2);
  lv_obj_set_style_line_color(h_line4,
                             LVGL_GRAY,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line4, 1, LV_PART_MAIN);

  // Ensure warning_label is drawn on top of lines and other elements in its area
  if (warning_label != NULL) {
      lv_obj_move_foreground(warning_label);
  }

  // Create arm indicator (initially hidden)
  arm_indicator = lv_obj_create(main_screen);
  lv_obj_set_size(arm_indicator, 50, 33);
  lv_obj_set_pos(arm_indicator, 110, 37);
  lv_obj_set_style_border_width(arm_indicator, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(arm_indicator, 0, LV_PART_MAIN); // Ensure sharp corners
  // Move to background and hide initially
  lv_obj_move_background(arm_indicator);
  lv_obj_add_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);

  // Create cruise control icon (initially hidden)
  cruise_icon_img = lv_img_create(main_screen);
  // lv_img_set_src(cruise_icon_img, &noun_cruise_control_340255_1); // Use the old 40x40 image descriptor
  lv_img_set_src(cruise_icon_img, &cruise_control_340255_30); // Use the new 30x30 image descriptor

  // Set icon color using recoloring based on theme
  lv_color_t icon_color;
  if (darkMode) {
    icon_color = lv_color_white(); // White icon on dark background
  } else {
    icon_color = lv_color_black(); // Black icon on light background
  }
  lv_obj_set_style_img_recolor(cruise_icon_img, icon_color, LV_PART_MAIN);
  lv_obj_set_style_img_recolor_opa(cruise_icon_img, LV_OPA_COVER, LV_PART_MAIN); // Make icon fully opaque

  // lv_obj_align(cruise_icon_img, LV_ALIGN_CENTER, 12, -12); // Align center, then offset right 12, up 12
  lv_obj_align(cruise_icon_img, LV_ALIGN_CENTER, 12, -9); // Align center, offset right 12, up 9 (down 3 from -12)
  lv_obj_move_foreground(cruise_icon_img); // Ensure icon is on the top layer
  // Don't hide it initially for debugging
  lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN); // Hide initially

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

// Refactored function to handle notification logic with proper latching
static DisplayedNotification manageDisplayNotifications(
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry
) {
  // --- Notification System Static Variables ---
  static std::map<std::string, NotificationInfo> notification_states; // Holds all potential notifications and their last active time
  static std::string currently_displayed_key = "";                     // Key of the notification currently shown
  static uint32_t currently_displayed_start_time = 0;                 // When the current notification started displaying
  static uint32_t last_notification_switch_time = 0;                  // When we last *switched* to a new notification (for cycle timing)
  static int current_error_index = -1;                                // Index for cycling through active errors
  static int current_warning_index = -1;                              // Index for cycling through active warnings
  static bool was_displaying_error_last_cycle = false;                // Track category (error/warning) for index reset logic

  uint32_t current_time = millis();
  DisplayedNotification result; // Struct to return
  result.visible = false;       // Default to not visible

  // --- 1. Update Notification States ---
  // Check all conditions and update/add entries in notification_states
  bool bmsConnected = (bmsTelemetry.bmsState == TelemetryState::CONNECTED);
  bool escConnected = (escTelemetry.escState == TelemetryState::CONNECTED);

  bool low_cell_warn = bmsConnected && (bmsTelemetry.lowest_cell_voltage > 0 && bmsTelemetry.lowest_cell_voltage <= CELL_VOLTAGE_WARNING);
  bool low_cell_crit = bmsConnected && (bmsTelemetry.lowest_cell_voltage > 0 && bmsTelemetry.lowest_cell_voltage <= CELL_VOLTAGE_CRITICAL);
  updateOrAddNotification(notification_states, "LOW_CELL_CRIT", low_cell_crit, CRIT_LOW_CELL, LVGL_RED, true);
  updateOrAddNotification(notification_states, "LOW_CELL_WARN", low_cell_warn && !low_cell_crit, WARN_LOW_CELL, LVGL_ORANGE, false);

  bool batt_temp_warn = bmsConnected && (bmsTelemetry.highest_temperature >= TEMP_WARNING_THRESHOLD);
  bool batt_temp_crit = bmsConnected && (bmsTelemetry.highest_temperature >= TEMP_CRITICAL_THRESHOLD);
  updateOrAddNotification(notification_states, "BATT_TEMP_CRIT", batt_temp_crit, CRIT_BATT_TEMP, LVGL_RED, true);
  updateOrAddNotification(notification_states, "BATT_TEMP_WARN", batt_temp_warn && !batt_temp_crit, WARN_BATT_TEMP, LVGL_ORANGE, false);

  bool esc_temp_warn = escConnected && (escTelemetry.mos_temp >= TEMP_WARNING_THRESHOLD);
  bool esc_temp_crit = escConnected && (escTelemetry.mos_temp >= TEMP_CRITICAL_THRESHOLD);
  updateOrAddNotification(notification_states, "ESC_TEMP_CRIT", esc_temp_crit, CRIT_ESC_TEMP, LVGL_RED, true);
  updateOrAddNotification(notification_states, "ESC_TEMP_WARN", esc_temp_warn && !esc_temp_crit, WARN_ESC_TEMP, LVGL_ORANGE, false);

  bool motor_temp_warn = escConnected && (escTelemetry.motor_temp >= TEMP_WARNING_THRESHOLD);
  bool motor_temp_crit = escConnected && (escTelemetry.motor_temp >= TEMP_CRITICAL_THRESHOLD);
  updateOrAddNotification(notification_states, "MOTOR_TEMP_CRIT", motor_temp_crit, CRIT_MOTOR_TEMP, LVGL_RED, true);
  updateOrAddNotification(notification_states, "MOTOR_TEMP_WARN", motor_temp_warn && !motor_temp_crit, WARN_MOTOR_TEMP, LVGL_ORANGE, false);

  // Use STALE state as well for timeout errors
  bool bms_error = (bmsTelemetry.bmsState == TelemetryState::NOT_CONNECTED || bmsTelemetry.bmsState == TelemetryState::STALE);
  updateOrAddNotification(notification_states, "BMS_TIMEOUT", bms_error, ERR_BMS_TIMEOUT, LVGL_RED, true);

  bool esc_error = (escTelemetry.escState == TelemetryState::NOT_CONNECTED || escTelemetry.escState == TelemetryState::STALE);
  updateOrAddNotification(notification_states, "ESC_TIMEOUT", esc_error, ERR_ESC_TIMEOUT, LVGL_RED, true);

  // --- 2. Filter *Currently Active* Notifications ---
  // Create lists of notifications whose conditions are true *right now*
  std::vector<std::string> currently_true_error_keys;
  std::vector<std::string> currently_true_warning_keys;
  bool current_displayed_condition_is_true = false;

  for (std::map<std::string, NotificationInfo>::const_iterator it = notification_states.begin(); it != notification_states.end(); ++it) {
    const std::string& key = it->first;
    const NotificationInfo& info = it->second;
    // Check if condition was met *on this specific cycle*
    // We rely on updateOrAddNotification having just updated last_active_time if the condition is true
    bool condition_currently_true = (current_time - info.last_active_time < 50); // Use a small threshold to check if updated recently

    if(condition_currently_true) {
        if (info.is_error) {
            currently_true_error_keys.push_back(key);
        } else {
            currently_true_warning_keys.push_back(key);
        }
        // Check if the one *currently* on screen is true right now
        if (key == currently_displayed_key) {
            current_displayed_condition_is_true = true;
        }
    }
  }

  int current_error_count = currently_true_error_keys.size();
  int current_warning_count = currently_true_warning_keys.size();

  // --- 3. Update Counter Display (Based on currently true conditions) ---
  if (error_counter_label != NULL && error_counter_circle != NULL) {
    if (current_error_count > 0) {
      lv_label_set_text_fmt(error_counter_label, "%d", current_error_count);
      lv_obj_clear_flag(error_counter_circle, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(error_counter_circle, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (warning_counter_label != NULL && warning_counter_circle != NULL) {
    if (current_warning_count > 0) {
      lv_label_set_text_fmt(warning_counter_label, "%d", current_warning_count);
      lv_obj_clear_flag(warning_counter_circle, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(warning_counter_circle, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // --- 4. Selection Logic --- Refined ---
  std::string key_to_display_this_cycle = "";
  uint32_t current_display_duration = current_time - currently_displayed_start_time;
  bool current_latch_met = current_display_duration >= NOTIFICATION_LATCH_MS;

  bool currently_displaying = !currently_displayed_key.empty() && notification_states.count(currently_displayed_key);

  // Decision: Keep current? Switch? Or select new?
  if (currently_displaying && (current_displayed_condition_is_true || !current_latch_met)) {
    // KEEP CURRENT: Either condition is still true, OR latch time isn't met yet.
    key_to_display_this_cycle = currently_displayed_key;
    USBSerial.println("[manageDisplay] Decision: Keep current notification.");
  } else {
    // SWITCH or SELECT NEW: Current is either done (condition false + latch met) or wasn't displaying.
    if (currently_displaying) {
        USBSerial.println("[manageDisplay] Decision: Current notification finished, selecting next.");
        currently_displayed_key = ""; // Clear current since it's finished
    } else {
        USBSerial.println("[manageDisplay] Decision: Nothing displayed, selecting new.");
    }

    // Prioritize currently active errors
    if (current_error_count > 0) {
      bool switched_category = !was_displaying_error_last_cycle;
      if (switched_category || current_error_index < 0 || current_error_index >= current_error_count) {
          current_error_index = 0;
      } else {
          // Cycle to next error if not switching category
          current_error_index = (current_error_index + 1) % current_error_count;
      }
      key_to_display_this_cycle = currently_true_error_keys[current_error_index];
      was_displaying_error_last_cycle = true;
      current_warning_index = -1; // Reset warning index
      USBSerial.print("[manageDisplay] Selected Error: "); USBSerial.println(key_to_display_this_cycle.c_str());
    }
    // Else, prioritize currently active warnings
    else if (current_warning_count > 0) {
      bool switched_category = was_displaying_error_last_cycle;
       if (switched_category || current_warning_index < 0 || current_warning_index >= current_warning_count) {
          current_warning_index = 0;
      } else {
          // Cycle to next warning if not switching category
          current_warning_index = (current_warning_index + 1) % current_warning_count;
      }
      key_to_display_this_cycle = currently_true_warning_keys[current_warning_index];
      was_displaying_error_last_cycle = false;
      current_error_index = -1; // Reset error index
      USBSerial.print("[manageDisplay] Selected Warning: "); USBSerial.println(key_to_display_this_cycle.c_str());
    } else {
      // No active errors or warnings
      key_to_display_this_cycle = "";
      was_displaying_error_last_cycle = false;
      current_error_index = -1;
      current_warning_index = -1;
       USBSerial.println("[manageDisplay] Selected: None");
    }

    // If we selected a *new* notification, update the state
    if (!key_to_display_this_cycle.empty()) {
        currently_displayed_key = key_to_display_this_cycle;
        currently_displayed_start_time = current_time;
        last_notification_switch_time = current_time; // Record switch time for potential future cycling logic
    }
  }


  // --- 5. Set Result ---
  if (!key_to_display_this_cycle.empty() && notification_states.count(key_to_display_this_cycle)) {
      const NotificationInfo& info_to_display = notification_states.at(key_to_display_this_cycle);
      result.message = info_to_display.message;
      result.color = info_to_display.color;
      result.visible = true;
  } else {
      // Nothing to display this cycle
      result.visible = false;
      // Ensure the state reflects nothing is displayed if key_to_display is empty
      if (key_to_display_this_cycle.empty()) {
         currently_displayed_key = "";
      }
  }

  // This step is not strictly necessary for correctness but prevents the map from growing indefinitely
  // Remove entries whose last_active_time is very old (e.g., > 1 minute)
  const uint32_t cleanup_threshold = 60000; // 1 minute
  for (auto it = notification_states.begin(); it != notification_states.end(); /* manual increment */) {
    if (it->first != currently_displayed_key && (current_time - it->second.last_active_time) > cleanup_threshold) {
      it = notification_states.erase(it);
    } else {
      ++it;
    }
  }

  // DEBUG: Print final result before returning
  USBSerial.print("[manageDisplay] Visible: "); USBSerial.print(result.visible);
  USBSerial.print(", Message: "); USBSerial.println(result.message ? result.message : "NULL");

  return result;
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
  float escTemp = escTelemetry.mos_temp;
  float motorTemp = escTelemetry.motor_temp;
  // Check if BMS or ESC is connected
  bool bmsConnected = (bmsTelemetry.bmsState == TelemetryState::CONNECTED);
  bool escConnected = (escTelemetry.escState == TelemetryState::CONNECTED);

  // Check if the theme has changed or if the main screen needs to be created
  static int last_theme = -1;
  if (main_screen == NULL || last_theme != deviceData.theme) {
    setupMainScreen(darkMode);
    last_theme = deviceData.theme;
  } else if (last_theme == deviceData.theme) {
    // Just update spinner colors if theme didn't change but screen exists
    lv_obj_set_style_arc_color(spinner, darkMode ? lv_color_make(100, 100, 100) : lv_color_make(230, 230, 230), LV_PART_MAIN);
    lv_obj_set_style_arc_color(spinner, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
  }

  // Update battery bar and percentage
  if (bmsConnected && batteryPercent > 0) {
    lv_bar_set_value(battery_bar, (int)batteryPercent, LV_ANIM_OFF);

    // Set color based on percentage
    lv_color_t batteryColor = LVGL_RED;
    if (batteryPercent >= BATTERY_MEDIUM_THRESHOLD) {
      batteryColor = LVGL_GREEN;
    } else if (batteryPercent >= BATTERY_LOW_THRESHOLD) {
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
  if (voltage_left_label != NULL) { // Check object exists
    if (bmsConnected) {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%2.2fv", lowestCellV);
        lv_label_set_text(voltage_left_label, buffer);

        // Set color based on voltage
        if (lowestCellV <= CELL_VOLTAGE_CRITICAL && lowestCellV > 0) { // Add > 0 check
        lv_obj_set_style_text_color(voltage_left_label, LVGL_RED, 0);
        } else if (lowestCellV <= CELL_VOLTAGE_WARNING && lowestCellV > 0) { // Add > 0 check
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
  if (voltage_right_label != NULL && battery_label != NULL) { // Check objects exist
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
  if (power_label != NULL && power_bar != NULL) { // Check objects exist
    if (bmsConnected || escConnected) {
        char buffer[10];
        float kWatts = unifiedBatteryData.power;
        snprintf(buffer, sizeof(buffer), kWatts < 1.0 ? "%.2f kW" : "%.1f kW", kWatts);
        lv_label_set_text(power_label, buffer);

        // Update power bar (range 0-2000 representing 0-20kW)
        lv_bar_set_value(power_bar, (int)(kWatts * 100), LV_ANIM_OFF);
    } else {
        lv_label_set_text(power_label, "");
        lv_bar_set_value(power_bar, 0, LV_ANIM_OFF);
    }
  }

  // Update performance mode - Re-added this section
  if (perf_mode_label != NULL) { // Check object exists
     lv_label_set_text(perf_mode_label, deviceData.performance_mode == 0 ? "CHILL" : "SPORT");
  }

  // Update armed time
  if (armed_time_label != NULL) { // Check object exists
    const unsigned int nowMillis = millis();
    static unsigned int _lastArmedMillis = 0; // Renamed to avoid conflict
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
    lv_label_set_text(altitude_unit_label, ""); // Clear unit on error
    lv_obj_set_style_text_color(altitude_label, LVGL_RED, 0);
    lv_obj_set_style_text_color(altitude_unit_label, LVGL_RED, 0); // Match color
  } else {
    if (deviceData.metric_alt) {
      snprintf(altBuffer, sizeof(altBuffer), "%.1f", altitude); // Format number only
      lv_label_set_text(altitude_label, altBuffer);
      lv_label_set_text(altitude_unit_label, "m"); // Set unit separately
    } else {
      snprintf(altBuffer, sizeof(altBuffer), "%d", static_cast<int>(round(altitude * 3.28084))); // Format number only
      lv_label_set_text(altitude_label, altBuffer);
      lv_label_set_text(altitude_unit_label, "f"); // Set unit separately (Changed from "ft")
    }
    // Set color for both labels
    lv_obj_set_style_text_color(altitude_label,
                              darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
    lv_obj_set_style_text_color(altitude_unit_label,
                              darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  }

  // --- Get Current Notification ---
  DisplayedNotification current_notification = manageDisplayNotifications(
      escTelemetry, bmsTelemetry
  );

  // --- Update Notification Display ---
  if (warning_label != NULL) { // Check if label exists
    // DEBUG: Print notification state received by updateLvglMainScreen
    USBSerial.print("[updateLvgl] Received Visible: "); USBSerial.print(current_notification.visible);
    USBSerial.print(", Message: "); USBSerial.println(current_notification.message ? current_notification.message : "NULL");

    if (current_notification.visible) {
      // Ensure text and color are set correctly
      lv_label_set_text(warning_label, current_notification.message);
      // DEBUG: Confirm text set
      USBSerial.print("[updateLvgl] Set warning_label text to: "); USBSerial.println(current_notification.message);
      lv_obj_set_style_text_color(warning_label, current_notification.color, 0);
      // Ensure the label is made visible
      lv_obj_clear_flag(warning_label, LV_OBJ_FLAG_HIDDEN);
    } else {
      // Ensure the label is hidden when no notification is active
      lv_obj_add_flag(warning_label, LV_OBJ_FLAG_HIDDEN);
    }
  }
  // --- End Notification Update ---

  // Update temperature labels
  struct {
    float temp;
    TelemetryState state;
    const char* label;
    lv_obj_t* value_label;
    lv_obj_t* char_label;
  } temps_to_update[] = {
    {batteryTemp, bmsTelemetry.bmsState, "B", temp_labels[0], temp_letter_labels[0]},
    {escTemp, escTelemetry.escState, "E", temp_labels[1], temp_letter_labels[1]},
    {motorTemp, escTelemetry.escState, "M", temp_labels[2], temp_letter_labels[2]}
  };

  for (int i = 0; i < 3; i++) {
     // Check labels exist using correct member names
     if (temps_to_update[i].value_label == NULL || temps_to_update[i].char_label == NULL) continue;

    // Set the temperature value or dash
    if (temps_to_update[i].state == TelemetryState::CONNECTED) {
      // Use value_label for the numeric temperature display
      lv_label_set_text_fmt(temps_to_update[i].value_label, "%d", static_cast<int>(temps_to_update[i].temp));
    } else {
      // Use value_label for the dash display
      lv_label_set_text(temps_to_update[i].value_label, "-");
    }

    // Set colors based on temperature
    lv_color_t bg_color;
    lv_color_t text_color;
    lv_opa_t bg_opacity = LV_OPA_0;  // Default transparent

    if (temps_to_update[i].state == TelemetryState::CONNECTED) {
      if (temps_to_update[i].temp >= TEMP_CRITICAL_THRESHOLD) {
        bg_color = LVGL_RED;
        text_color = LVGL_WHITE;
        bg_opacity = LV_OPA_100;
      } else if (temps_to_update[i].temp >= TEMP_WARNING_THRESHOLD) {
        bg_color = LVGL_ORANGE;
        text_color = LVGL_BLACK;
        bg_opacity = LV_OPA_100;
      } else {
        text_color = darkMode ? LVGL_WHITE : LVGL_BLACK;
      }
    } else {
      text_color = darkMode ? LVGL_WHITE : LVGL_BLACK;
    }

    // Apply same styles to both labels using correct member names
    lv_obj_set_style_bg_opa(temps_to_update[i].value_label, bg_opacity, 0);
    lv_obj_set_style_bg_opa(temps_to_update[i].char_label, bg_opacity, 0);

    lv_obj_set_style_text_color(temps_to_update[i].value_label, text_color, 0);
    lv_obj_set_style_text_color(temps_to_update[i].char_label, text_color, 0);

    if (bg_opacity > 0) {
      lv_obj_set_style_bg_color(temps_to_update[i].value_label, bg_color, LV_PART_MAIN);
      lv_obj_set_style_bg_color(temps_to_update[i].char_label, bg_color, LV_PART_MAIN);
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
  if (cruising) {
    lv_obj_clear_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
  }

  // Deselect display CS when done drawing
  digitalWrite(displayCS, HIGH);
}
