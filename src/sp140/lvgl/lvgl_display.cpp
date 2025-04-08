#include "../../../inc/sp140/lvgl/lvgl_display.h"
#include "../../../inc/version.h"
#include "../../../inc/sp140/structs.h"
#include "../../../inc/sp140/bms.h"
#include <Adafruit_ST7735.h>

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
#define BATTERY_LOW_THRESHOLD 30.0f
#define BATTERY_MEDIUM_THRESHOLD 60.0f
#define CELL_VOLTAGE_WARNING 3.6f
#define CELL_VOLTAGE_CRITICAL 3.5f

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
static lv_obj_t* batt_temp_label = NULL;
static lv_obj_t* esc_temp_label = NULL;
static lv_obj_t* motor_temp_label = NULL;
static lv_obj_t* arm_indicator = NULL;
static lv_obj_t* spinner = NULL;       // For the spinning animation

// Create a static array of temperature labels for easy access
static lv_obj_t* temp_labels[3];

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
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(title_label,
                             deviceData.theme == 1 ? lv_color_white() : lv_color_black(),
                             0);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);

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
  lv_obj_set_size(battery_bar, SCREEN_WIDTH, 32);
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
  lv_obj_align(battery_label, LV_ALIGN_TOP_MID, 0, 8);
  lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_16, 0);
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
  lv_obj_align(perf_mode_label, LV_ALIGN_RIGHT_MID, -15, -12);
  lv_obj_set_style_text_font(perf_mode_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(perf_mode_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Armed time label - adjust position now that there's no bluetooth icon
  armed_time_label = lv_label_create(main_screen);
  lv_obj_align(armed_time_label, LV_ALIGN_RIGHT_MID, -15, 5);
  lv_obj_set_style_text_font(armed_time_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(armed_time_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

  // Bottom section - altitude and temperatures
  altitude_label = lv_label_create(main_screen);
  lv_obj_align(altitude_label, LV_ALIGN_BOTTOM_LEFT, 5, -25);
  // Use a larger font for altitude
  lv_obj_set_style_text_font(altitude_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(altitude_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);

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
  temp_labels[0] = batt_temp_label;

  esc_temp_label = lv_label_create(main_screen);
  lv_obj_align(esc_temp_label, LV_ALIGN_BOTTOM_RIGHT, -5, -18);  // Between middle and bottom line
  lv_obj_set_style_text_font(esc_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(esc_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(esc_temp_label, LV_OPA_0, 0); // Initially transparent
  lv_obj_set_style_pad_left(esc_temp_label, 3, 0); // Add padding for when background is visible
  lv_obj_set_style_pad_right(esc_temp_label, 3, 0);
  lv_obj_set_style_pad_top(esc_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(esc_temp_label, 2, 0);
  temp_labels[1] = esc_temp_label;

  motor_temp_label = lv_label_create(main_screen);
  lv_obj_align(motor_temp_label, LV_ALIGN_BOTTOM_RIGHT, -5, -2);  // Below bottom line
  lv_obj_set_style_text_font(motor_temp_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(motor_temp_label,
                             darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  lv_obj_set_style_bg_opa(motor_temp_label, LV_OPA_0, 0); // Initially transparent
  lv_obj_set_style_pad_left(motor_temp_label, 3, 0); // Add padding for when background is visible
  lv_obj_set_style_pad_right(motor_temp_label, 3, 0);
  lv_obj_set_style_pad_top(motor_temp_label, 2, 0);
  lv_obj_set_style_pad_bottom(motor_temp_label, 2, 0);
  temp_labels[2] = motor_temp_label;

  // Draw divider lines
  // Create horizontal line between top and middle sections
  lv_obj_t* h_line1 = lv_line_create(main_screen);
  static lv_point_t h_line1_points[] = {{0, 32}, {SCREEN_WIDTH, 32}};
  lv_line_set_points(h_line1, h_line1_points, 2);
  lv_obj_set_style_line_color(h_line1,
                             darkMode ? LVGL_GRAY : LVGL_BLACK,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line1, 1, LV_PART_MAIN);

  // Create horizontal line between middle and bottom sections
  lv_obj_t* h_line2 = lv_line_create(main_screen);
  static lv_point_t h_line2_points[] = {{0, 75}, {SCREEN_WIDTH, 75}};
  lv_line_set_points(h_line2, h_line2_points, 2);
  lv_obj_set_style_line_color(h_line2,
                             darkMode ? LVGL_GRAY : LVGL_BLACK,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line2, 1, LV_PART_MAIN);

  // Create vertical line in middle section
  lv_obj_t* v_line1 = lv_line_create(main_screen);
  static lv_point_t v_line1_points[] = {{90, 32}, {90, 75}};
  lv_line_set_points(v_line1, v_line1_points, 2);
  lv_obj_set_style_line_color(v_line1,
                             darkMode ? LVGL_GRAY : LVGL_BLACK,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line1, 1, LV_PART_MAIN);

  // Create vertical line in bottom section
  lv_obj_t* v_line2 = lv_line_create(main_screen);
  static lv_point_t v_line2_points[] = {{120, 75}, {120, 128}};
  lv_line_set_points(v_line2, v_line2_points, 2);
  lv_obj_set_style_line_color(v_line2,
                             darkMode ? LVGL_GRAY : LVGL_BLACK,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(v_line2, 1, LV_PART_MAIN);

  // Create horizontal dividers for temperature section
  lv_obj_t* h_line3 = lv_line_create(main_screen);
  static lv_point_t h_line3_points[] = {{120, 94}, {SCREEN_WIDTH, 94}};
  lv_line_set_points(h_line3, h_line3_points, 2);
  lv_obj_set_style_line_color(h_line3,
                             darkMode ? LVGL_GRAY : LVGL_BLACK,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line3, 1, LV_PART_MAIN);

  lv_obj_t* h_line4 = lv_line_create(main_screen);
  static lv_point_t h_line4_points[] = {{120, 111}, {SCREEN_WIDTH, 111}};
  lv_line_set_points(h_line4, h_line4_points, 2);
  lv_obj_set_style_line_color(h_line4,
                             darkMode ? LVGL_GRAY : LVGL_BLACK,
                             LV_PART_MAIN);
  lv_obj_set_style_line_width(h_line4, 1, LV_PART_MAIN);

  // Create arm indicator (initially hidden)
  arm_indicator = lv_obj_create(main_screen);
  lv_obj_set_size(arm_indicator, 70, 43);
  lv_obj_set_pos(arm_indicator, 90, 32);
  lv_obj_set_style_border_width(arm_indicator, 0, LV_PART_MAIN);
  lv_obj_clear_flag(arm_indicator, LV_OBJ_FLAG_CLICKABLE); // Not clickable
  // Move to background and hide initially
  lv_obj_move_background(arm_indicator);
  lv_obj_add_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);

  // Create spinning animation at the top center
  spinner = lv_spinner_create(main_screen, 1000, 60);  // 1000ms period, 60 arcade width
  lv_obj_set_size(spinner, 16, 16);  // Small 16x16 spinner
  lv_obj_align(spinner, LV_ALIGN_TOP_MID, 0, 4);  // Position at top center
  lv_obj_set_style_arc_width(spinner, 2, LV_PART_INDICATOR);  // Thin arc
  lv_obj_set_style_arc_width(spinner, 2, LV_PART_MAIN);  // Thin background arc

  // Set spinner colors
  lv_obj_set_style_arc_color(spinner, darkMode ? LVGL_GRAY : lv_color_make(200, 200, 200), LV_PART_MAIN);
  lv_obj_set_style_arc_color(spinner, LVGL_BLUE, LV_PART_INDICATOR);

  // Load the screen
  lv_scr_load(main_screen);
}

void updateLvglMainScreen(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
) {
  // Make sure display CS is selected
  digitalWrite(displayCS, LOW);

  bool darkMode = (deviceData.theme == 1);
  float batteryPercent = unifiedBatteryData.soc;
  float totalVolts = unifiedBatteryData.volts;
  float lowestCellV = bmsTelemetry.lowest_cell_voltage;
  float batteryTemp = bmsTelemetry.highest_temperature;
  float escTemp = escTelemetry.mos_temp;
  float motorTemp = escTelemetry.motor_temp;
  TelemetryState bmsState = bmsTelemetry.state;

  // Check if the theme has changed or if the main screen needs to be created
  static int last_theme = -1;
  if (main_screen == NULL || last_theme != deviceData.theme) {
    setupMainScreen(darkMode);
    last_theme = deviceData.theme;
  } else if (last_theme == deviceData.theme) {
    // Just update spinner colors if theme didn't change but screen exists
    lv_obj_set_style_arc_color(spinner, darkMode ? LVGL_GRAY : lv_color_make(200, 200, 200), LV_PART_MAIN);
    lv_obj_set_style_arc_color(spinner, LVGL_BLUE, LV_PART_INDICATOR);
  }

  // Update battery bar and percentage
  if (bmsState == TelemetryState::CONNECTED && batteryPercent > 0) {
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
  } else {
    lv_bar_set_value(battery_bar, 0, LV_ANIM_OFF);
    lv_label_set_text(battery_label, "NO DATA");
    lv_obj_set_style_text_color(battery_label, LVGL_RED, 0);
  }

  // Update left voltage (cell voltage)
  if (bmsState == TelemetryState::CONNECTED) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%2.2fv", lowestCellV);
    lv_label_set_text(voltage_left_label, buffer);

    // Set color based on voltage
    if (lowestCellV <= CELL_VOLTAGE_CRITICAL) {
      lv_obj_set_style_text_color(voltage_left_label, LVGL_RED, 0);
    } else if (lowestCellV <= CELL_VOLTAGE_WARNING) {
      lv_obj_set_style_text_color(voltage_left_label, LVGL_ORANGE, 0);
    } else {
      lv_obj_set_style_text_color(voltage_left_label,
                                darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
    }
  } else {
    lv_label_set_text(voltage_left_label, "");
  }

  // Update right voltage (total voltage)
  if (bmsState == TelemetryState::CONNECTED) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%2.0fv", totalVolts);
    lv_label_set_text(voltage_right_label, buffer);
  } else {
    lv_label_set_text(voltage_right_label, "");
  }

  // Update power display
  if (bmsState == TelemetryState::CONNECTED) {
    char buffer[10];
    float kWatts = bmsTelemetry.power;
    snprintf(buffer, sizeof(buffer), kWatts < 10 ? "%.1f kW" : "%.1f kW", kWatts);
    lv_label_set_text(power_label, buffer);

    // Update power bar
    lv_bar_set_value(power_bar, (int)(kWatts * 100), LV_ANIM_OFF);
  } else {
    lv_label_set_text(power_label, "");
    lv_bar_set_value(power_bar, 0, LV_ANIM_OFF);
  }

  // Update performance mode
  lv_label_set_text(perf_mode_label, deviceData.performance_mode == 0 ? "CHILL" : "SPORT");

  // Update armed time
  const unsigned int nowMillis = millis();
  static unsigned int _lastArmedMillis = 0;
  if (armed) _lastArmedMillis = nowMillis;
  const int sessionSeconds = (_lastArmedMillis - armedStartMillis) / 1000.0;
  char timeBuffer[10];
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);
  lv_label_set_text(armed_time_label, timeBuffer);

  // Update altitude
  char altBuffer[15];
  if (altitude == __FLT_MIN__) {
    lv_label_set_text(altitude_label, "ERR");
    lv_obj_set_style_text_color(altitude_label, LVGL_RED, 0);
  } else {
    if (deviceData.metric_alt) {
      snprintf(altBuffer, sizeof(altBuffer), "%.1f m", altitude);
    } else {
      snprintf(altBuffer, sizeof(altBuffer), "%d ft", static_cast<int>(round(altitude * 3.28084)));
    }
    lv_label_set_text(altitude_label, altBuffer);
    lv_obj_set_style_text_color(altitude_label,
                              darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
  }

  // Update temperature labels
  struct {
    float temp;
    TelemetryState state;
    const char* label;
  } temps[] = {
    {batteryTemp, bmsState, "B"},
    {escTemp, escTelemetry.state, "E"},
    {motorTemp, escTelemetry.state, "M"}
  };

  for (int i = 0; i < 3; i++) {
    char tempBuffer[10];

    if (temps[i].state == TelemetryState::CONNECTED) {
      snprintf(tempBuffer, sizeof(tempBuffer), "%s %d", temps[i].label, static_cast<int>(temps[i].temp));
      lv_label_set_text(temp_labels[i], tempBuffer);

      if (temps[i].temp >= TEMP_CRITICAL_THRESHOLD) {
        lv_obj_set_style_bg_color(temp_labels[i], LVGL_RED, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(temp_labels[i], LV_OPA_100, 0); // Full opacity
        lv_obj_set_style_text_color(temp_labels[i], LVGL_WHITE, 0);
      } else if (temps[i].temp >= TEMP_WARNING_THRESHOLD) {
        lv_obj_set_style_bg_color(temp_labels[i], LVGL_ORANGE, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(temp_labels[i], LV_OPA_100, 0); // Full opacity
        lv_obj_set_style_text_color(temp_labels[i], LVGL_BLACK, 0);
      } else {
        lv_obj_set_style_bg_opa(temp_labels[i], LV_OPA_0, 0); // Transparent
        lv_obj_set_style_text_color(temp_labels[i],
                                  darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
      }
    } else {
      snprintf(tempBuffer, sizeof(tempBuffer), "%s -", temps[i].label);
      lv_label_set_text(temp_labels[i], tempBuffer);
      lv_obj_set_style_bg_opa(temp_labels[i], LV_OPA_0, 0); // Transparent
      lv_obj_set_style_text_color(temp_labels[i],
                                darkMode ? LVGL_WHITE : LVGL_BLACK, 0);
    }
  }

  // Update armed indicator
  if (armed) {
    lv_color_t bgColor = cruising ? LVGL_YELLOW : LVGL_CYAN;
    lv_obj_set_style_bg_color(arm_indicator, bgColor, LV_PART_MAIN);
    lv_obj_clear_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(arm_indicator, LV_OBJ_FLAG_HIDDEN);
  }

  // Deselect display CS when done
  digitalWrite(displayCS, HIGH);
}
