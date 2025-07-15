#ifndef LVGL_CORE_H  // NOLINT(build/header_guard)
#define LVGL_CORE_H  // NOLINT(build/header_guard)

#include <lvgl.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "../structs.h"

// LVGL buffer size - adjust based on available memory
#define LVGL_BUFFER_SIZE 160 * 10  // 10 rows of display width

// Display dimensions
#define SCREEN_WIDTH  160
#define SCREEN_HEIGHT 128

// LVGL refresh time in milliseconds
#define LVGL_REFRESH_TIME 10

// External variables from main sketch
extern int8_t displayCS;

// Global variables
extern lv_display_t* disp;
extern lv_color_t buf[LVGL_BUFFER_SIZE];
extern Adafruit_ST7735* tft_driver;
extern uint32_t lvgl_last_update;

// Function declarations
void setupLvglBuffer();
void setupLvglDisplay(const STR_DEVICE_DATA_140_V1& deviceData, int8_t dc_pin, int8_t rst_pin, SPIClass* spi);
void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
void lv_tick_handler();
void updateLvgl();
void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration);

#endif // LVGL_CORE_H
