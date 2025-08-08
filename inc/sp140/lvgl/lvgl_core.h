#ifndef LVGL_CORE_H
#define LVGL_CORE_H

#include <lvgl.h>
#include <Adafruit_ST7735.h>
#include "../structs.h"
// FreeRTOS for mutex used to guard shared SPI bus
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Display dimensions
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

// LVGL buffer size - optimize for our display
// Use 1/4 of the screen size to balance memory usage and performance
#define LVGL_BUFFER_SIZE (SCREEN_WIDTH * (SCREEN_HEIGHT / 4))

// LVGL refresh time in ms - match the config file setting
#define LVGL_REFRESH_TIME 40

// Core LVGL globals
extern int8_t displayCS;  // Display chip select pin
extern lv_disp_drv_t disp_drv;
extern lv_disp_draw_buf_t draw_buf;
extern lv_color_t buf[LVGL_BUFFER_SIZE];
extern Adafruit_ST7735* tft_driver;
extern uint32_t lvgl_last_update;
// Shared SPI bus mutex (guards TFT + MCP2515 access)
extern SemaphoreHandle_t spiBusMutex;

// Core function declarations
void setupLvglBuffer();
void setupLvglDisplay(const STR_DEVICE_DATA_140_V1& deviceData, int8_t dc_pin, int8_t rst_pin, SPIClass* spi);
void lvgl_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);
void lv_tick_handler();
void updateLvgl();
void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration);

#endif // LVGL_CORE_H
