#ifndef INC_SP140_LVGL_LVGL_CORE_H_
#define INC_SP140_LVGL_LVGL_CORE_H_

#include <lvgl.h>
#include <Adafruit_ST7735.h>
#include "../structs.h"
// FreeRTOS for mutex used to guard shared SPI bus
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Display dimensions
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

// LVGL buffer size in bytes - half screen for partial rendering
// v9 uses byte-based buffers: width * height * bytes_per_pixel / 2
#define LVGL_BUF_BYTES (SCREEN_WIDTH * SCREEN_HEIGHT * 2 / 2)

// LVGL refresh time in ms - match the config file setting
#define LVGL_REFRESH_TIME 40

// Core LVGL globals
extern int8_t displayCS;  // Display chip select pin
extern lv_display_t* main_display;
extern Adafruit_ST7735* tft_driver;
extern uint32_t lvgl_last_update;
// Shared SPI bus mutex (guards TFT + MCP2515 access)
extern SemaphoreHandle_t spiBusMutex;

// Core function declarations
void setupLvglBuffer();
void setupLvglDisplay(const STR_DEVICE_DATA_140_V1& deviceData, int8_t dc_pin, int8_t rst_pin, SPIClass* spi);
void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
void lv_tick_handler();
void updateLvgl();
void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration);

#endif  // INC_SP140_LVGL_LVGL_CORE_H_
