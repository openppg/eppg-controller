#pragma once

#include <Arduino.h>
#include <lvgl.h>
#include <SPI.h>

// Include current device structures
#include "../../sp140/structs.h"

// Enum to define different screen pages
typedef enum {
  MAIN_SCREEN,  // Default main display screen
  // Add other screen states here later (e.g., SETTINGS_SCREEN, INFO_SCREEN)
} ScreenPage;

// Global variable to track the current screen page
extern ScreenPage currentScreenPage;

// Expose main screen object for setup sequence control
extern lv_obj_t* main_screen;

// Function to switch between screen pages
void switchScreenPage(ScreenPage newPage);

// Function to set up the main screen UI elements
void setupMainScreen(bool darkMode);

// Initialize LVGL display and components
void setupLvglDisplay(const STR_DEVICE_DATA_140_V1& deviceData, int8_t dc_pin, int8_t rst_pin, SPIClass* spi);

// Display splash screen with LVGL
void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration = 1500);

// Set LVGL display buffer
void setupLvglBuffer();

// LVGL display flush callback
void lvgl_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);

// LVGL tick handler
void lv_tick_handler();

// Update LVGL - calls lv_task_handler and updates internal ticks
void updateLvgl();

// Main screen update function
void updateLvglMainScreen(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
);

// Functions to control the loading overlay
void showLoadingOverlay();
void hideLoadingOverlay();

// Cruise Control Icon
// LV_IMG_DECLARE(noun_cruise_control_340255_1); // Old 40x40 icon
LV_IMG_DECLARE(cruise_control_340255_30); // Use the corrected name from the new 30x30 C file
extern lv_obj_t *cruise_icon_img; // Declaration for the image object

// External declarations
extern SPIClass* hardwareSPI;
extern int8_t displayCS;
