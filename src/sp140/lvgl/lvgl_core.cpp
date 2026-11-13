#include "platform/runtime.h"
// Copyright 2020 <Zach Whitehead>
#include "../../../inc/sp140/lvgl/lvgl_core.h"
#include "sp140/lvgl/lvgl_splash.h"
#include "../../../inc/sp140/esp32s3-config.h"

// Global variables for core LVGL functionality
lv_display_t* main_display = nullptr;
static uint8_t buf[LVGL_BUF_BYTES];
static uint8_t buf2[LVGL_BUF_BYTES];
platform::St7735* tft_driver = nullptr;
// Define the shared SPI bus mutex
SemaphoreHandle_t spiBusMutex = NULL;
// Set when a flush had to be skipped because the SPI bus was busy. LVGL is
// told the flush succeeded (required to keep rendering), so the skipped area
// would otherwise never be repainted — updateLvgl() checks this flag and
// forces a full repaint to recover.
static volatile bool flushSkipped = false;

void setupLvglBuffer() {
  // Initialize LVGL library
  lv_init();
  // Let LVGL read time directly so animation/refresh pacing stays accurate
  // regardless of how often lv_timer_handler() gets called.
  lv_tick_set_cb([]() -> uint32_t { return platform::millis32(); });
}

void setupLvglDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  int8_t dc_pin,
  int8_t rst_pin
) {
  platform::console.line("Initializing display system");

  // Create SPI bus mutex on first use if not already created
  if (spiBusMutex == NULL) {
    spiBusMutex = xSemaphoreCreateMutex();
    if (spiBusMutex == NULL) {
      platform::console.line("Error creating SPI bus mutex");
    }
  }

  // Create the TFT driver instance if not already created
  if (tft_driver == nullptr) {
    tft_driver = new platform::St7735();
    if (!tft_driver->initialize(displayCS, dc_pin, rst_pin, deviceData.screen_rotation)) {
      platform::console.line("ST7735 initialization failed");
    }
  }

  // Initialize LVGL buffer
  setupLvglBuffer();

  // Create display and configure it
  main_display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(main_display, lvgl_flush_cb);
  lv_display_set_buffers(main_display, buf, buf2, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_color_format(main_display, LV_COLOR_FORMAT_RGB565);

  platform::console.line("Display driver registered");

  // Set LVGL default theme - using default font
  lv_theme_t* theme = lv_theme_default_init(
    main_display,                           // Display
    lv_palette_main(LV_PALETTE_BLUE),     // Primary color
    lv_palette_main(LV_PALETTE_AMBER),    // Secondary color
    deviceData.theme == 1,                // Dark mode
    LV_FONT_DEFAULT);                     // Default font

  lv_display_set_theme(main_display, theme);
}

// Optimize the flush callback to minimize SPI transfers
// CS pin management is handled here where actual SPI communication occurs
void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  // Set drawing window
  // Guard shared SPI bus for display flush
  if (spiBusMutex != NULL) {
    if (xSemaphoreTake(spiBusMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
      // SPI bus timeout - BMS might be doing long operation, skip display flush
      platform::console.line("[DISPLAY] SPI bus timeout - skipping display flush");
      // Must still signal LVGL that flush is done to avoid deadlock. That
      // marks the area clean without it ever reaching the panel, so flag it
      // for a recovery repaint (handled in updateLvgl).
      flushSkipped = true;
      lv_display_flush_ready(disp);
      return;
    }
  }

  if (!tft_driver->writeRegion(area->x1, area->y1, w, h,
                              reinterpret_cast<const uint16_t*>(px_map))) {
    flushSkipped = true;
  }

  if (spiBusMutex != NULL) {
    xSemaphoreGive(spiBusMutex);
  }

  // Indicate to LVGL that flush is done
  lv_display_flush_ready(disp);
}

// Update LVGL - call this regularly with lvglMutex held. LVGL paces actual
// redraws internally (LV_DEF_REFR_PERIOD), so no extra gating is needed here.
void updateLvgl() {
  lv_timer_handler();

  // A flush was dropped because the SPI bus was busy: LVGL believes those
  // pixels reached the panel, so invalidate everything once to repaint any
  // stale content on the next refresh cycle.
  if (flushSkipped) {
    flushSkipped = false;
    lv_obj_t* screen = lv_screen_active();
    if (screen != NULL) {
      lv_obj_invalidate(screen);
    }
  }
}

void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  platform::console.line("Displaying LVGL splash screen");
  createLvglSplashScreen(deviceData);

  // Process LVGL for the duration
  uint32_t start_time = platform::millis32();
  while (platform::millis32() - start_time < duration) {
    updateLvgl();
    vTaskDelay(pdMS_TO_TICKS(LVGL_REFRESH_TIME));
  }
}
