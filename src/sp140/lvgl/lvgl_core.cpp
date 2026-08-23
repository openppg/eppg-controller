// Copyright 2020 <Zach Whitehead>
#include "../../../inc/sp140/lvgl/lvgl_core.h"
#include "sp140/lvgl/lvgl_splash.h"
#include "../../../inc/sp140/esp32s3-config.h"

// Global variables for core LVGL functionality
lv_display_t* main_display = nullptr;
static uint8_t buf[LVGL_BUF_BYTES];
Adafruit_ST7735* tft_driver = nullptr;
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
  lv_tick_set_cb([]() -> uint32_t { return millis(); });
}

void setupLvglDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  int8_t dc_pin,
  int8_t rst_pin,
  SPIClass* spi
) {
  USBSerial.println("Initializing display system");

  // Create SPI bus mutex on first use if not already created
  if (spiBusMutex == NULL) {
    spiBusMutex = xSemaphoreCreateMutex();
    if (spiBusMutex == NULL) {
      USBSerial.println("Error creating SPI bus mutex");
    }
  }

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

  // Create display and configure it
  main_display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(main_display, lvgl_flush_cb);
  // The current SPI flush is synchronous, so LVGL cannot render into a second
  // buffer while the first is being transferred. Keep one half-screen buffer
  // and restore double buffering when the transport becomes asynchronous.
  lv_display_set_buffers(main_display, buf, nullptr, sizeof(buf),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_color_format(main_display, LV_COLOR_FORMAT_RGB565);

  USBSerial.println("Display driver registered");

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
      USBSerial.println("[DISPLAY] SPI bus timeout - skipping display flush");
      // Must still signal LVGL that flush is done to avoid deadlock. That
      // marks the area clean without it ever reaching the panel, so flag it
      // for a recovery repaint (handled in updateLvgl).
      flushSkipped = true;
      lv_display_flush_ready(disp);
      return;
    }
  }

  // Make sure display CS is selected only after SPI bus is acquired
  digitalWrite(displayCS, LOW);
  tft_driver->startWrite();
  tft_driver->setAddrWindow(area->x1, area->y1, w, h);

  // Push colors - using DMA if available
  uint32_t len = w * h;
  tft_driver->writePixels((uint16_t*)px_map, len);  // NOLINT(readability/casting)
  tft_driver->endWrite();

  // Deselect display CS when done
  digitalWrite(displayCS, HIGH);
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
  USBSerial.println("Displaying LVGL splash screen");
  createLvglSplashScreen(deviceData);

  // Process LVGL for the duration
  uint32_t start_time = millis();
  while (millis() - start_time < duration) {
    updateLvgl();
    vTaskDelay(pdMS_TO_TICKS(LVGL_REFRESH_TIME));
  }
}
