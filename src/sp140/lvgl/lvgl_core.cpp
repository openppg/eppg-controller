// Copyright 2020 <Zach Whitehead>
#include "../../../inc/sp140/lvgl/lvgl_core.h"
#include "../../../inc/version.h"
#include "../../../inc/sp140/esp32s3-config.h"

// Global variables for core LVGL functionality
lv_display_t* main_display = nullptr;
static uint8_t buf[LVGL_BUF_BYTES];
static uint8_t buf2[LVGL_BUF_BYTES];
Adafruit_ST7735* tft_driver = nullptr;
uint32_t lvgl_last_update = 0;
// Define the shared SPI bus mutex
SemaphoreHandle_t spiBusMutex = NULL;

void setupLvglBuffer() {
  // Initialize LVGL library
  lv_init();
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
  if (main_display == NULL) {
    USBSerial.println("FATAL: Failed to create LVGL display");
    return;
  }
  lv_display_set_flush_cb(main_display, lvgl_flush_cb);
  lv_display_set_buffers(main_display, buf, buf2, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
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
  // Guard against NULL pointers that would cause hard fault
  if (tft_driver == nullptr || px_map == NULL) {
    lv_display_flush_ready(disp);
    return;
  }

  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  // Set drawing window
  // Guard shared SPI bus for display flush
  if (spiBusMutex != NULL) {
    if (xSemaphoreTake(spiBusMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
      // SPI bus timeout - BMS might be doing long operation, skip display flush
      USBSerial.println("[DISPLAY] SPI bus timeout - skipping display flush");
      // Must still signal LVGL that flush is done to avoid deadlock
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
    lv_timer_handler();
    lvgl_last_update = current_ms;
  }
}

void displayLvglSplash(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  USBSerial.println("Displaying LVGL splash screen");

  // Create a new screen for the splash
  lv_obj_t* splash_screen = lv_obj_create(NULL);
  lv_screen_load(splash_screen);

  // Disable scrollbars
  lv_obj_remove_flag(splash_screen, LV_OBJ_FLAG_SCROLLABLE);

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
    lv_obj_set_style_opa((lv_obj_t*)var, value, 0);  // NOLINT(readability/casting)
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
    vTaskDelay(pdMS_TO_TICKS(LVGL_REFRESH_TIME));
  }
}
