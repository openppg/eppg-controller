// Copyright 2020 <Zach Whitehead>
#include "../../../inc/sp140/lvgl/lvgl_core.h"
#include "../../../inc/version.h"
#include "../../../inc/sp140/esp32s3-config.h"

// Global variables for core LVGL functionality
lv_disp_drv_t disp_drv;
lv_disp_draw_buf_t draw_buf;
lv_color_t buf[LVGL_BUFFER_SIZE];
lv_color_t buf2[LVGL_BUFFER_SIZE];  // Second buffer for double buffering
lgfx::LGFX_Device* tft_driver = nullptr;
extern SPIClass* hardwareSPI; // External reference to shared SPI instance

// Custom LovyanGFX configuration for ST7735
class LGFX_ST7735 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7735S _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX_ST7735() {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;  // Use SPI2 (HSPI)
      cfg.spi_mode = 0;
      cfg.freq_write = 27000000;  // SPI speed for writing
      cfg.freq_read = 16000000;   // SPI speed for reading
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 12;
      cfg.pin_mosi = 11;
      cfg.pin_miso = 13;
      cfg.pin_dc = 14;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = 10;
      cfg.pin_rst = 15;
      cfg.pin_busy = -1;
      cfg.memory_width = 160;
      cfg.memory_height = 128;
      cfg.panel_width = 160;
      cfg.panel_height = 128;
      // ST7735 BLACKTAB variant - specific settings for 160x128 display
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = false;  // User reported darker display, so no invert needed
      cfg.rgb_order = false;  // Try RGB order instead of BGR
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;  // Important for shared SPI
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
uint32_t lvgl_last_update = 0;
// Define the shared SPI bus mutex
SemaphoreHandle_t spiBusMutex = NULL;

void setupLvglBuffer() {
  // Initialize LVGL library
  USBSerial.println("Initializing LVGL");
  lv_init();
  USBSerial.println("LVGL initialized");

  // Setup double buffer for LVGL to reduce tearing
  USBSerial.println("Setting up LVGL double buffer");
  lv_disp_draw_buf_init(&draw_buf, buf, buf2, LVGL_BUFFER_SIZE);
  USBSerial.println("LVGL double buffer initialized");
}

void setupLvglDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  int8_t dc_pin,
  int8_t rst_pin,
  SPIClass* spi
) {
  // Note: TFT_eSPI doesn't use the spi parameter - it's configured via build flags
  USBSerial.println("Setting up LVGL display");

  // Create SPI bus mutex on first use if not already created
  if (spiBusMutex == NULL) {
    spiBusMutex = xSemaphoreCreateMutex();
    if (spiBusMutex == NULL) {
      USBSerial.println("Error creating SPI bus mutex");
    }
  }

  // Create the LovyanGFX driver instance if not already created
  if (tft_driver == nullptr) {
    // Store the shared SPI instance
    hardwareSPI = spi;
    
    // Create custom ST7735 configuration for LovyanGFX
    tft_driver = new LGFX_ST7735();
    
    // Initialize display with custom configuration
    if (tft_driver->init()) {
      tft_driver->setRotation(deviceData.screen_rotation);
      
      // Clear screen with a test pattern to verify it's working
      tft_driver->fillScreen(0x0000);  // Black
      tft_driver->fillRect(10, 10, 50, 50, 0xF800);  // Red square for testing
      
      USBSerial.println("LovyanGFX ST7735 initialized successfully");
    } else {
      USBSerial.println("ERROR: LovyanGFX ST7735 initialization failed!");
    }
  }

  // Initialize LVGL buffer
  setupLvglBuffer();

  // Initialize display driver (use global disp_drv)
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

// High-performance flush callback using LovyanGFX with DMA
void lvgl_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  // LovyanGFX handles SPI transactions internally, but we still guard for MCP2515
  if (spiBusMutex != NULL) {
    xSemaphoreTake(spiBusMutex, portMAX_DELAY);
  }

  // LovyanGFX DMA-optimized pixel transfer
  tft_driver->startWrite();
  tft_driver->setAddrWindow(area->x1, area->y1, w, h);
  tft_driver->writePixels((uint16_t*)color_p, w * h);  // NOLINT(readability/casting)
  tft_driver->endWrite();

  if (spiBusMutex != NULL) {
    xSemaphoreGive(spiBusMutex);
  }

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

// Shared SPI helper function
SPIClass* getSharedSPI() {
  return hardwareSPI;
}

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
