#include "emulator_display.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

// Include project headers that define the display globals
#include "sp140/lvgl/lvgl_core.h"
#include "sp140/lvgl/lvgl_main_screen.h"
#include "sp140/structs.h"

// Full-screen framebuffer: 160 x 128 pixels, RGB565
static uint16_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

// LVGL draw buffer - must be aligned to LV_DRAW_BUF_ALIGN (4 bytes).
// LVGL v9 asserts on misaligned buffers and spins in while(1) on failure.
static uint8_t lvgl_buf1[SCREEN_WIDTH * SCREEN_HEIGHT * 2] __attribute__((aligned(4)));

// Define globals declared in lvgl_core.h
lv_display_t* main_display = nullptr;
int8_t displayCS = -1;
Adafruit_ST7735* tft_driver = nullptr;
uint32_t lvgl_last_update = 0;
SemaphoreHandle_t spiBusMutex = nullptr;

// Define lvglMutex from lvgl_updates.h
SemaphoreHandle_t lvglMutex = nullptr;

// Track initialization state
static bool lvgl_initialized = false;

uint16_t* emulator_get_framebuffer() {
  return framebuffer;
}

// Flush callback: copies rendered pixels into our framebuffer
static void emulator_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;
  uint16_t* src = (uint16_t*)px_map;

  for (int32_t y = 0; y < h; y++) {
    for (int32_t x = 0; x < w; x++) {
      int fb_x = area->x1 + x;
      int fb_y = area->y1 + y;
      if (fb_x >= 0 && fb_x < SCREEN_WIDTH && fb_y >= 0 && fb_y < SCREEN_HEIGHT) {
        framebuffer[fb_y * SCREEN_WIDTH + fb_x] = src[y * w + x];
      }
    }
  }

  lv_display_flush_ready(disp);
}

lv_display_t* emulator_init_display(bool darkMode) {
  // Clear framebuffer
  memset(framebuffer, 0, sizeof(framebuffer));

  if (!lvgl_initialized) {
    // First time: initialize LVGL
    lv_init();
    lvgl_initialized = true;

    // Create display with framebuffer flush
    main_display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_display_set_flush_cb(main_display, emulator_flush_cb);
    lv_display_set_buffers(main_display, lvgl_buf1, NULL, sizeof(lvgl_buf1),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_color_format(main_display, LV_COLOR_FORMAT_RGB565);
  }

  // Set up theme (matches real hardware setup in lvgl_core.cpp)
  lv_theme_t* theme = lv_theme_default_init(
    main_display,
    lv_palette_main(LV_PALETTE_BLUE),
    lv_palette_main(LV_PALETTE_AMBER),
    darkMode,
    LV_FONT_DEFAULT);
  lv_display_set_theme(main_display, theme);

  return main_display;
}

void emulator_render_frame() {
  // Tick LVGL forward so any time-dependent layout/animation work is scheduled.
  lv_tick_inc(100);
  // Drain LVGL's internal timer queue before forcing the refresh.  Without
  // this the first render on a freshly-created screen can block waiting for
  // deferred layout passes that are only triggered by lv_timer_handler().
  for (int i = 0; i < 64; i++) {
    lv_timer_handler();
  }
  lv_refr_now(main_display);
}

// Write a 24-bit BMP file from the RGB565 framebuffer
bool emulator_save_bmp(const char* filename) {
  FILE* f = fopen(filename, "wb");
  if (!f) return false;

  const int width = SCREEN_WIDTH;
  const int height = SCREEN_HEIGHT;
  // BMP rows must be padded to 4-byte boundaries
  int row_bytes = width * 3;
  int padding = (4 - (row_bytes % 4)) % 4;
  int padded_row = row_bytes + padding;
  int pixel_data_size = padded_row * height;
  int file_size = 54 + pixel_data_size;

  // BMP File Header (14 bytes)
  uint8_t file_header[14] = {
    'B', 'M',
    (uint8_t)(file_size), (uint8_t)(file_size >> 8),
    (uint8_t)(file_size >> 16), (uint8_t)(file_size >> 24),
    0, 0, 0, 0,
    54, 0, 0, 0
  };
  fwrite(file_header, 1, 14, f);

  // BMP Info Header (40 bytes)
  uint8_t info_header[40] = {};
  info_header[0] = 40;  // header size
  info_header[4] = (uint8_t)(width);
  info_header[5] = (uint8_t)(width >> 8);
  info_header[8] = (uint8_t)(height);
  info_header[9] = (uint8_t)(height >> 8);
  info_header[12] = 1;   // color planes
  info_header[14] = 24;  // bits per pixel
  // compression = 0 (BI_RGB), rest zeroed
  fwrite(info_header, 1, 40, f);

  // Pixel data (BMP is bottom-up)
  uint8_t pad_bytes[3] = {0, 0, 0};
  for (int y = height - 1; y >= 0; y--) {
    for (int x = 0; x < width; x++) {
      uint16_t rgb565 = framebuffer[y * width + x];
      // Convert RGB565 to BGR24
      uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
      uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
      uint8_t b = (rgb565 & 0x1F) * 255 / 31;
      uint8_t bgr[3] = {b, g, r};
      fwrite(bgr, 1, 3, f);
    }
    if (padding > 0) {
      fwrite(pad_bytes, 1, padding, f);
    }
  }

  fclose(f);
  return true;
}

int emulator_compare_bmp(const char* file_a, const char* file_b) {
  FILE* fa = fopen(file_a, "rb");
  FILE* fb = fopen(file_b, "rb");
  if (!fa || !fb) {
    if (fa) fclose(fa);
    if (fb) fclose(fb);
    return -1;  // File open error
  }

  // Get file sizes
  fseek(fa, 0, SEEK_END);
  fseek(fb, 0, SEEK_END);
  long size_a = ftell(fa);
  long size_b = ftell(fb);
  fseek(fa, 0, SEEK_SET);
  fseek(fb, 0, SEEK_SET);

  if (size_a != size_b) {
    fclose(fa);
    fclose(fb);
    return -2;  // Size mismatch
  }

  // Skip BMP header (54 bytes), compare pixel data
  fseek(fa, 54, SEEK_SET);
  fseek(fb, 54, SEEK_SET);

  int diff_count = 0;
  int pixel_count = SCREEN_WIDTH * SCREEN_HEIGHT;
  for (int i = 0; i < pixel_count; i++) {
    uint8_t bgr_a[3], bgr_b[3];
    if (fread(bgr_a, 1, 3, fa) != 3 || fread(bgr_b, 1, 3, fb) != 3) break;
    if (bgr_a[0] != bgr_b[0] || bgr_a[1] != bgr_b[1] || bgr_a[2] != bgr_b[2]) {
      diff_count++;
    }
  }

  fclose(fa);
  fclose(fb);
  return diff_count;
}

void emulator_teardown() {
  // Delete the screen (which deletes all child widgets)
  // Don't call lv_deinit - keep LVGL alive across tests
  if (main_screen != NULL) {
    lv_obj_delete(main_screen);
  }

  // Reset all global widget pointers (children are already deleted with main_screen)
  main_screen = NULL;
  battery_bar = NULL;
  battery_label = NULL;
  voltage_left_label = NULL;
  voltage_right_label = NULL;
  for (int i = 0; i < 4; i++) power_char_labels[i] = NULL;
  power_unit_label = NULL;
  power_bar = NULL;
  perf_mode_label = NULL;
  armed_time_label = NULL;
  for (int i = 0; i < 7; i++) altitude_char_labels[i] = NULL;
  batt_temp_label = NULL;
  esc_temp_label = NULL;
  motor_temp_label = NULL;
  arm_indicator = NULL;
  batt_letter_label = NULL;
  esc_letter_label = NULL;
  motor_letter_label = NULL;
  batt_temp_bg = NULL;
  esc_temp_bg = NULL;
  motor_temp_bg = NULL;
  cruise_icon_img = NULL;
  charging_icon_img = NULL;
  arm_fail_warning_icon_img = NULL;
  for (int i = 0; i < 13; i++) climb_rate_divider_lines[i] = NULL;
  for (int i = 0; i < 12; i++) climb_rate_fill_sections[i] = NULL;
  critical_border = NULL;

  // Reset alert UI pointers (declared in lvgl_alerts.h)
  extern lv_obj_t* warning_counter_circle;
  extern lv_obj_t* warning_counter_label;
  extern lv_obj_t* critical_counter_circle;
  extern lv_obj_t* critical_counter_label;
  extern lv_obj_t* alert_text_label;
  extern lv_obj_t* critical_text_label;
  extern lv_timer_t* alert_cycle_timer;
  warning_counter_circle = NULL;
  warning_counter_label = NULL;
  critical_counter_circle = NULL;
  critical_counter_label = NULL;
  alert_text_label = NULL;
  critical_text_label = NULL;
  alert_cycle_timer = NULL;

  // Reset flash timer state
  extern lv_timer_t* cruise_flash_timer;
  extern lv_timer_t* arm_fail_flash_timer;
  extern lv_timer_t* critical_border_flash_timer;
  extern bool isFlashingCruiseIcon;
  extern bool isFlashingArmFailIcon;
  extern bool isFlashingCriticalBorder;
  cruise_flash_timer = NULL;
  arm_fail_flash_timer = NULL;
  critical_border_flash_timer = NULL;
  isFlashingCruiseIcon = false;
  isFlashingArmFailIcon = false;
  isFlashingCriticalBorder = false;

  memset(framebuffer, 0, sizeof(framebuffer));
}
