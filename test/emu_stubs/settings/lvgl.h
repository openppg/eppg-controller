// Minimal LVGL 9 surface needed to compile first_boot_qc.cpp and the screen
// headers it includes. The QC screen itself is faked in the test.
#pragma once

#include <stdint.h>

typedef struct _lv_obj_t lv_obj_t;
typedef struct _lv_display_t lv_display_t;
typedef uint8_t lv_opa_t;

typedef struct {
  uint8_t blue;
  uint8_t green;
  uint8_t red;
} lv_color_t;

typedef struct {
  int32_t x1, y1, x2, y2;
} lv_area_t;

typedef struct {
  void* values;
} lv_style_t;

typedef struct {
  int line_height;
} lv_font_t;

#define LV_FONT_DECLARE(font_name) extern const lv_font_t font_name;

inline lv_color_t lv_color_make(uint8_t r, uint8_t g, uint8_t b) { return {b, g, r}; }
inline lv_color_t lv_color_black() { return lv_color_make(0, 0, 0); }
inline lv_color_t lv_color_white() { return lv_color_make(255, 255, 255); }

inline uint32_t& emuLvTimerHandlerCalls() {
  static uint32_t n = 0;
  return n;
}

inline uint32_t lv_timer_handler() {
  emuLvTimerHandlerCalls()++;
  return 5;
}
