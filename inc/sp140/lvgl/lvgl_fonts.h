#ifndef INC_SP140_LVGL_LVGL_FONTS_H_
#define INC_SP140_LVGL_LVGL_FONTS_H_

#include <lvgl.h>

// Montserrat Medium, the same face as LVGL's built-in fonts, regenerated with
// strong autohinting (src/assets/fonts/montserrat_hinted_*.c, lv_font_conv
// --autohint-strong --bpp 4). Hinting snaps vertical stems to whole pixels,
// so small text on the 160x128 screen gets solid strokes (the built-in
// fonts smear e.g. the right leg of "M" across two grey columns) while
// curves keep their 4-bit anti-aliasing. Size 14 is also LVGL's default font
// (lv_conf.h) and carries the LV_SYMBOL_* icons.
LV_FONT_DECLARE(montserrat_hinted_10)
LV_FONT_DECLARE(montserrat_hinted_12)
LV_FONT_DECLARE(montserrat_hinted_14)
LV_FONT_DECLARE(montserrat_hinted_16)
LV_FONT_DECLARE(montserrat_hinted_18)
LV_FONT_DECLARE(montserrat_hinted_24)
LV_FONT_DECLARE(montserrat_hinted_28)

#endif  // INC_SP140_LVGL_LVGL_FONTS_H_
