/**
 * @file lv_conf.h
 * Configuration file for LVGL v9
 */

#ifndef LV_CONF_H  // NOLINT(build/header_guard)
#define LV_CONF_H  // NOLINT(build/header_guard)

/*====================
   COLOR SETTINGS
 *====================*/

/*Color depth: 8 (A8), 16 (RGB565), 24 (RGB888), 32 (XRGB8888)*/
#define LV_COLOR_DEPTH 16

/*=========================
   STDLIB SETTINGS
 *=========================*/

/*Use LVGL's built-in memory manager*/
#define LV_USE_STDLIB_MALLOC LV_STDLIB_BUILTIN

/*Size of the memory available for `lv_malloc()` in bytes (>= 2kB)*/
#define LV_MEM_SIZE (48U * 1024U)

#define LV_USE_STDLIB_STRING  LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_BUILTIN

/*=========================
   OS SETTINGS
 *=========================*/

/*Use no OS - keep existing lvglMutex for now*/
#define LV_USE_OS LV_OS_NONE

/*====================
   HAL SETTINGS
 *====================*/

/*Default display refresh, period. LVGL will redraw changed areas with this period time*/
#define LV_DEF_REFR_PERIOD 30      /*[ms]*/

/*Default Dot Per Inch. Used to initialize default sizes such as widgets sized, style paddings.*/
#define LV_DPI_DEF 130     /*[px/inch]*/

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

/*-------------
 * Drawing
 *-----------*/

/*Enable complex draw engine*/
#define LV_DRAW_SW_COMPLEX 1

/*Allow buffering some shadow calculation.
 *LV_DRAW_SW_SHADOW_CACHE_SIZE is the max. shadow size to buffer*/
#define LV_DRAW_SW_SHADOW_CACHE_SIZE 0

/*Set number of maximally cached circle data*/
#define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4

/*Buffer size for simple layers*/
#define LV_DRAW_LAYER_SIMPLE_BUF_SIZE (24 * 1024)

/*Default image cache size*/
#define LV_IMAGE_CACHE_DEF_SIZE 0

/*Number of stops allowed per gradient*/
#define LV_GRADIENT_MAX_STOPS 2

/*Number of parallel draw units (1 = single threaded)*/
#define LV_DRAW_THREAD_COUNT 1

/*-------------
 * Fonts
 *-----------*/

/**
 * Default font to use
 */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/**
 * Enable built-in fonts - only enable a minimal set
 */
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 1  /* For smaller kW unit label */
#define LV_FONT_MONTSERRAT_12 1  /* For medium text, temperatures, and voltage labels */
#define LV_FONT_MONTSERRAT_14 1  /* Used as default font */
#define LV_FONT_MONTSERRAT_16 1  /* For battery percentage and power */
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1  /* For larger power display numbers */
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 1  /* For larger power display numbers */
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

/**
 * Enable handling large font and/or fonts with a lot of characters.
 */
#define LV_FONT_FMT_TXT_LARGE 0

/*Enables/disables support for compressed fonts.*/
#define LV_USE_FONT_COMPRESSED 0

/*Enable drawing placeholders when glyph dsc is not found*/
#define LV_USE_FONT_PLACEHOLDER 1

/*=================
 *  TEXT SETTINGS
 *=================*/

/**
 * Select a character encoding for strings.
 */
#define LV_TXT_ENC LV_TXT_ENC_UTF8

/*Can break (wrap) texts on these chars*/
#define LV_TXT_BREAK_CHARS " ,.;:-_"

/*=================
 *  WIDGET USAGE
 *=================*/

#define LV_USE_ANIMIMAGE    0

#define LV_USE_ARC        1

#define LV_USE_BAR        1

#define LV_USE_BUTTON     1

#define LV_USE_BUTTONMATRIX  1  /* Required by other components */

#define LV_USE_CANVAS     0  /* Don't need canvas */

#define LV_USE_CALENDAR   0 /* Explicitly disable calendar widget */

#define LV_USE_CHECKBOX   0  /* Don't need checkbox */

#define LV_USE_DROPDOWN   0  /* Don't need dropdown */

#define LV_USE_IMAGE      1   /*Requires: lv_label*/

#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION 1 /*Enable selecting text of the label*/
    #define LV_LABEL_LONG_TXT_HINT 1  /*Store some extra info in labels to speed up drawing of very long texts*/
#endif

#define LV_USE_LINE       1

#define LV_USE_ROLLER     0  /* Don't need roller */

#define LV_USE_SLIDER     0  /* Don't need slider */

#define LV_USE_SPAN       1
#if LV_USE_SPAN
    /*A line text can contain maximum num of span descriptor */
    #define LV_SPAN_SNIPPET_STACK_SIZE 64
#endif

#define LV_USE_SWITCH     0  /* Don't need switch */

#define LV_USE_TEXTAREA   1  /* Required by other components */

#define LV_USE_SPINNER    1

/* keyboard widget (set to 0 since we don't use it) */
#define LV_USE_KEYBOARD   0

/*Extra widgets*/
#define LV_USE_LED        1

/*==================
 * THEMES
 *==================*/

/*A simple, impressive and very complete theme*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT

    /*0: Light mode; 1: Dark mode*/
    #define LV_THEME_DEFAULT_DARK 0

    /*1: Enable grow on press*/
    #define LV_THEME_DEFAULT_GROW 0

    /*Default transition time in [ms]*/
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif /*LV_USE_THEME_DEFAULT*/

/*==================
* LAYOUTS
*==================*/

/*A layout similar to Flexbox in CSS.*/
#define LV_USE_FLEX 1

/*A layout similar to Grid in CSS.*/
#define LV_USE_GRID 1

/*====================
 * 3RD PARTS LIBRARIES
 *====================*/

#define LV_USE_FS_STDIO 0

/*====================
 * DEMOS & EXAMPLES
 *====================*/

#define LV_BUILD_EXAMPLES 0

#endif /*LV_CONF_H*/
