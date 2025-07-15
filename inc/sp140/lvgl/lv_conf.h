/**
 * @file lv_conf.h
 * Configuration file for LVGL v9
 */

#ifndef LV_CONF_H  // NOLINT(build/header_guard)
#define LV_CONF_H  // NOLINT(build/header_guard)

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/*Color depth: 8 (A8), 16 (RGB565), 24 (RGB888), 32 (XRGB8888)*/
#define LV_COLOR_DEPTH 16

/*=========================
   MEMORY SETTINGS
 *=========================*/

/*1: use custom malloc/free, 0: use the built-in `lv_malloc()` and `lv_free()`*/
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_STRING    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN

/*Size of the memory available for `lv_malloc()` in bytes (>= 2kB)*/
#define LV_MEM_SIZE (48U * 1024U)          /*[bytes]*/

/*===================
 * HAL SETTINGS
 *==================*/

/*Default display refresh, input device read and animation step period.*/
#define LV_DEF_REFR_PERIOD  30      /*[ms]*/

/*Default Dot Per Inch. Used to initialize default sizes such as widgets sized, style paddings.
 *(Not so important, you can adjust it to modify default sizes and spaces)*/
#define LV_DPI_DEF 130     /*[px/inch]*/

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

/*-------------
 * Drawing
 *-----------*/

/*Enable complex draw engine.
 *Required to draw shadow, gradient, rounded corners, circles, arc, skew lines, image transformations or any masks*/
#define LV_USE_DRAW_SW 1

/*Allow buffering some shadow calculation.
 *LV_DRAW_SW_SHADOW_CACHE_SIZE is the max. shadow size to buffer, where shadow size is `shadow_width + radius`
 *Caching has LV_DRAW_SW_SHADOW_CACHE_SIZE^2 RAM cost*/
#define LV_DRAW_SW_SHADOW_CACHE_SIZE 0

/* Set number of maximally cached circle data.
 * The circumference of 1/4 circle are saved for anti-aliasing
 * radius * 4 bytes are used per circle (the most often used radiuses are saved)
 * 0: to disable caching */
#define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4

/*Enable drawing complex masks*/
#define LV_USE_DRAW_MASKS 1

/*Default gradient buffer size.
 *When LVGL calculates the gradient "maps" it can save them into a cache to avoid calculating them again.
 *LV_DRAW_SW_GRADIENT_CACHE_DEF_SIZE sets the size of this cache in bytes.
 *If the cache is too small the map will be allocated only while it's required for the drawing.
 *0 mean no caching.*/
#define LV_DRAW_SW_GRADIENT_CACHE_DEF_SIZE 0

/*Allow dithering the gradients (to achieve visual smooth color gradients on limited color depth display)
 *LV_DRAW_SW_GRADIENT_DITHER implies allocating one or two more lines of the object's rendering surface
 *The increase in memory consumption is (32 bits * object width) plus 24 bits * object width if using error diffusion */
#define LV_DRAW_SW_GRADIENT_DITHER 0

/*-------------
 * GPU
 *-----------*/

/*Use STM32's DMA2D (aka Chrom Art) GPU*/
#define LV_USE_DRAW_DMA2D 0

/*Use SWM341's DMA2D GPU*/
#define LV_USE_DRAW_SWM341_DMA2D 0

/*Use NXP's PXP GPU iMX RTxxx platforms*/
#define LV_USE_DRAW_PXP 0

/*Use NXP's VGLite GPU iMX RTxxx platforms*/
#define LV_USE_DRAW_VGLITE 0

/*Use SDL renderer API*/
#define LV_USE_DRAW_SDL 0

/*Use Arm's 2D acceleration library Arm-2D */
#define LV_USE_DRAW_ARM2D 0

/*Use NXP's G2D GPU*/
#define LV_USE_DRAW_NXP_G2D 0

/*Use VG-Lite GPU*/
#define LV_USE_DRAW_VG_LITE 0

#define LV_USE_DRAW_SW_BLEND_HELIUM 0

#define LV_USE_DRAW_ARM2D_SYNC 0

#define LV_USE_NATIVE_HELIUM_ASM 0

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

/*-------------
 * Logging
 *-----------*/

/*Enable the log module*/
#define LV_USE_LOG 1
#if LV_USE_LOG

    /*How important log should be added:
     *LV_LOG_LEVEL_TRACE       A lot of logs to give detailed information
     *LV_LOG_LEVEL_INFO        Log important events
     *LV_LOG_LEVEL_WARN        Log if something unwanted happened but didn't cause a problem
     *LV_LOG_LEVEL_ERROR       Only critical issue, when the system may fail
     *LV_LOG_LEVEL_USER        Only logs added by the user
     *LV_LOG_LEVEL_NONE        Do not log anything*/
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN

    /*1: Print the log with 'printf';
     *0: User need to register a callback with `lv_log_register_print_cb()`*/
    #define LV_LOG_PRINTF 1

    /*1: Enable print timestamp;
     *0: Disable print timestamp*/
    #define LV_LOG_USE_TIMESTAMP 1

    /*1: Print file and line number of the log;
     *0: Do not print file and line number of the log*/
    #define LV_LOG_USE_FILE_LINE 1

    /*Enable/disable LV_LOG_TRACE in modules that produces a huge number of logs*/
    #define LV_LOG_TRACE_MEM        1
    #define LV_LOG_TRACE_TIMER      1
    #define LV_LOG_TRACE_INDEV      1
    #define LV_LOG_TRACE_DISP_REFR  1
    #define LV_LOG_TRACE_EVENT      1
    #define LV_LOG_TRACE_OBJ_CREATE 1
    #define LV_LOG_TRACE_LAYOUT     1
    #define LV_LOG_TRACE_ANIM       1
    #define LV_LOG_TRACE_MSG        1

#endif  /*LV_USE_LOG*/

/*-------------
 * Asserts
 *-----------*/

/*Enable asserts if an operation is failed or an invalid data is found.
 *If LV_USE_LOG is enabled an error message will be printed on failure*/
#define LV_USE_ASSERT_NULL          1   /*Check if the parameter is NULL. (Very fast, recommended)*/
#define LV_USE_ASSERT_MALLOC        1   /*Checks is the memory is successfully allocated or no. (Very fast, recommended)*/
#define LV_USE_ASSERT_STYLE         0   /*Check if the styles are properly initialized. (Very fast, recommended)*/
#define LV_USE_ASSERT_MEM_INTEGRITY 0   /*Check the integrity of `lv_mem` after critical operations. (Slow)*/
#define LV_USE_ASSERT_OBJ           0   /*Check the object's type and existence (e.g. not deleted). (Slow)*/

/*-------------
 * Others
 *-----------*/

#define LV_USE_PROFILER    0

/*1: Show CPU usage and FPS count*/
#define LV_USE_BUILTIN_PROFILER 0
#if LV_USE_BUILTIN_PROFILER
    /*1: Show the used memory and the memory fragmentation
     * Requires `LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN`*/
    #define LV_USE_BUILTIN_PROFILER_MEM 0
#endif

/*1: Show the used memory and the memory fragmentation  in percentage ratio of total/free meter*/
#define LV_USE_MEM_MONITOR 0
#if LV_USE_MEM_MONITOR
    #define LV_MEM_MONITOR_PERIOD 1000
#endif

/*1: Draw random colored rectangles over the redrawn areas*/
#define LV_USE_REFR_DEBUG 0

/*Maximum buffer size to allocate for rotation.
 *Only used if software rotation is enabled in the display driver.*/
#define LV_DRAW_BUF_MAX_SIZE (10*1024)

#define LV_USE_USER_DATA 1

/*Place performance critical functions into a faster memory (e.g RAM)*/
#define LV_ATTRIBUTE_FAST_MEM

/*Export integer constant to binding. This macro is used with constants in the form of LV_<CONST> that
 *should also appear on LVGL binding API such as Micropython.*/
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning /*The default value just prevents GCC warning*/

/*===================
 * FONT USAGE
 *==================*/

/*Montserrat fonts with ASCII range and some symbols using bpp = 4
 *https://fonts.google.com/specimen/Montserrat*/
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 1
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

/*Demonstrate special features*/
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0  /*bpp = 3*/
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0  /*Hebrew, Arabic, Persian letters and all their forms*/
#define LV_FONT_SIMSUN_16_CJK            0  /*1000 most common CJK radicals*/

/*Pixel perfect monospace fonts*/
#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0

/*Optionally declare custom fonts here.
 *You can use these fonts as default font too and they will be available globally.
 *E.g. #define LV_FONT_CUSTOM_DECLARE   LV_FONT_DECLARE(my_font_1) LV_FONT_DECLARE(my_font_2)*/
#define LV_FONT_CUSTOM_DECLARE

/*Always set a default font*/
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/*Enable handling large font and/or fonts with a lot of characters.
 *The limit depends on the font size, font face and bpp.
 *Compiler error will be triggered if a font needs it.*/
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
 * Your IDE or editor should have the same character encoding
 * - LV_TXT_ENC_UTF8
 * - LV_TXT_ENC_ASCII
 */
#define LV_TXT_ENC LV_TXT_ENC_UTF8

/*Can break (wrap) texts on these chars*/
#define LV_TXT_BREAK_CHARS " ,.;:-_)]}"

/*If a word is at least this long, will break wherever "prettiest"
 *To disable, set to a value <= 0*/
#define LV_TXT_LINE_BREAK_LONG_LEN 0

/*Minimum number of characters in a long word to put on a line before a break.
 *Depends on LV_TXT_LINE_BREAK_LONG_LEN.*/
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3

/*Minimum number of characters in a long word to put on a line after a break.
 *Depends on LV_TXT_LINE_BREAK_LONG_LEN.*/
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

/*The control character to use for signaling text recoloring.*/
#define LV_TXT_COLOR_CMD "#"

/*Support bidirectional texts. Allows mixing Left-to-Right and Right-to-Left texts.
 *The direction will be processed according to the Unicode Bidirectional Algorithm:
 *https://www.w3.org/International/articles/inline-bidi-markup/uba-basics*/
#define LV_USE_BIDI 0
#if LV_USE_BIDI
    /*Set the default direction. Supported values:
     *`LV_BASE_DIR_LTR` Left-to-Right
     *`LV_BASE_DIR_RTL` Right-to-Left
     *`LV_BASE_DIR_AUTO` detect texts base direction*/
    #define LV_BIDI_BASE_DIR_DEF LV_BASE_DIR_AUTO
#endif

/*Enable Arabic/Persian processing
 *In these languages characters should be replaced with an other form based on their position in the text*/
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*==================
 * WIDGETS
 *================*/

/*Documentation of the widgets: https://docs.lvgl.io/latest/en/html/widgets/index.html*/

#define LV_WIDGETS_HAS_DEFAULT_VALUE  1

#define LV_USE_ANIMIMG    1

#define LV_USE_ARC        1

#define LV_USE_BAR        1

#define LV_USE_BUTTON     1

#define LV_USE_BUTTONMATRIX  1

#define LV_USE_CALENDAR   1
#if LV_USE_CALENDAR
    #define LV_CALENDAR_WEEK_STARTS_MONDAY 0
    #if LV_CALENDAR_WEEK_STARTS_MONDAY
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"}
    #else
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"}
    #endif

    #define LV_CALENDAR_DEFAULT_MONTH_NAMES {"January", "February", "March",  "April", "May",  "June", "July", "August", "September", "October", "November", "December"}
    #define LV_USE_CALENDAR_HEADER_ARROW 1
    #define LV_USE_CALENDAR_HEADER_DROPDOWN 1
#endif  /*LV_USE_CALENDAR*/

#define LV_USE_CANVAS     1

#define LV_USE_CHART      1

#define LV_USE_CHECKBOX   1

#define LV_USE_DROPDOWN   1   /*Requires: lv_label*/

#define LV_USE_IMAGE      1   /*Requires: lv_label*/

#define LV_USE_IMAGEBUTTON     1

#define LV_USE_KEYBOARD   1

#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION 1 /*Enable selecting text of the label*/
    #define LV_LABEL_LONG_TXT_HINT 1  /*Store some extra info in labels to speed up drawing of very long texts*/
    #define LV_LABEL_WAIT_CHAR_COUNT 3 /*The count of wait chart*/
#endif

#define LV_USE_LED        1

#define LV_USE_LINE       1

#define LV_USE_LIST       1

#define LV_USE_MENU       1

#define LV_USE_MSGBOX     1

#define LV_USE_ROLLER     1   /*Requires: lv_label*/

#define LV_USE_SCALE      1

#define LV_USE_SLIDER     1   /*Requires: lv_bar*/

#define LV_USE_SPAN       1
#if LV_USE_SPAN
    /*A line text can contain maximum num of span descriptor */
    #define LV_SPAN_SNIPPET_STACK_SIZE 64
#endif

#define LV_USE_SPINBOX    1

#define LV_USE_SPINNER    1

#define LV_USE_SWITCH     1

#define LV_USE_TEXTAREA   1   /*Requires: lv_label*/
#if LV_USE_TEXTAREA != 0
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500    /*ms*/
#endif

#define LV_USE_TABLE      1

#define LV_USE_TABVIEW    1

#define LV_USE_TILEVIEW   1

#define LV_USE_WIN        1

/*==================
 * THEMES
 *==================*/

/*A simple, impressive and very complete theme*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT

    /*0: Light mode; 1: Dark mode*/
    #define LV_THEME_DEFAULT_DARK 0

    /*1: Enable grow on press*/
    #define LV_THEME_DEFAULT_GROW 1

    /*Default transition time in [ms]*/
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif /*LV_USE_THEME_DEFAULT*/

/*A very simple theme that is a good starting point for a custom theme*/
#define LV_USE_THEME_SIMPLE 1

/*A theme designed for monochrome displays*/
#define LV_USE_THEME_MONO 1

/*==================
 * LAYOUTS
 *==================*/

/*A layout similar to Flexbox in CSS.*/
#define LV_USE_FLEX 1

/*A layout similar to Grid in CSS.*/
#define LV_USE_GRID 1

/*==================
 * 3RD PARTS LIBRARIES
 *==================*/

/*File system interfaces for common APIs */

/*API for fopen, fread, etc*/
#define LV_USE_FS_STDIO 0
#if LV_USE_FS_STDIO
    #define LV_FS_STDIO_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A')*/
    #define LV_FS_STDIO_PATH ""         /*Set the working directory. File/directory paths will be appended to it.*/
    #define LV_FS_STDIO_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read()*/
#endif

/*API for open, read, etc*/
#define LV_USE_FS_POSIX 0
#if LV_USE_FS_POSIX
    #define LV_FS_POSIX_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A')*/
    #define LV_FS_POSIX_PATH ""         /*Set the working directory. File/directory paths will be appended to it.*/
    #define LV_FS_POSIX_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read()*/
#endif

/*API for CreateFile, ReadFile, etc*/
#define LV_USE_FS_WIN32 0
#if LV_USE_FS_WIN32
    #define LV_FS_WIN32_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A')*/
    #define LV_FS_WIN32_PATH ""         /*Set the working directory. File/directory paths will be appended to it.*/
    #define LV_FS_WIN32_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read()*/
#endif

/*API for FATFS (needs to be added separately). Uses f_open, f_read, etc*/
#define LV_USE_FS_FATFS 0
#if LV_USE_FS_FATFS
    #define LV_FS_FATFS_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A')*/
    #define LV_FS_FATFS_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read()*/
#endif

/*LittleFS library integration*/
#define LV_USE_FS_LITTLEFS 0
#if LV_USE_FS_LITTLEFS
    #define LV_FS_LITTLEFS_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A')*/
#endif

/*==================
 * DRIVERS
 *==================*/

/*Enable the built-in display drivers*/
#define LV_USE_ST7735        1

#define LV_USE_ST7789        0

#define LV_USE_ST7796        0

#define LV_USE_ILI9341       0

#define LV_USE_TFT_ESPI      1

/*Enable the built-in input device drivers*/
#define LV_USE_EVDEV         0

#define LV_USE_LIBINPUT      0

#define LV_USE_WINDOWS       0

#endif /*LV_CONF_H*/
