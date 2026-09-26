/*******************************************************************************
 * Size: 10 px
 * Bpp: 4
 * Opts: --no-compress --no-prefilter --bpp 4 --size 10 --font Montserrat-Medium.ttf --autohint-strong -r 0x20-0x7F,0xB0,0x2022 --format lvgl --force-fast-kern-format -o montserrat_hinted_10.c
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef MONTSERRAT_HINTED_10
#define MONTSERRAT_HINTED_10 1
#endif

#if MONTSERRAT_HINTED_10

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t montserrat_hinted_10_glyph_bitmap[] = {
    /* U+0020 " " */

    /* U+0021 "!" */
    0xf, 0xf, 0xf, 0xf, 0x0, 0x0, 0x29,

    /* U+0022 "\"" */
    0xf0, 0xff, 0xf, 0xd0, 0xd0,

    /* U+0023 "#" */
    0x0, 0xb0, 0x28, 0x0, 0xb, 0x4, 0x60, 0x5c,
    0xec, 0xed, 0x90, 0x29, 0x8, 0x20, 0x9d, 0xdc,
    0xec, 0x50, 0x64, 0xb, 0x0, 0x8, 0x30, 0xb0,
    0x0,

    /* U+0024 "$" */
    0x0, 0x7, 0x0, 0x4, 0xdf, 0xc6, 0xe, 0x2f,
    0x1, 0xe, 0x5f, 0x0, 0x3, 0xbf, 0xb2, 0x0,
    0xf, 0x6d, 0x4, 0xf, 0x1e, 0x9, 0xcf, 0xc4,
    0x0, 0xf, 0x0,

    /* U+0025 "%" */
    0x7d, 0x70, 0x8, 0x30, 0xe2, 0xe0, 0x37, 0x0,
    0xe2, 0xe0, 0x90, 0x0, 0x49, 0x47, 0x59, 0x92,
    0x0, 0x25, 0xd1, 0x1d, 0x0, 0x70, 0xe1, 0x1e,
    0x5, 0x10, 0x5d, 0xd5,

    /* U+0026 "&" */
    0x6, 0xdd, 0x70, 0x0, 0xe1, 0x1e, 0x0, 0xa,
    0x8b, 0x70, 0x0, 0x9d, 0xa0, 0x10, 0xb5, 0x9,
    0xaa, 0x4e, 0x30, 0xb, 0xf0, 0x4c, 0xcc, 0x96,
    0x70, 0x0, 0x0, 0x0,

    /* U+0027 "'" */
    0xff, 0xd0,

    /* U+0028 "(" */
    0x1d, 0x7, 0x70, 0xc3, 0xe, 0x0, 0xf0, 0xe,
    0x0, 0xc3, 0x7, 0x70, 0x1d, 0x0,

    /* U+0029 ")" */
    0xd, 0x10, 0x77, 0x3, 0xc0, 0xe, 0x0, 0xf0,
    0xe, 0x3, 0xc0, 0x77, 0xd, 0x10,

    /* U+002A "*" */
    0x22, 0xf2, 0x21, 0xcf, 0xc1, 0x22, 0xf2, 0x20,

    /* U+002B "+" */
    0x0, 0xf0, 0x9, 0xcf, 0xca, 0x0, 0xf0, 0x0,
    0xf, 0x0,

    /* U+002C "," */
    0x48, 0x4a, 0x55,

    /* U+002D "-" */
    0x5c, 0xc3,

    /* U+002E "." */
    0x47,

    /* U+002F "/" */
    0x0, 0x2, 0xb0, 0x0, 0x85, 0x0, 0xd, 0x0,
    0x4, 0x90, 0x0, 0xa3, 0x0, 0xd, 0x0, 0x5,
    0x80, 0x0, 0xb2, 0x0, 0x1c, 0x0, 0x0,

    /* U+0030 "0" */
    0x8, 0xdd, 0x80, 0x89, 0x0, 0x97, 0xd1, 0x0,
    0x1d, 0xf0, 0x0, 0xf, 0xd1, 0x0, 0x1d, 0x89,
    0x0, 0x97, 0x9, 0xdd, 0x80,

    /* U+0031 "1" */
    0x7c, 0xf0, 0xf, 0x0, 0xf0, 0xf, 0x0, 0xf0,
    0xf, 0x0, 0xf0,

    /* U+0032 "2" */
    0x3b, 0xcc, 0x40, 0x32, 0x2, 0xe0, 0x0, 0x1,
    0xd0, 0x0, 0xb, 0x60, 0x0, 0xa8, 0x0, 0x9,
    0x90, 0x0, 0x6f, 0xcc, 0xc3,

    /* U+0033 "3" */
    0x1c, 0xcc, 0xea, 0x0, 0x3, 0xd1, 0x0, 0x2e,
    0x30, 0x0, 0x5b, 0xe5, 0x0, 0x0, 0x2e, 0x14,
    0x0, 0x3d, 0x2b, 0xdc, 0xc3,

    /* U+0034 "4" */
    0x0, 0xb, 0x50, 0x0, 0x9, 0x80, 0x0, 0x7,
    0xa0, 0x40, 0x5, 0xc0, 0xf, 0x0, 0xdd, 0xcc,
    0xfc, 0x30, 0x0, 0xf, 0x0, 0x0, 0x0, 0xf0,
    0x0,

    /* U+0035 "5" */
    0x6, 0xec, 0xc7, 0x7, 0x70, 0x0, 0x8, 0x60,
    0x0, 0x9, 0xdc, 0xb2, 0x0, 0x0, 0x4d, 0x4,
    0x0, 0x2e, 0x9, 0xcc, 0xc4,

    /* U+0036 "6" */
    0x9, 0xdc, 0x58, 0xa0, 0x0, 0xd2, 0x0, 0xf,
    0x6b, 0xb3, 0xd2, 0x3, 0xe8, 0x10, 0x2e, 0x19,
    0xcc, 0x30,

    /* U+0037 "7" */
    0xfc, 0xcc, 0xe7, 0xe0, 0x0, 0xe1, 0x0, 0x7,
    0xa0, 0x0, 0xe, 0x20, 0x0, 0x6b, 0x0, 0x0,
    0xd3, 0x0, 0x5, 0xc0, 0x0,

    /* U+0038 "8" */
    0x3c, 0xcc, 0x40, 0xe2, 0x2, 0xe0, 0xd3, 0x3,
    0xd0, 0x5f, 0xde, 0x70, 0xd4, 0x0, 0x4c, 0xe3,
    0x0, 0x3e, 0x3b, 0xcc, 0xb3,

    /* U+0039 "9" */
    0x3c, 0xc9, 0x1e, 0x20, 0x18, 0xe3, 0x2, 0xd3,
    0xbb, 0x7f, 0x0, 0x2, 0xd0, 0x0, 0xa7, 0x6c,
    0xd9, 0x0,

    /* U+003A ":" */
    0x47, 0x0, 0x0, 0x0, 0x47,

    /* U+003B ";" */
    0x47, 0x0, 0x0, 0x48, 0x4a, 0x55,

    /* U+003C "<" */
    0x0, 0x0, 0x10, 0x39, 0xb4, 0xd9, 0x20, 0x4,
    0xab, 0x50, 0x0, 0x16, 0x50,

    /* U+003D "=" */
    0x4c, 0xcc, 0xc1, 0x0, 0x0, 0x0, 0x4c, 0xcc,
    0xc1,

    /* U+003E ">" */
    0x20, 0x0, 0x9, 0xb6, 0x10, 0x0, 0x5d, 0x62,
    0x8b, 0x71, 0x93, 0x0, 0x0,

    /* U+003F "?" */
    0x4c, 0xcd, 0x45, 0x10, 0x2e, 0x0, 0x3, 0xd0,
    0x1, 0xd4, 0x0, 0x97, 0x0, 0x1, 0x0, 0x0,
    0x83, 0x0,

    /* U+0040 "@" */
    0x0, 0x6c, 0xcc, 0xc7, 0x0, 0xc, 0x81, 0x0,
    0x8, 0xc0, 0x89, 0x2c, 0xc8, 0xf0, 0x88, 0xd2,
    0xc4, 0x5, 0xf0, 0x1d, 0xf0, 0xf0, 0x0, 0xf0,
    0xf, 0xd1, 0xc4, 0x4, 0xf0, 0x3c, 0x88, 0x2c,
    0xb8, 0x9c, 0xb2, 0xd, 0x60, 0x0, 0x0, 0x0,
    0x0, 0x8c, 0xcc, 0x40, 0x0,

    /* U+0041 "A" */
    0x0, 0x1, 0xf6, 0x0, 0x0, 0x0, 0x88, 0xc0,
    0x0, 0x0, 0xd, 0x9, 0x40, 0x0, 0x6, 0x70,
    0x2b, 0x0, 0x0, 0xdc, 0xcc, 0xe3, 0x0, 0x59,
    0x0, 0x4, 0xa0, 0xc, 0x30, 0x0, 0xd, 0x10,

    /* U+0042 "B" */
    0xfc, 0xcc, 0xc4, 0xf0, 0x0, 0x2e, 0xf0, 0x0,
    0x3d, 0xfc, 0xcd, 0xf5, 0xf0, 0x0, 0x3d, 0xf0,
    0x0, 0x2e, 0xfc, 0xcc, 0xc4,

    /* U+0043 "C" */
    0x5, 0xcc, 0xc8, 0x5, 0xc1, 0x0, 0x60, 0xd2,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x0, 0xd2, 0x0,
    0x0, 0x5, 0xc1, 0x0, 0x60, 0x5, 0xcc, 0xc8,
    0x0,

    /* U+0044 "D" */
    0xfc, 0xcc, 0xc4, 0xf, 0x0, 0x2, 0xc5, 0xf0,
    0x0, 0x3, 0xcf, 0x0, 0x0, 0xf, 0xf0, 0x0,
    0x3, 0xcf, 0x0, 0x2, 0xc5, 0xfc, 0xcc, 0xc4,
    0x0,

    /* U+0045 "E" */
    0xfc, 0xcc, 0xb0, 0xf0, 0x0, 0x0, 0xf0, 0x0,
    0x0, 0xfc, 0xcc, 0x60, 0xf0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0xfc, 0xcc, 0xc0,

    /* U+0046 "F" */
    0xfc, 0xcc, 0xbf, 0x0, 0x0, 0xf0, 0x0, 0xf,
    0xcc, 0xc6, 0xf0, 0x0, 0xf, 0x0, 0x0, 0xf0,
    0x0, 0x0,

    /* U+0047 "G" */
    0x5, 0xcc, 0xc6, 0x5, 0xc1, 0x1, 0x50, 0xd2,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x80, 0xd2, 0x0,
    0xf, 0x5, 0xc1, 0x0, 0xf0, 0x5, 0xcc, 0xc7,
    0x0,

    /* U+0048 "H" */
    0xf0, 0x0, 0xf, 0xf0, 0x0, 0xf, 0xf0, 0x0,
    0xf, 0xfc, 0xcc, 0xcf, 0xf0, 0x0, 0xf, 0xf0,
    0x0, 0xf, 0xf0, 0x0, 0xf,

    /* U+0049 "I" */
    0xff, 0xff, 0xff, 0xf0,

    /* U+004A "J" */
    0x6, 0xcc, 0xf0, 0x0, 0xf, 0x0, 0x0, 0xf0,
    0x0, 0xf, 0x0, 0x0, 0xf0, 0x50, 0x2d, 0x9,
    0xcd, 0x40,

    /* U+004B "K" */
    0xf0, 0x0, 0xa7, 0xf, 0x0, 0xa8, 0x0, 0xf0,
    0xa8, 0x0, 0xf, 0x9f, 0x30, 0x0, 0xfa, 0x4e,
    0x20, 0xf, 0x0, 0x6d, 0x0, 0xf0, 0x0, 0x7b,
    0x0,

    /* U+004C "L" */
    0xf0, 0x0, 0xf, 0x0, 0x0, 0xf0, 0x0, 0xf,
    0x0, 0x0, 0xf0, 0x0, 0xf, 0x0, 0x0, 0xfd,
    0xdd, 0xa0,

    /* U+004D "M" */
    0xf2, 0x0, 0x2, 0xff, 0xa0, 0x0, 0xaf, 0xfb,
    0x30, 0x3a, 0xff, 0x2b, 0xb, 0x2f, 0xf0, 0x98,
    0x90, 0xff, 0x1, 0xe1, 0xf, 0xf0, 0x1, 0x0,
    0xf0,

    /* U+004E "N" */
    0xf3, 0x0, 0xf, 0xfe, 0x10, 0xf, 0xf7, 0xc0,
    0xf, 0xf0, 0xaa, 0xf, 0xf0, 0xc, 0x6f, 0xf0,
    0x1, 0xef, 0xf0, 0x0, 0x3f,

    /* U+004F "O" */
    0x5, 0xcc, 0xc5, 0x5, 0xc1, 0x1, 0xc5, 0xd2,
    0x0, 0x2, 0xdf, 0x0, 0x0, 0xf, 0xd2, 0x0,
    0x2, 0xd5, 0xc1, 0x1, 0xc5, 0x5, 0xcc, 0xc5,
    0x0,

    /* U+0050 "P" */
    0xfc, 0xcc, 0xa1, 0xf0, 0x0, 0x5b, 0xf0, 0x0,
    0xe, 0xf0, 0x0, 0x7a, 0xfc, 0xcc, 0x80, 0xf0,
    0x0, 0x0, 0xf0, 0x0, 0x0,

    /* U+0051 "Q" */
    0x5, 0xcc, 0xc5, 0x0, 0x5c, 0x10, 0x1c, 0x50,
    0xd2, 0x0, 0x2, 0xd0, 0xf0, 0x0, 0x0, 0xf0,
    0xd2, 0x0, 0x2, 0xd0, 0x6c, 0x10, 0x1c, 0x60,
    0x6, 0xdc, 0xd6, 0x0, 0x0, 0x2, 0xac, 0xb1,

    /* U+0052 "R" */
    0xfc, 0xcc, 0xa1, 0xf, 0x0, 0x5, 0xb0, 0xf0,
    0x0, 0xf, 0xf, 0x0, 0x7, 0xb0, 0xfc, 0xce,
    0xc1, 0xf, 0x0, 0x2d, 0x10, 0xf0, 0x0, 0x5b,
    0x0,

    /* U+0053 "S" */
    0x4, 0xcc, 0xc5, 0xe, 0x20, 0x1, 0xd, 0x60,
    0x0, 0x2, 0xbf, 0xb2, 0x0, 0x0, 0x7d, 0x5,
    0x0, 0x2e, 0x9, 0xcc, 0xc4,

    /* U+0054 "T" */
    0x4c, 0xcf, 0xcc, 0x40, 0x0, 0xf0, 0x0, 0x0,
    0xf, 0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf,
    0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf, 0x0,
    0x0,

    /* U+0055 "U" */
    0xf0, 0x0, 0xf, 0xf0, 0x0, 0xf, 0xf0, 0x0,
    0xf, 0xf0, 0x0, 0xf, 0xe0, 0x0, 0xe, 0x97,
    0x0, 0x79, 0x1a, 0xcc, 0xa1,

    /* U+0056 "V" */
    0xc, 0x40, 0x0, 0x1d, 0x0, 0x5b, 0x0, 0x8,
    0x70, 0x0, 0xe2, 0x0, 0xe1, 0x0, 0x7, 0x90,
    0x69, 0x0, 0x0, 0x1e, 0x1d, 0x20, 0x0, 0x0,
    0x9c, 0xb0, 0x0, 0x0, 0x2, 0xf4, 0x0, 0x0,

    /* U+0057 "W" */
    0x88, 0x0, 0xf, 0x40, 0x3, 0xc3, 0xd0, 0x5,
    0xea, 0x0, 0x86, 0xd, 0x20, 0xa4, 0xe0, 0xd,
    0x10, 0x88, 0xd, 0xa, 0x43, 0xc0, 0x2, 0xd5,
    0x90, 0x4a, 0x86, 0x0, 0xd, 0xd3, 0x0, 0xed,
    0x10, 0x0, 0x8e, 0x0, 0xa, 0xc0, 0x0,

    /* U+0058 "X" */
    0x5c, 0x0, 0x1d, 0x10, 0x98, 0xb, 0x50, 0x0,
    0xda, 0x90, 0x0, 0x6, 0xf2, 0x0, 0x1, 0xd7,
    0xc0, 0x0, 0xc5, 0xa, 0x80, 0x8a, 0x0, 0xd,
    0x30,

    /* U+0059 "Y" */
    0x79, 0x0, 0x8, 0x70, 0xd3, 0x2, 0xc0, 0x3,
    0xc0, 0xb3, 0x0, 0xa, 0xba, 0x0, 0x0, 0x1f,
    0x10, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf, 0x0,
    0x0,

    /* U+005A "Z" */
    0xbc, 0xcc, 0xf7, 0x0, 0x4, 0xc0, 0x0, 0x2e,
    0x20, 0x0, 0xc5, 0x0, 0x9, 0x90, 0x0, 0x5c,
    0x0, 0x0, 0xfd, 0xcc, 0xc7,

    /* U+005B "[" */
    0xfc, 0x1f, 0x0, 0xf0, 0xf, 0x0, 0xf0, 0xf,
    0x0, 0xf0, 0xf, 0x0, 0xfc, 0x10,

    /* U+005C "\\" */
    0x3a, 0x0, 0x0, 0xc1, 0x0, 0x7, 0x60, 0x0,
    0x1c, 0x0, 0x0, 0xb2, 0x0, 0x5, 0x80, 0x0,
    0xd, 0x0, 0x0, 0xa3, 0x0, 0x4, 0x90,

    /* U+005D "]" */
    0x1c, 0xf0, 0xf, 0x0, 0xf0, 0xf, 0x0, 0xf0,
    0xf, 0x0, 0xf0, 0xf, 0x1c, 0xf0,

    /* U+005E "^" */
    0x0, 0xa8, 0x0, 0x2, 0x9b, 0x0, 0x9, 0x25,
    0x60, 0x1b, 0x0, 0xb0,

    /* U+005F "_" */
    0xcc, 0xcc, 0xc0,

    /* U+0060 "`" */
    0x3a, 0x30,

    /* U+0061 "a" */
    0x6c, 0xcc, 0x41, 0x0, 0x2e, 0x5c, 0xcc, 0xfe,
    0x10, 0x2f, 0x7d, 0xc9, 0xe0,

    /* U+0062 "b" */
    0xf0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf8, 0xcd,
    0xa1, 0xf6, 0x0, 0x7b, 0xf0, 0x0, 0xf, 0xf6,
    0x0, 0x7b, 0xe8, 0xcd, 0xa1,

    /* U+0063 "c" */
    0x1b, 0xdd, 0x5b, 0x60, 0x4, 0xf0, 0x0, 0xb,
    0x60, 0x4, 0x1b, 0xdd, 0x50,

    /* U+0064 "d" */
    0x0, 0x0, 0xf, 0x0, 0x0, 0xf, 0x1a, 0xdc,
    0x8f, 0xb6, 0x0, 0x6f, 0xf0, 0x0, 0xf, 0xb6,
    0x0, 0x6f, 0x1a, 0xdc, 0x8e,

    /* U+0065 "e" */
    0x2c, 0xcc, 0x2c, 0x30, 0x2b, 0xfc, 0xcc, 0xdc,
    0x30, 0x1, 0x2b, 0xcc, 0x40,

    /* U+0066 "f" */
    0x7, 0xd9, 0xf, 0x0, 0xaf, 0xc6, 0xf, 0x0,
    0xf, 0x0, 0xf, 0x0, 0xf, 0x0,

    /* U+0067 "g" */
    0x1a, 0xdc, 0x8e, 0xb6, 0x0, 0x6f, 0xf0, 0x0,
    0xf, 0xb7, 0x0, 0x7f, 0x1a, 0xdc, 0x9f, 0x21,
    0x0, 0x4b, 0x4c, 0xcc, 0xb2,

    /* U+0068 "h" */
    0xf0, 0x0, 0xf, 0x0, 0x0, 0xf9, 0xcd, 0x4f,
    0x40, 0x3d, 0xf0, 0x0, 0xff, 0x0, 0xf, 0xf0,
    0x0, 0xf0,

    /* U+0069 "i" */
    0x1a, 0x0, 0x0, 0xf, 0x0, 0xf0, 0xf, 0x0,
    0xf0, 0xf, 0x0,

    /* U+006A "j" */
    0x1, 0xa0, 0x0, 0x0, 0x0, 0xf0, 0x0, 0xf0,
    0x0, 0xf0, 0x0, 0xf0, 0x0, 0xf0, 0x0, 0xf0,
    0x9d, 0x70,

    /* U+006B "k" */
    0xf0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf0, 0xa,
    0x70, 0xf1, 0xc6, 0x0, 0xfd, 0xe3, 0x0, 0xf3,
    0x3d, 0x10, 0xf0, 0x5, 0xc0,

    /* U+006C "l" */
    0xff, 0xff, 0xff, 0xf0,

    /* U+006D "m" */
    0xf9, 0xcc, 0x5a, 0xcd, 0x4f, 0x40, 0x2f, 0x40,
    0x2d, 0xf0, 0x0, 0xf0, 0x0, 0xff, 0x0, 0xf,
    0x0, 0xf, 0xf0, 0x0, 0xf0, 0x0, 0xf0,

    /* U+006E "n" */
    0xf9, 0xcd, 0x4f, 0x40, 0x3d, 0xf0, 0x0, 0xff,
    0x0, 0xf, 0xf0, 0x0, 0xf0,

    /* U+006F "o" */
    0x1a, 0xdd, 0xa1, 0xb6, 0x0, 0x6b, 0xf0, 0x0,
    0xf, 0xb6, 0x0, 0x6b, 0x1a, 0xdd, 0xa1,

    /* U+0070 "p" */
    0xe8, 0xcd, 0xa1, 0xf6, 0x0, 0x7b, 0xf0, 0x0,
    0xf, 0xf6, 0x0, 0x7b, 0xf8, 0xcd, 0xa1, 0xf0,
    0x0, 0x0, 0xf0, 0x0, 0x0,

    /* U+0071 "q" */
    0x1a, 0xdc, 0x8e, 0xb6, 0x0, 0x6f, 0xf0, 0x0,
    0xf, 0xb6, 0x0, 0x6f, 0x1a, 0xdc, 0x8f, 0x0,
    0x0, 0xf, 0x0, 0x0, 0xf,

    /* U+0072 "r" */
    0xe9, 0xaf, 0x40, 0xf0, 0xf, 0x0, 0xf0, 0x0,

    /* U+0073 "s" */
    0x1b, 0xcc, 0x76, 0xc0, 0x0, 0x19, 0xcb, 0x41,
    0x0, 0x2e, 0x5c, 0xcc, 0x60,

    /* U+0074 "t" */
    0xf, 0x0, 0xaf, 0xc6, 0xf, 0x0, 0xf, 0x0,
    0xf, 0x0, 0x8, 0xd9,

    /* U+0075 "u" */
    0xf0, 0x0, 0xff, 0x0, 0xf, 0xf0, 0x0, 0xfd,
    0x20, 0x4f, 0x4d, 0xc9, 0xe0,

    /* U+0076 "v" */
    0xc, 0x30, 0x9, 0x50, 0x5a, 0x1, 0xd0, 0x0,
    0xd2, 0x86, 0x0, 0x6, 0x9d, 0x0, 0x0, 0xe,
    0x80, 0x0,

    /* U+0077 "w" */
    0xb2, 0x1, 0xf1, 0x2, 0xb5, 0x80, 0x7b, 0x80,
    0x85, 0xd, 0xd, 0x1d, 0xd, 0x0, 0x89, 0x90,
    0x99, 0x80, 0x2, 0xf2, 0x2, 0xf2, 0x0,

    /* U+0078 "x" */
    0x5b, 0x3, 0xc0, 0x8, 0x9c, 0x10, 0x0, 0xe7,
    0x0, 0xa, 0x7c, 0x20, 0x79, 0x2, 0xd1,

    /* U+0079 "y" */
    0xc, 0x30, 0x9, 0x50, 0x5a, 0x1, 0xd0, 0x0,
    0xd2, 0x77, 0x0, 0x6, 0x9d, 0x0, 0x0, 0xe,
    0x80, 0x0, 0x0, 0xd1, 0x0, 0xc, 0xd6, 0x0,
    0x0,

    /* U+007A "z" */
    0xbc, 0xcf, 0x60, 0x6, 0xb0, 0x5, 0xc0, 0x3,
    0xc1, 0x0, 0xed, 0xcc, 0x70,

    /* U+007B "{" */
    0x9, 0xd0, 0xf0, 0xf, 0x0, 0xf0, 0xab, 0x0,
    0xf0, 0xf, 0x0, 0xf0, 0x9, 0xd0,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xff, 0xf0,

    /* U+007D "}" */
    0xd9, 0x0, 0xf0, 0xf, 0x0, 0xf0, 0xc, 0xa0,
    0xf0, 0xf, 0x0, 0xf0, 0xd9, 0x0,

    /* U+007E "~" */
    0x7c, 0x43, 0x69, 0x9, 0xc2,

    /* U+00B0 "°" */
    0x5c, 0xc5, 0xe1, 0x2e, 0x5c, 0xc5,

    /* U+2022 "•" */
    0xa8, 0xca
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t montserrat_hinted_10_glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 43, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 43, .box_w = 2, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 7, .adv_w = 63, .box_w = 3, .box_h = 3, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 12, .adv_w = 112, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 37, .adv_w = 99, .box_w = 6, .box_h = 9, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 64, .adv_w = 135, .box_w = 8, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 92, .adv_w = 110, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 120, .adv_w = 34, .box_w = 1, .box_h = 3, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 122, .adv_w = 54, .box_w = 3, .box_h = 9, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 136, .adv_w = 54, .box_w = 3, .box_h = 9, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 150, .adv_w = 64, .box_w = 5, .box_h = 3, .ofs_x = 0, .ofs_y = 4},
    {.bitmap_index = 158, .adv_w = 93, .box_w = 5, .box_h = 4, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 168, .adv_w = 36, .box_w = 2, .box_h = 3, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 171, .adv_w = 61, .box_w = 4, .box_h = 1, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 173, .adv_w = 36, .box_w = 2, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 174, .adv_w = 56, .box_w = 5, .box_h = 9, .ofs_x = -1, .ofs_y = -1},
    {.bitmap_index = 197, .adv_w = 107, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 218, .adv_w = 59, .box_w = 3, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 229, .adv_w = 92, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 250, .adv_w = 92, .box_w = 6, .box_h = 7, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 271, .adv_w = 107, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 296, .adv_w = 92, .box_w = 6, .box_h = 7, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 317, .adv_w = 99, .box_w = 5, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 335, .adv_w = 96, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 356, .adv_w = 103, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 377, .adv_w = 99, .box_w = 5, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 395, .adv_w = 36, .box_w = 2, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 400, .adv_w = 36, .box_w = 2, .box_h = 6, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 406, .adv_w = 93, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 419, .adv_w = 93, .box_w = 6, .box_h = 3, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 428, .adv_w = 93, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 441, .adv_w = 92, .box_w = 5, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 459, .adv_w = 165, .box_w = 10, .box_h = 9, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 504, .adv_w = 117, .box_w = 9, .box_h = 7, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 536, .adv_w = 121, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 557, .adv_w = 116, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 582, .adv_w = 132, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 607, .adv_w = 107, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 628, .adv_w = 102, .box_w = 5, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 646, .adv_w = 124, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 671, .adv_w = 130, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 692, .adv_w = 50, .box_w = 1, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 696, .adv_w = 82, .box_w = 5, .box_h = 7, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 714, .adv_w = 115, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 739, .adv_w = 95, .box_w = 5, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 757, .adv_w = 153, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 782, .adv_w = 130, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 803, .adv_w = 134, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 828, .adv_w = 116, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 849, .adv_w = 134, .box_w = 8, .box_h = 8, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 881, .adv_w = 116, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 906, .adv_w = 99, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 927, .adv_w = 94, .box_w = 7, .box_h = 7, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 952, .adv_w = 127, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 973, .adv_w = 114, .box_w = 9, .box_h = 7, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 1005, .adv_w = 180, .box_w = 11, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1044, .adv_w = 108, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1069, .adv_w = 104, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1094, .adv_w = 105, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1115, .adv_w = 53, .box_w = 3, .box_h = 9, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 1129, .adv_w = 56, .box_w = 5, .box_h = 9, .ofs_x = -1, .ofs_y = -1},
    {.bitmap_index = 1152, .adv_w = 53, .box_w = 3, .box_h = 9, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 1166, .adv_w = 93, .box_w = 6, .box_h = 4, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 1178, .adv_w = 80, .box_w = 5, .box_h = 1, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 1181, .adv_w = 96, .box_w = 3, .box_h = 1, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 1183, .adv_w = 96, .box_w = 5, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1196, .adv_w = 109, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1217, .adv_w = 91, .box_w = 5, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1230, .adv_w = 109, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1251, .adv_w = 98, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1264, .adv_w = 56, .box_w = 4, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1278, .adv_w = 110, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1299, .adv_w = 109, .box_w = 5, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1317, .adv_w = 45, .box_w = 3, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1328, .adv_w = 45, .box_w = 4, .box_h = 9, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 1346, .adv_w = 99, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1367, .adv_w = 45, .box_w = 1, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1371, .adv_w = 169, .box_w = 9, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1394, .adv_w = 109, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1407, .adv_w = 102, .box_w = 6, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1422, .adv_w = 109, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 1443, .adv_w = 109, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1464, .adv_w = 66, .box_w = 3, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1472, .adv_w = 80, .box_w = 5, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1485, .adv_w = 66, .box_w = 4, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1497, .adv_w = 108, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1510, .adv_w = 89, .box_w = 7, .box_h = 5, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 1528, .adv_w = 144, .box_w = 9, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1551, .adv_w = 88, .box_w = 6, .box_h = 5, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1566, .adv_w = 89, .box_w = 7, .box_h = 7, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 1591, .adv_w = 83, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1604, .adv_w = 56, .box_w = 3, .box_h = 9, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1618, .adv_w = 48, .box_w = 1, .box_h = 9, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 1623, .adv_w = 56, .box_w = 3, .box_h = 9, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1637, .adv_w = 93, .box_w = 5, .box_h = 2, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 1642, .adv_w = 67, .box_w = 4, .box_h = 3, .ofs_x = 0, .ofs_y = 3},
    {.bitmap_index = 1648, .adv_w = 50, .box_w = 2, .box_h = 2, .ofs_x = 1, .ofs_y = 2}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t montserrat_hinted_10_unicode_list_1[] = {
    0x0, 0x1f72
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t montserrat_hinted_10_cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    },
    {
        .range_start = 176, .range_length = 8051, .glyph_id_start = 96,
        .unicode_list = montserrat_hinted_10_unicode_list_1, .glyph_id_ofs_list = NULL, .list_length = 2, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t montserrat_hinted_10_kern_left_class_mapping[] =
{
    0, 0, 1, 2, 0, 3, 4, 5,
    2, 6, 7, 8, 9, 10, 9, 10,
    11, 12, 0, 13, 14, 15, 16, 17,
    18, 19, 12, 20, 20, 0, 0, 0,
    21, 22, 23, 24, 25, 22, 26, 27,
    28, 29, 29, 30, 31, 32, 29, 29,
    22, 33, 34, 35, 3, 36, 30, 37,
    37, 38, 39, 40, 41, 42, 43, 0,
    44, 0, 45, 46, 47, 48, 49, 50,
    51, 45, 52, 52, 53, 48, 45, 45,
    46, 46, 54, 55, 56, 57, 51, 58,
    58, 59, 58, 60, 41, 0, 0, 9,
    61, 9
};

/*Map glyph_ids to kern right classes*/
static const uint8_t montserrat_hinted_10_kern_right_class_mapping[] =
{
    0, 0, 1, 2, 0, 3, 4, 5,
    2, 6, 7, 8, 9, 10, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 12,
    18, 19, 20, 21, 21, 0, 0, 0,
    22, 23, 24, 25, 23, 25, 25, 25,
    23, 25, 25, 26, 25, 25, 25, 25,
    23, 25, 23, 25, 3, 27, 28, 29,
    29, 30, 31, 32, 33, 34, 35, 0,
    36, 0, 37, 38, 39, 39, 39, 0,
    39, 38, 40, 41, 38, 38, 42, 42,
    39, 42, 39, 42, 43, 44, 45, 46,
    46, 47, 46, 48, 0, 0, 35, 9,
    49, 9
};

/*Kern values between classes*/
static const int8_t montserrat_hinted_10_kern_class_values[] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 7, 0, 4, -4, 0, 0,
    0, 0, -9, -10, 1, 8, 4, 3,
    -6, 1, 8, 0, 7, 2, 5, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 10, 1, -1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 0, -5, 0, 0, 0, 0,
    0, -3, 3, 3, 0, 0, -2, 0,
    -1, 2, 0, -2, 0, -2, -1, -3,
    0, 0, 0, 0, -2, 0, 0, -2,
    -2, 0, 0, -2, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    -2, 0, -2, 0, -4, 0, -19, 0,
    0, -3, 0, 3, 5, 0, 0, -3,
    2, 2, 5, 3, -3, 3, 0, 0,
    -9, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, -2, -8, 0, -6,
    -1, 0, 0, 0, 0, 0, 6, 0,
    -5, -1, 0, 0, 0, -3, 0, 0,
    -1, -12, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -13, -1, 6,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 5,
    0, 2, 0, 0, -3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 6, 1,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -6, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1,
    3, 2, 5, -2, 0, 0, 3, -2,
    -5, -22, 1, 4, 3, 0, -2, 0,
    6, 0, 5, 0, 5, 0, -15, 0,
    -2, 5, 0, 5, -2, 3, 2, 0,
    0, 0, -2, 0, 0, -3, 13, 0,
    13, 0, 5, 0, 7, 2, 3, 5,
    0, 0, 0, -6, 0, 0, 0, 0,
    0, -1, 0, 1, -3, -2, -3, 1,
    0, -2, 0, 0, 0, -6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -10, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -9, 0, -10, 0, 0, 0,
    0, -1, 0, 16, -2, -2, 2, 2,
    -1, 0, -2, 2, 0, 0, -8, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -16, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -10, 0, 10, 0, 0, -6, 0,
    5, 0, -11, -16, -11, -3, 5, 0,
    0, -11, 0, 2, -4, 0, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 4, 5, -20, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 8, 0, 1, 0, 0, 0,
    0, 0, 1, 1, -2, -3, 0, 0,
    0, -2, 0, 0, -1, 0, 0, 0,
    -3, 0, -1, 0, -4, -3, 0, -4,
    -5, -5, -3, 0, -3, 0, -3, 0,
    0, 0, 0, -1, 0, 0, 2, 0,
    1, -2, 0, 0, 0, 0, 0, 2,
    -1, 0, 0, 0, -1, 2, 2, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 2, -1, 0,
    -2, 0, -3, 0, 0, -1, 0, 5,
    0, 0, -2, 0, 0, 0, 0, 0,
    0, 0, -1, -1, 0, 0, -2, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, -1, 0, -2, -2, 0,
    0, 0, 0, 0, 0, 0, 0, -1,
    0, -2, -2, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, -1, -2, 0, -2, 0, -5,
    -1, -5, 3, 0, 0, -3, 2, 3,
    4, 0, -4, 0, -2, 0, 0, -8,
    2, -1, 1, -8, 2, 0, 0, 0,
    -8, 0, -8, -1, -14, -1, 0, -8,
    0, 3, 4, 0, 2, 0, 0, 0,
    0, 0, 0, -3, -2, 0, -5, 0,
    0, 0, -2, 0, 0, 0, -2, 0,
    0, 0, 0, 0, -1, -1, 0, -1,
    -2, 0, 0, 0, 0, 0, 0, 0,
    -2, -2, 0, -1, -2, -1, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, -1, 0, -2,
    0, -1, 0, -3, 2, 0, 0, -2,
    1, 2, 2, 0, 0, 0, 0, 0,
    0, -1, 0, 0, 0, 0, 0, 1,
    0, 0, -2, 0, -2, -1, -2, 0,
    0, 0, 0, 0, 0, 0, 1, 0,
    -1, 0, 0, 0, 0, -2, -2, 0,
    -3, 0, 5, -1, 0, -5, 0, 0,
    4, -8, -8, -7, -3, 2, 0, -1,
    -10, -3, 0, -3, 0, -3, 2, -3,
    -10, 0, -4, 0, 0, 1, 0, 1,
    -1, 0, 2, 0, -5, -6, 0, -8,
    -4, -3, -4, -5, -2, -4, 0, -3,
    -4, 1, 0, 0, 0, -2, 0, 0,
    0, 1, 0, 2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    0, -1, 0, 0, -2, 0, -3, -4,
    -4, 0, 0, -5, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 1,
    -1, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 8, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0,
    -3, 0, 0, 0, 0, -8, -5, 0,
    0, 0, -2, -8, 0, 0, -2, 2,
    0, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, -3,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, -3, 0,
    0, 0, 0, 2, 0, 1, -3, -3,
    0, -2, -2, -2, 0, 0, 0, 0,
    0, 0, -5, 0, -2, 0, -2, -2,
    0, -4, -4, -5, -1, 0, -3, 0,
    -5, 0, 0, 0, 0, 13, 0, 0,
    1, 0, 0, -2, 0, 2, 0, -7,
    0, 0, 0, 0, 0, -15, -3, 5,
    5, -1, -7, 0, 2, -2, 0, -8,
    -1, -2, 2, -11, -2, 2, 0, 2,
    -6, -2, -6, -5, -7, 0, 0, -10,
    0, 9, 0, 0, -1, 0, 0, 0,
    -1, -1, -2, -4, -5, 0, -15, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, -1, -2, -2, 0, 0,
    -3, 0, -2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 3,
    0, 2, 0, -4, 2, -1, 0, -4,
    -2, 0, -2, -2, -1, 0, -2, -3,
    0, 0, -1, 0, -1, -3, -2, 0,
    0, -2, 0, 2, -1, 0, -4, 0,
    0, 0, -3, 0, -3, 0, -3, -3,
    2, 0, 0, 0, 0, 0, 0, 0,
    0, -3, 2, 0, -2, 0, -1, -2,
    -5, -1, -1, -1, 0, -1, -2, 0,
    0, 0, 0, 0, 0, -2, -1, -1,
    0, 0, 0, 0, 2, -1, 0, -1,
    0, 0, 0, -1, -2, -1, -1, -2,
    -1, 0, 1, 6, 0, 0, -4, 0,
    -1, 3, 0, -2, -7, -2, 2, 0,
    0, -8, -3, 2, -3, 1, 0, -1,
    -1, -5, 0, -2, 1, 0, 0, -3,
    0, 0, 0, 2, 2, -3, -3, 0,
    -3, -2, -2, -2, -2, 0, -3, 1,
    -3, -3, 5, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, -2,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -2, 0, 0, -2,
    0, 0, -2, -2, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, -1, 0, 0, 0, 0,
    -2, 0, -3, 0, 0, 0, -5, 0,
    1, -4, 3, 0, -1, -8, 0, 0,
    -4, -2, 0, -6, -4, -4, 0, 0,
    -7, -2, -6, -6, -8, 0, -4, 0,
    1, 11, -2, 0, -4, -2, 0, -2,
    -3, -4, -3, -6, -7, -4, -2, 0,
    0, -1, 0, 0, 0, 0, -11, -1,
    5, 4, -4, -6, 0, 0, -5, 0,
    -8, -1, -2, 3, -15, -2, 0, 0,
    0, -10, -2, -8, -2, -12, 0, 0,
    -11, 0, 9, 0, 0, -1, 0, 0,
    0, 0, -1, -1, -6, -1, 0, -10,
    0, 0, 0, 0, -5, 0, -1, 0,
    0, -4, -8, 0, 0, -1, -2, -5,
    -2, 0, -1, 0, 0, 0, 0, -7,
    -2, -5, -5, -1, -3, -4, -2, -3,
    0, -3, -1, -5, -2, 0, -2, -3,
    -2, -3, 0, 1, 0, -1, -5, 0,
    3, 0, -3, 0, 0, 0, 0, 2,
    0, 1, -3, 7, 0, -2, -2, -2,
    0, 0, 0, 0, 0, 0, -5, 0,
    -2, 0, -2, -2, 0, -4, -4, -5,
    -1, 0, -3, 1, 6, 0, 0, 0,
    0, 13, 0, 0, 1, 0, 0, -2,
    0, 2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -1, -3, 0, 0, 0, 0, 0, -1,
    0, 0, 0, -2, -2, 0, 0, -3,
    -2, 0, 0, -3, 0, 3, -1, 0,
    0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 2, 3, 1, -1, 0, -5,
    -3, 0, 5, -5, -5, -3, -3, 6,
    3, 2, -14, -1, 3, -2, 0, -2,
    2, -2, -6, 0, -2, 2, -2, -1,
    -5, -1, 0, 0, 5, 3, 0, -4,
    0, -9, -2, 5, -2, -6, 0, -2,
    -5, -5, -2, 6, 2, 0, -2, 0,
    -4, 0, 1, 5, -4, -6, -6, -4,
    5, 0, 0, -12, -1, 2, -3, -1,
    -4, 0, -4, -6, -2, -2, -1, 0,
    0, -4, -3, -2, 0, 5, 4, -2,
    -9, 0, -9, -2, 0, -6, -9, 0,
    -5, -3, -5, -4, 4, 0, 0, -2,
    0, -3, -1, 0, -2, -3, 0, 3,
    -5, 2, 0, 0, -8, 0, -2, -4,
    -3, -1, -5, -4, -5, -4, 0, -5,
    -2, -4, -3, -5, -2, 0, 0, 0,
    8, -3, 0, -5, -2, 0, -2, -3,
    -4, -4, -4, -6, -2, -3, 3, 0,
    -2, 0, -8, -2, 1, 3, -5, -6,
    -3, -5, 5, -2, 1, -15, -3, 3,
    -4, -3, -6, 0, -5, -7, -2, -2,
    -1, -2, -3, -5, 0, 0, 0, 5,
    4, -1, -10, 0, -10, -4, 4, -6,
    -11, -3, -6, -7, -8, -5, 3, 0,
    0, 0, 0, -2, 0, 0, 2, -2,
    3, 1, -3, 3, 0, 0, -5, 0,
    0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, -2, 0, 0, 0,
    0, 1, 5, 0, 0, -2, 0, 0,
    0, 0, -1, -1, -2, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 1, 0,
    -1, 0, 6, 0, 3, 0, 0, -2,
    0, 3, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 4, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -10, 0, -2, 3, 0, 5,
    0, 0, 16, 2, -3, -3, 2, 2,
    -1, 0, -8, 0, 0, 8, -10, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -11, 6, 22, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -10, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, -3,
    -1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, -4, 0,
    0, 0, 0, 0, 2, 21, -3, -1,
    5, 4, -4, 2, 0, 0, 2, 2,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -21, 4, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -4,
    0, 0, 0, -4, 0, 0, 0, 0,
    -4, -1, 0, 0, 0, -4, 0, -2,
    0, -8, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -11, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, 0, -3, 0, -2, 0,
    -4, 0, 0, 0, -3, 2, -2, 0,
    0, -4, -2, -4, 0, 0, -4, 0,
    -2, 0, -8, 0, -2, 0, 0, -13,
    -3, -6, -2, -6, 0, 0, -11, 0,
    -4, -1, 0, 0, 0, 0, 0, 0,
    0, 0, -2, -3, -1, -3, 0, 0,
    0, 0, -4, 0, -4, 2, -2, 3,
    0, -1, -4, -1, -3, -3, 0, -2,
    -1, -1, 1, -4, 0, 0, 0, 0,
    -14, -1, -2, 0, -4, 0, -1, -8,
    -1, 0, 0, -1, -1, 0, 0, 0,
    0, 1, 0, -1, -3, -1, 3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0,
    0, -4, 0, -1, 0, 0, 0, -3,
    2, 0, 0, 0, -4, -2, -3, 0,
    0, -4, 0, -2, 0, -8, 0, 0,
    0, 0, -16, 0, -3, -6, -8, 0,
    0, -11, 0, -1, -2, 0, 0, 0,
    0, 0, 0, 0, 0, -2, -2, -1,
    -2, 0, 0, 0, 3, -2, 0, 5,
    8, -2, -2, -5, 2, 8, 3, 4,
    -4, 2, 7, 2, 5, 4, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 10, 8, -3, -2, 0, -1,
    13, 7, 13, 0, 0, 0, 2, 0,
    0, 6, 0, 0, -3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0,
    0, 0, 0, 0, 0, 0, 0, 2,
    0, 0, 0, 0, -13, -2, -1, -7,
    -8, 0, 0, -11, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0,
    2, 0, 0, 0, 0, -13, -2, -1,
    -7, -8, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, -4, 2, 0, -2,
    1, 3, 2, -5, 0, 0, -1, 2,
    0, 1, 0, 0, 0, 0, -4, 0,
    -1, -1, -3, 0, -1, -6, 0, 10,
    -2, 0, -4, -1, 0, -1, -3, 0,
    -2, -4, -3, -2, 0, 0, 0, -3,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, -13,
    -2, -1, -7, -8, 0, 0, -11, 0,
    0, 0, 0, 0, 0, 8, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, -5, -2, -1, 5, -1, -2,
    -6, 0, -1, 0, -1, -4, 0, 4,
    0, 1, 0, 1, -4, -6, -2, 0,
    -6, -3, -4, -7, -6, 0, -3, -3,
    -2, -2, -1, -1, -2, -1, 0, -1,
    0, 2, 0, 2, -1, 0, 5, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -1, -2, -2, 0, 0,
    -4, 0, -1, 0, -3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -10, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -2, -2, 0, -2,
    0, 0, 0, 0, -1, 0, 0, -3,
    -2, 2, 0, -3, -3, -1, 0, -5,
    -1, -4, -1, -2, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -11, 0, 5, 0, 0, -3, 0,
    0, 0, 0, -2, 0, -2, 0, 0,
    -1, 0, 0, -1, 0, -4, 0, 0,
    7, -2, -5, -5, 1, 2, 2, 0,
    -4, 1, 2, 1, 5, 1, 5, -1,
    -4, 0, 0, -6, 0, 0, -5, -4,
    0, 0, -3, 0, -2, -3, 0, -2,
    0, -2, 0, -1, 2, 0, -1, -5,
    -2, 6, 0, 0, -1, 0, -3, 0,
    0, 2, -4, 0, 2, -2, 1, 0,
    0, -5, 0, -1, 0, 0, -2, 2,
    -1, 0, 0, 0, -7, -2, -4, 0,
    -5, 0, 0, -8, 0, 6, -2, 0,
    -3, 0, 1, 0, -2, 0, -2, -5,
    0, -2, 2, 0, 0, 0, 0, -1,
    0, 0, 2, -2, 0, 0, 0, -2,
    -1, 0, -2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -10, 0, 4, 0,
    0, -1, 0, 0, 0, 0, 0, 0,
    -2, -2, 0, 0, 0, 3, 0, 4,
    0, 0, 0, 0, 0, -10, -9, 0,
    7, 5, 3, -6, 1, 7, 0, 6,
    0, 3, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 8, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t montserrat_hinted_10_kern_classes =
{
    .class_pair_values   = montserrat_hinted_10_kern_class_values,
    .left_class_mapping  = montserrat_hinted_10_kern_left_class_mapping,
    .right_class_mapping = montserrat_hinted_10_kern_right_class_mapping,
    .left_class_cnt      = 61,
    .right_class_cnt     = 49,
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t montserrat_hinted_10_cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t montserrat_hinted_10_font_dsc = {
#else
static lv_font_fmt_txt_dsc_t montserrat_hinted_10_font_dsc = {
#endif
    .glyph_bitmap = montserrat_hinted_10_glyph_bitmap,
    .glyph_dsc = montserrat_hinted_10_glyph_dsc,
    .cmaps = montserrat_hinted_10_cmaps,
    .kern_dsc = &montserrat_hinted_10_kern_classes,
    .kern_scale = 16,
    .cmap_num = 2,
    .bpp = 4,
    .kern_classes = 1,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &montserrat_hinted_10_cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t montserrat_hinted_10 = {
#else
lv_font_t montserrat_hinted_10 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 10,          /*The maximum line height required by the font*/
    .base_line = 2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &montserrat_hinted_10_font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if MONTSERRAT_HINTED_10*/

