/* Generic LVGL ABI adapter. Screen state, animation values and timing stay Rust-owned. */
#include <lvgl.h>
#include <stddef.h>

size_t eppg_lvgl_size(unsigned type) {
    switch (type) {
    case 0: return sizeof(lv_style_t);
    case 1: return sizeof(lv_point_precise_t);
    case 2: return sizeof(lv_area_t);
    case 3: return sizeof(lv_color_t);
    default: return sizeof(lv_anim_t);
    }
}
size_t eppg_lvgl_align(unsigned type) {
    switch (type) {
    case 0: return _Alignof(lv_style_t);
    case 1: return _Alignof(lv_point_precise_t);
    case 2: return _Alignof(lv_area_t);
    case 3: return _Alignof(lv_color_t);
    default: return _Alignof(lv_anim_t);
    }
}
/* LVGL copies an animation descriptor in lv_anim_start. Keep its bitfield-rich
 * descriptor on the C side of the ABI; no C function writes a Rust stack frame. */
void eppg_lvgl_start_animation(void *object, int32_t start, int32_t end,
                               uint32_t duration, lv_anim_exec_xcb_t callback) {
    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, object);
    lv_anim_set_values(&animation, start, end);
    lv_anim_set_duration(&animation, duration);
    lv_anim_set_exec_cb(&animation, callback);
    lv_anim_start(&animation);
}
