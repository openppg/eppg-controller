#include "sp140/lvgl/lvgl_splash.h"

#include <cstdio>

#include "version.h"

lv_obj_t* createLvglSplashScreen(const STR_DEVICE_DATA_140_V1& deviceData) {
  lv_obj_t* splash_screen = lv_obj_create(NULL);
  lv_screen_load(splash_screen);
  lv_obj_remove_flag(splash_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(
      splash_screen,
      deviceData.theme == 1 ? lv_color_black() : lv_color_white(),
      LV_PART_MAIN);

  lv_obj_t* title_label = lv_label_create(splash_screen);
  lv_label_set_text(title_label, "OpenPPG");
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(
      title_label,
      deviceData.theme == 1 ? lv_color_white() : lv_color_black(), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 15);

  lv_anim_t title_anim;
  lv_anim_init(&title_anim);
  lv_anim_set_var(&title_anim, title_label);
  lv_anim_set_values(&title_anim, 0, 255);
  lv_anim_set_time(&title_anim, 500);
  lv_anim_set_exec_cb(&title_anim, [](void* var, int32_t value) {
    lv_obj_set_style_opa((lv_obj_t*)var, value, 0);  // NOLINT(readability/casting)
  });
  lv_anim_start(&title_anim);

  lv_obj_t* version_label = lv_label_create(splash_screen);
  char version_str[10];
  snprintf(version_str, sizeof(version_str), "v%d.%d", VERSION_MAJOR,
           VERSION_MINOR);
  lv_label_set_text(version_label, version_str);
  lv_obj_set_style_text_font(version_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(
      version_label,
      deviceData.theme == 1 ? lv_color_white() : lv_color_black(), 0);
  lv_obj_align(version_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t* time_label = lv_label_create(splash_screen);
  char time_str[10];
  const int hours = deviceData.armed_time / 60;
  const int minutes = deviceData.armed_time % 60;
  snprintf(time_str, sizeof(time_str), "%02d:%02d", hours, minutes);
  lv_label_set_text(time_label, time_str);
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(
      time_label,
      deviceData.theme == 1 ? lv_color_white() : lv_color_black(), 0);
  lv_obj_align(time_label, LV_ALIGN_BOTTOM_MID, 0, -20);

  return splash_screen;
}
