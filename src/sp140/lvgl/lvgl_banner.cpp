#include "../../../inc/sp140/lvgl/lvgl_banner.h"
#include "../../../inc/sp140/lvgl/lvgl_main_screen.h"

static lv_obj_t* bannerOverlay = NULL;
static lv_obj_t* bannerLabel = NULL;
static lv_timer_t* bannerTimer = NULL;
static bool bannerActive = false;

static void hideBanner() {
  if (bannerOverlay != NULL) {
    lv_obj_add_flag(bannerOverlay, LV_OBJ_FLAG_HIDDEN);
  }
  bannerActive = false;
}

static void bannerTimerCb(lv_timer_t* timer) {
  (void)timer;
  hideBanner();

  if (bannerTimer != NULL) {
    lv_timer_del(bannerTimer);
    bannerTimer = NULL;
  }
}

void setupBannerOverlay(bool darkMode) {
  if (main_screen == NULL || bannerOverlay != NULL) {
    return;
  }

  bannerOverlay = lv_obj_create(main_screen);
  lv_obj_set_size(bannerOverlay, SCREEN_WIDTH, 40);
  lv_obj_set_pos(bannerOverlay, 0, 0);
  lv_obj_set_style_radius(bannerOverlay, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(bannerOverlay, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(
    bannerOverlay,
    darkMode ? lv_color_make(180, 90, 0) : lv_color_make(230, 120, 0),
    LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bannerOverlay, LV_OPA_COVER, LV_PART_MAIN);

  bannerLabel = lv_label_create(bannerOverlay);
  lv_obj_set_style_text_font(bannerLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(bannerLabel, lv_color_white(), 0);
  lv_obj_set_style_text_align(bannerLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_width(bannerLabel, SCREEN_WIDTH - 8);
  lv_obj_align(bannerLabel, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(bannerLabel, "");

  lv_obj_add_flag(bannerOverlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(bannerOverlay);
}

void showBanner(const char* text, BannerColor color, uint16_t durationMs) {
  if (bannerOverlay == NULL || bannerLabel == NULL) {
    return;
  }

  lv_color_t bgColor = (color == BannerColor::CAUTION)
    ? lv_color_make(210, 105, 0)
    : lv_color_make(0, 110, 200);
  lv_obj_set_style_bg_color(bannerOverlay, bgColor, LV_PART_MAIN);
  lv_label_set_text(bannerLabel, text);
  lv_obj_clear_flag(bannerOverlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(bannerOverlay);
  bannerActive = true;

  if (bannerTimer != NULL) {
    lv_timer_del(bannerTimer);
    bannerTimer = NULL;
  }

  const uint16_t duration = (durationMs > 0) ? durationMs : 3000;
  bannerTimer = lv_timer_create(bannerTimerCb, duration, NULL);
  if (bannerTimer == NULL) {
    hideBanner();
  }
}

bool isBannerActive() {
  return bannerActive;
}
