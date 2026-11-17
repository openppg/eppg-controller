#include "../../../inc/sp140/lvgl/lvgl_climb_efficiency.h"
#include "../../../inc/sp140/lvgl/lvgl_main_screen.h"

#include <math.h>

// ---------------------------------------------------------------------------
// Power bar along the bottom of the power row (y 37..70, x 0..102), under the
// kW digits:
//
//   [------|=|------|=========|==|====]
//         ^ ^ ^     green       yellow red
//         | pink tick: level-flight power, with its certainty margin
//                  ^ white/black tick = current power
//
// green  = within 5 % of the best climb per Wh you can sustain
// yellow = holding this power would reach a warning temperature
// red    = holding this power would reach a critical temperature
//
// Nothing is drawn until the estimator has learned something, so an early
// flight shows the normal screen.
// ---------------------------------------------------------------------------
static const int kBarX = 3;
static const int kBarY = 65;
static const int kBarW = 96;
static const int kBarH = 3;
static const int kPowerTickW = 2;
static const int kPowerTickH = 7;
static const int kLevelTickW = 3;
static const int kLevelTickH = 5;
// Full scale = SP140 peak power (the band is capped there too): 6 px per kW,
// so even the settled ~0.3 kW level margin is visible.
static const float kBarMaxKw = 16.0f;

static bool barEnabled = CLIMB_EFF_BAR_DEFAULT_ENABLED;

static lv_obj_t* bar_track = NULL;
static lv_obj_t* bar_band = NULL;
static lv_obj_t* bar_yellow = NULL;
static lv_obj_t* bar_red = NULL;
static lv_obj_t* margin_left = NULL;
static lv_obj_t* margin_right = NULL;
static lv_obj_t* bar_level = NULL;
static lv_obj_t* bar_power = NULL;

void setClimbEffBarEnabled(bool enabled) { barEnabled = enabled; }
bool getClimbEffBarEnabled() { return barEnabled; }

static lv_obj_t* createRect(lv_obj_t* parent, int x, int y, int w, int h,
                            lv_color_t color) {
  lv_obj_t* obj = lv_obj_create(parent);
  lv_obj_set_pos(obj, x, y);
  lv_obj_set_size(obj, w, h);
  lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_outline_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(obj, color, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_100, LV_PART_MAIN);
  lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  return obj;
}

void clearClimbEfficiencyWidgets() {
  bar_track = NULL;
  bar_band = NULL;
  bar_yellow = NULL;
  bar_red = NULL;
  margin_left = NULL;
  margin_right = NULL;
  bar_level = NULL;
  bar_power = NULL;
}

void setupClimbEfficiencyWidgets(lv_obj_t* parent, bool darkMode) {
  clearClimbEfficiencyWidgets();
  const lv_color_t pink =
      darkMode ? lv_color_make(255, 80, 190) : lv_color_make(220, 0, 140);

  // Created back to front: track, zones, margin, level tick, power tick.
  bar_track = createRect(parent, kBarX, kBarY, kBarW, kBarH,
                         darkMode ? lv_color_make(64, 64, 64)
                                  : lv_color_make(200, 200, 200));
  bar_band = createRect(parent, kBarX, kBarY, 1, kBarH,
                        darkMode ? LVGL_DARK_GREEN : lv_color_make(0, 170, 0));
  bar_yellow = createRect(parent, kBarX, kBarY, 1, kBarH,
                          darkMode ? lv_color_make(210, 170, 0)
                                   : lv_color_make(235, 185, 0));
  bar_red = createRect(parent, kBarX, kBarY, 1, kBarH,
                       darkMode ? lv_color_make(200, 30, 40)
                                : lv_color_make(225, 0, 0));
  margin_left = createRect(parent, kBarX, kBarY - 1, 1, kLevelTickH, pink);
  margin_right = createRect(parent, kBarX, kBarY - 1, 1, kLevelTickH, pink);
  bar_level = createRect(parent, kBarX, kBarY - (kLevelTickH - kBarH) / 2,
                         kLevelTickW, kLevelTickH, pink);
  bar_power = createRect(parent, kBarX, kBarY - (kPowerTickH - kBarH) / 2,
                         kPowerTickW, kPowerTickH,
                         darkMode ? LVGL_WHITE : LVGL_BLACK);
}

static void setHidden(lv_obj_t* obj, bool hidden) {
  if (obj == NULL) return;
  if (lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN) == hidden) return;
  if (hidden) {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
}

static void setGeometry(lv_obj_t* obj, int x, int y, int w, int h) {
  if (obj == NULL) return;
  // lv_obj_set_pos/size are change-detected internally.
  lv_obj_set_pos(obj, x, y);
  lv_obj_set_size(obj, w, h);
}

static float powerToBarXf(float kw) {
  if (kw < 0.0f) kw = 0.0f;
  if (kw > kBarMaxKw) kw = kBarMaxKw;
  return kBarX + kw / kBarMaxKw * (kBarW - 1);
}

static int powerToBarX(float kw) {
  return static_cast<int>(powerToBarXf(kw) + 0.5f);
}

// Certainty brackets: thin pink ticks at L +/- margin, the height of the
// level tick. They close in as the estimate firms up and, once within ~2 px,
// merge with the tick into one slightly wider block.
static void updateMargin(float levelKw, float marginKw, bool show) {
  setHidden(margin_left, !show);
  setHidden(margin_right, !show);
  if (!show) return;
  const int center = powerToBarX(levelKw);
  // Half-width in pixels, never inside the 3 px tick itself.
  int half = static_cast<int>(marginKw / kBarMaxKw * (kBarW - 1) + 0.5f);
  if (half < 2) half = 2;
  setGeometry(margin_left, center - half, kBarY - 1, 1, kLevelTickH);
  setGeometry(margin_right, center + half, kBarY - 1, 1, kLevelTickH);
}

static void placeZone(lv_obj_t* obj, bool on, float lowKw, float highKw) {
  setHidden(obj, !on);
  if (!on) return;
  const int x0 = powerToBarX(lowKw);
  const int x1 = powerToBarX(highKw);
  setGeometry(obj, x0, kBarY, x1 > x0 ? x1 - x0 + 1 : 1, kBarH);
}

void updateClimbEfficiencyDisplay(const ClimbEfficiencyResult& r,
                                  const ClimbBarZones& zones, float powerKw,
                                  bool armed) {
  const bool show = barEnabled && armed &&
                    (r.levelValid || zones.green || zones.yellow || zones.red);
  setHidden(bar_track, !show);
  setHidden(bar_power, !show);
  setHidden(bar_level, !(show && r.levelValid));
  placeZone(bar_band, show && zones.green, zones.greenLowKw, zones.greenHighKw);
  placeZone(bar_yellow, show && zones.yellow, zones.yellowLowKw, zones.yellowHighKw);
  placeZone(bar_red, show && zones.red, zones.redLowKw, zones.redHighKw);
  updateMargin(r.levelPowerKw, r.levelMarginKw,
               show && r.levelValid && !isnan(r.levelMarginKw));
  if (!show) return;

  if (r.levelValid) {
    setGeometry(bar_level, powerToBarX(r.levelPowerKw) - kLevelTickW / 2,
                kBarY - (kLevelTickH - kBarH) / 2, kLevelTickW, kLevelTickH);
  }
  setGeometry(bar_power, powerToBarX(powerKw) - kPowerTickW / 2,
              kBarY - (kPowerTickH - kBarH) / 2, kPowerTickW, kPowerTickH);
}
