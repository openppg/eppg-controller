// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// Factory QC screen implementation. 160x128 ST7735. See lvgl_qc_screen.h.

#include "../../../inc/sp140/lvgl/lvgl_qc_screen.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// Colors
// ---------------------------------------------------------------------------

#define QC_COLOR_PASS lv_color_hex(0x00A650)   // green
#define QC_COLOR_FAIL lv_color_hex(0xE01010)   // red
#define QC_COLOR_SKIP lv_color_hex(0xE0A000)   // amber
#define QC_COLOR_PEND lv_color_hex(0x808080)   // gray

// ---------------------------------------------------------------------------
// Screen objects
// ---------------------------------------------------------------------------

static lv_obj_t* qc_screen = NULL;
static bool qc_dark = false;

// checklist view
static lv_obj_t* list_view = NULL;
static lv_obj_t* row_name_labels[QC_SCREEN_MAX_ROWS] = {NULL};
static lv_obj_t* row_status_labels[QC_SCREEN_MAX_ROWS] = {NULL};

// prompt view
static lv_obj_t* prompt_view = NULL;
static lv_obj_t* prompt_instruction = NULL;
static lv_obj_t* prompt_value = NULL;
static lv_obj_t* prompt_subtext = NULL;
static lv_obj_t* prompt_bar = NULL;

// banner view
static lv_obj_t* banner_view = NULL;
static lv_obj_t* banner_title = NULL;
static lv_obj_t* banner_detail = NULL;

static lv_color_t qcBgColor() {
  return qc_dark ? lv_color_black() : lv_color_white();
}

static lv_color_t qcFgColor() {
  return qc_dark ? lv_color_white() : lv_color_black();
}

static void qcShowOnly(lv_obj_t* view) {
  lv_obj_add_flag(list_view, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(prompt_view, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(banner_view, LV_OBJ_FLAG_HIDDEN);
  lv_obj_remove_flag(view, LV_OBJ_FLAG_HIDDEN);
}

// Bare full-size container with no LVGL default chrome.
static lv_obj_t* qcMakeView(lv_obj_t* parent) {
  lv_obj_t* v = lv_obj_create(parent);
  lv_obj_set_size(v, 160, 128);
  lv_obj_set_pos(v, 0, 0);
  lv_obj_remove_flag(v, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_border_width(v, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(v, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(v, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(v, LV_OPA_0, LV_PART_MAIN);
  return v;
}

void setupQcScreen(bool darkMode) {
  qc_dark = darkMode;

  if (qc_screen != NULL) {
    lv_obj_delete(qc_screen);
    qc_screen = NULL;
  }

  qc_screen = lv_obj_create(NULL);
  lv_obj_remove_flag(qc_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(qc_screen, qcBgColor(), LV_PART_MAIN);

  // ---- checklist view ----
  list_view = qcMakeView(qc_screen);

  lv_obj_t* title = lv_label_create(list_view);
  lv_label_set_text(title, "FACTORY QC");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(title, qcFgColor(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

  for (uint8_t i = 0; i < QC_SCREEN_MAX_ROWS; i++) {
    const int y = 17 + i * 13;

    row_name_labels[i] = lv_label_create(list_view);
    lv_label_set_text(row_name_labels[i], "");
    lv_obj_set_style_text_font(row_name_labels[i], &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(row_name_labels[i], qcFgColor(), 0);
    lv_obj_set_pos(row_name_labels[i], 4, y);

    row_status_labels[i] = lv_label_create(list_view);
    lv_label_set_text(row_status_labels[i], "");
    lv_obj_set_style_text_font(row_status_labels[i], &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(row_status_labels[i], QC_COLOR_PEND, 0);
    lv_obj_set_width(row_status_labels[i], 76);
    lv_obj_set_style_text_align(row_status_labels[i], LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(row_status_labels[i], 80, y);
  }

  // ---- prompt view ----
  prompt_view = qcMakeView(qc_screen);

  prompt_instruction = lv_label_create(prompt_view);
  lv_label_set_text(prompt_instruction, "");
  lv_obj_set_style_text_font(prompt_instruction, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(prompt_instruction, qcFgColor(), 0);
  lv_obj_set_width(prompt_instruction, 152);
  lv_obj_set_style_text_align(prompt_instruction, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(prompt_instruction, LV_LABEL_LONG_WRAP);
  lv_obj_align(prompt_instruction, LV_ALIGN_TOP_MID, 0, 6);

  prompt_value = lv_label_create(prompt_view);
  lv_label_set_text(prompt_value, "");
  lv_obj_set_style_text_font(prompt_value, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(prompt_value, qcFgColor(), 0);
  lv_obj_align(prompt_value, LV_ALIGN_CENTER, 0, 8);

  prompt_subtext = lv_label_create(prompt_view);
  lv_label_set_text(prompt_subtext, "");
  lv_obj_set_style_text_font(prompt_subtext, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(prompt_subtext, qcFgColor(), 0);
  lv_obj_set_width(prompt_subtext, 152);
  lv_obj_set_style_text_align(prompt_subtext, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(prompt_subtext, LV_LABEL_LONG_WRAP);
  lv_obj_align(prompt_subtext, LV_ALIGN_BOTTOM_MID, 0, -14);

  prompt_bar = lv_bar_create(prompt_view);
  lv_obj_set_size(prompt_bar, 152, 6);
  lv_obj_align(prompt_bar, LV_ALIGN_BOTTOM_MID, 0, -4);
  lv_bar_set_range(prompt_bar, 0, 100);
  lv_bar_set_value(prompt_bar, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(prompt_bar, QC_COLOR_PEND, LV_PART_MAIN);
  lv_obj_set_style_bg_color(prompt_bar, QC_COLOR_PASS, LV_PART_INDICATOR);

  // ---- banner view ----
  banner_view = qcMakeView(qc_screen);
  lv_obj_set_style_bg_opa(banner_view, LV_OPA_100, LV_PART_MAIN);

  banner_title = lv_label_create(banner_view);
  lv_label_set_text(banner_title, "");
  lv_obj_set_style_text_font(banner_title, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(banner_title, lv_color_white(), 0);
  lv_obj_align(banner_title, LV_ALIGN_CENTER, 0, -18);

  banner_detail = lv_label_create(banner_view);
  lv_label_set_text(banner_detail, "");
  lv_obj_set_style_text_font(banner_detail, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(banner_detail, lv_color_white(), 0);
  lv_obj_set_width(banner_detail, 152);
  lv_obj_set_style_text_align(banner_detail, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(banner_detail, LV_LABEL_LONG_WRAP);
  lv_obj_align(banner_detail, LV_ALIGN_CENTER, 0, 22);

  qcShowOnly(list_view);
  lv_screen_load(qc_screen);
}

void qcScreenSetCheck(uint8_t row, const char* name, QcCheckStatus status,
                      const char* value) {
  if (qc_screen == NULL || row >= QC_SCREEN_MAX_ROWS) {
    return;
  }
  lv_label_set_text(row_name_labels[row], name);

  char statusText[32];
  const char* word;
  lv_color_t color;
  switch (status) {
    case QcCheckStatus::PASS: word = "OK";   color = QC_COLOR_PASS; break;
    case QcCheckStatus::FAIL: word = "FAIL"; color = QC_COLOR_FAIL; break;
    case QcCheckStatus::SKIP: word = "SKIP"; color = QC_COLOR_SKIP; break;
    case QcCheckStatus::NOT_RUN:
    default:                  word = "...";  color = QC_COLOR_PEND; break;
  }
  if (value != NULL && value[0] != '\0') {
    snprintf(statusText, sizeof(statusText), "%s %s", value, word);
  } else {
    snprintf(statusText, sizeof(statusText), "%s", word);
  }
  lv_label_set_text(row_status_labels[row], statusText);
  lv_obj_set_style_text_color(row_status_labels[row], color, 0);
}

void qcScreenPrompt(const char* instruction, const char* subtext) {
  if (qc_screen == NULL) {
    return;
  }
  lv_label_set_text(prompt_instruction, instruction != NULL ? instruction : "");
  lv_label_set_text(prompt_subtext, subtext != NULL ? subtext : "");
  lv_label_set_text(prompt_value, "");
  lv_bar_set_value(prompt_bar, 0, LV_ANIM_OFF);
  qcShowOnly(prompt_view);
}

void qcScreenPromptValue(const char* text) {
  if (qc_screen == NULL) {
    return;
  }
  lv_label_set_text(prompt_value, text != NULL ? text : "");
}

void qcScreenPromptProgress(uint8_t pct) {
  if (qc_screen == NULL) {
    return;
  }
  lv_bar_set_value(prompt_bar, pct > 100 ? 100 : pct, LV_ANIM_OFF);
}

void qcScreenShowChecklist() {
  if (qc_screen == NULL) {
    return;
  }
  qcShowOnly(list_view);
}

void qcScreenBanner(bool passed, const char* detail) {
  if (qc_screen == NULL) {
    return;
  }
  lv_obj_set_style_bg_color(banner_view,
                            passed ? QC_COLOR_PASS : QC_COLOR_FAIL,
                            LV_PART_MAIN);
  lv_label_set_text(banner_title, passed ? "QC PASSED" : "QC FAILED");
  lv_label_set_text(banner_detail, detail != NULL ? detail : "");
  qcShowOnly(banner_view);
}

void teardownQcScreen(lv_obj_t* nextScreen) {
  if (qc_screen == NULL) {
    return;
  }
  if (nextScreen != NULL) {
    lv_screen_load(nextScreen);
  }
  lv_obj_delete(qc_screen);
  qc_screen = NULL;
  list_view = NULL;
  prompt_view = NULL;
  banner_view = NULL;
}
