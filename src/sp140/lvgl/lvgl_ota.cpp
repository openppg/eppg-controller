#include "sp140/lvgl/lvgl_ota.h"

#include <cstdio>
#include "version.h"

namespace {
lv_obj_t* label(lv_obj_t* parent, int y, const lv_font_t* font) {
  lv_obj_t* obj = lv_label_create(parent);
  lv_obj_set_width(obj, 148);
  lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(obj, font, 0);
  lv_obj_align(obj, LV_ALIGN_TOP_MID, 0, y);
  return obj;
}

const char* errorText(OtaError error) {
  switch (error) {
    case OtaError::Disconnected: return "Connection lost";
    case OtaError::Timeout: return "Transfer timed out";
    case OtaError::NoPartition: return "No update storage";
    case OtaError::InvalidSize: return "Invalid image size";
    case OtaError::Begin: return "Could not prepare";
    case OtaError::Write: return "Could not save";
    case OtaError::SizeMismatch: return "Incomplete firmware";
    case OtaError::Verification: return "Firmware not valid";
    case OtaError::BootPartition: return "Could not activate";
    case OtaError::RolledBack: return "Previous version kept";
    default: return "Please try again";
  }
}
}  // namespace

void showOtaScreen(OtaScreen& ui, const OtaStatus& status, bool dark) {
  if (!ui.screen) {
    ui.dark = dark;
    ui.screen = lv_obj_create(nullptr);
    lv_obj_remove_flag(ui.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(ui.screen, 0, 0);
    lv_obj_set_style_bg_color(ui.screen,
                             dark ? lv_color_black() : lv_color_white(), 0);
    lv_obj_set_style_text_color(ui.screen,
                               dark ? lv_color_white() : lv_color_black(), 0);
    lv_obj_t* heading = label(ui.screen, 9, &lv_font_montserrat_10);
    lv_label_set_text(heading, "UPDATE");
    ui.title = label(ui.screen, 29, &lv_font_montserrat_14);
    ui.value = label(ui.screen, 50, &lv_font_montserrat_28);
    ui.bar = lv_bar_create(ui.screen);
    lv_obj_set_size(ui.bar, 136, 8);
    lv_obj_align(ui.bar, LV_ALIGN_TOP_MID, 0, 86);
    lv_bar_set_range(ui.bar, 0, 100);
    lv_obj_set_style_bg_color(ui.bar, lv_color_hex(dark ? 0x30343B : 0xE1E5EA), 0);
    lv_obj_set_style_bg_opa(ui.bar, LV_OPA_COVER, 0);
    ui.footer = label(ui.screen, 106, &lv_font_montserrat_12);
    lv_screen_load(ui.screen);
  }

  const unsigned percent = otaProgressPercent(status);
  if (ui.phase == status.phase && ui.error == status.error && ui.percent == percent) return;
  ui.phase = status.phase;
  ui.error = status.error;
  ui.percent = percent;
  const bool failed = status.phase == OtaPhase::Failed;
  const bool complete = status.phase == OtaPhase::Complete;
  const uint32_t accent = failed ? (ui.dark ? 0xFF9A8B : 0xAD281B)
      : complete ? (ui.dark ? 0x62D99A : 0x147344)
                 : (ui.dark ? 0x72BEFF : 0x125FAD);
  lv_obj_set_style_text_color(ui.value, lv_color_hex(accent), 0);
  lv_obj_set_style_bg_color(ui.bar, lv_color_hex(accent), LV_PART_INDICATOR);
  lv_obj_set_style_text_font(ui.value,
      failed ? &lv_font_montserrat_12 : complete ? &lv_font_montserrat_20
                                               : &lv_font_montserrat_28, 0);
  lv_obj_align(ui.value, LV_ALIGN_TOP_MID, 0, failed ? 61 : 50);
  if (failed) lv_obj_add_flag(ui.bar, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_remove_flag(ui.bar, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text(ui.footer, failed ? "Retry in app" : complete ? "" : "Keep power on");

  switch (status.phase) {
    case OtaPhase::Preparing:
      lv_label_set_text(ui.title, "Preparing update");
      lv_label_set_text(ui.value, "...");
      lv_bar_set_value(ui.bar, 0, LV_ANIM_OFF);
      break;
    case OtaPhase::Receiving: {
      lv_label_set_text(ui.title, "Receiving firmware");
      char text[8];
      snprintf(text, sizeof(text), "%u%%", percent);
      lv_label_set_text(ui.value, text);
      lv_bar_set_value(ui.bar, percent, LV_ANIM_OFF);
      break;
    }
    case OtaPhase::Verifying:
      lv_label_set_text(ui.title, "Checking firmware");
      lv_label_set_text(ui.value, "...");
      lv_bar_set_value(ui.bar, 100, LV_ANIM_OFF);
      break;
    case OtaPhase::Restarting:
      lv_label_set_text(ui.title, "Restarting...");
      lv_label_set_text(ui.value, "...");
      lv_bar_set_value(ui.bar, 100, LV_ANIM_OFF);
      break;
    case OtaPhase::Complete:
      lv_label_set_text(ui.title, "Update complete");
      lv_label_set_text(ui.value, "v" VERSION_STRING);
      lv_bar_set_value(ui.bar, 100, LV_ANIM_OFF);
      break;
    case OtaPhase::Failed:
      lv_label_set_text(ui.title,
          status.error == OtaError::Disconnected || status.error == OtaError::Timeout
              ? "Update interrupted" : "Update failed");
      lv_label_set_text(ui.value, errorText(status.error));
      break;
    case OtaPhase::Idle: break;
  }
}

void hideOtaScreen(OtaScreen& ui, lv_obj_t* returnScreen) {
  if (!ui.screen) return;
  lv_screen_load(returnScreen);
  lv_obj_delete(ui.screen);
  ui = OtaScreen{};
}
