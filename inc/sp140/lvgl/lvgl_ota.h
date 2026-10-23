#ifndef INC_SP140_LVGL_LVGL_OTA_H_
#define INC_SP140_LVGL_LVGL_OTA_H_

#include <lvgl.h>
#include "sp140/ble/ota_status.h"

// Owned exclusively by the UI task; call with lvglMutex held.
struct OtaScreen {
  lv_obj_t* screen = nullptr;
  lv_obj_t* title = nullptr;
  lv_obj_t* value = nullptr;
  lv_obj_t* bar = nullptr;
  lv_obj_t* footer = nullptr;
  OtaPhase phase = OtaPhase::Idle;
  OtaError error = OtaError::None;
  unsigned percent = 101;
  bool dark = false;
};

void showOtaScreen(OtaScreen& ui, const OtaStatus& status, bool dark);
void hideOtaScreen(OtaScreen& ui, lv_obj_t* returnScreen);

#endif  // INC_SP140_LVGL_LVGL_OTA_H_
