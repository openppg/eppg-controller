#ifndef INC_SP140_LVGL_LVGL_SPLASH_H_
#define INC_SP140_LVGL_LVGL_SPLASH_H_

#include <lvgl.h>

#include "sp140/structs.h"

// Builds and loads the production splash screen. The caller owns the returned
// screen and may delete it after transitioning to the main screen.
lv_obj_t* createLvglSplashScreen(const STR_DEVICE_DATA_140_V1& deviceData);

#endif  // INC_SP140_LVGL_LVGL_SPLASH_H_
