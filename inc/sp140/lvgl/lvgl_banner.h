#ifndef INC_SP140_LVGL_LVGL_BANNER_H_
#define INC_SP140_LVGL_LVGL_BANNER_H_

#include <stdint.h>

enum class BannerColor {
  INFO,     // Blue
  CAUTION   // Amber/orange
};

// Generic timed banner overlay.  Knows nothing about notifications —
// any subsystem can call showBanner() to display a temporary message.
void setupBannerOverlay(bool darkMode);
void showBanner(const char* text, BannerColor color, uint16_t durationMs);
bool isBannerActive();

#endif  // INC_SP140_LVGL_LVGL_BANNER_H_
