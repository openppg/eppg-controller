#ifndef INC_SP140_DISPLAY_H_
#define INC_SP140_DISPLAY_H_

#include <Arduino.h>

#include "sp140/structs.h"

#ifdef RP_PIO
  #include "../../inc/sp140/rp2040-config.h"
#elif CAN_PIO
  #include "../../inc/sp140/esp32s3-config.h"
#endif

#include <Adafruit_ST7735.h>
#include "utilities.h"

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

// Include fonts
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include "../../inc/fonts/sans_reg10.h"
#include "../../inc/fonts/sans_reg12.h"
#include "../../inc/fonts/sans_reg14.h"
#include "../../inc/fonts/sans_reg16.h"
#include "../../inc/fonts/sans_reg18.h"

#define BLACK                 ST77XX_BLACK
#define WHITE                 ST77XX_WHITE
#define GREEN                 ST77XX_GREEN
#define YELLOW                ST77XX_YELLOW
#define RED                   ST77XX_RED
#define BLUE                  ST77XX_BLUE
#define ORANGE                ST77XX_ORANGE
#define CYAN                  ST77XX_CYAN
#define PURPLE                0x780F
#define GRAY                  0xDEFB

namespace Fonts {
  constexpr const GFXfont* Title = &FreeSansBold12pt7b;
  constexpr const GFXfont* Large = &Open_Sans_Reg_20;
  constexpr const GFXfont* Medium = &Open_Sans_Reg_14;
  constexpr const GFXfont* Small = &Open_Sans_Reg_10;
}

// Screen dimensions
#define SCREEN_WIDTH          160
#define SCREEN_HEIGHT         128

// Battery thresholds
#define BATTERY_LOW_THRESHOLD     15
#define BATTERY_MEDIUM_THRESHOLD  30

// Light mode (default)
// #define DEFAULT_TEXT_COLOR    BLACK
// #define ERROR_TEXT_COLOR      RED
// #define CHILL_TEXT_COLOR      CYAN

// #define DEFAULT_BG_COLOR      WHITE
// #define ARMED_BG_COLOR        CYAN
// #define CRUISE_BG_COLOR       YELLOW
// #define UI_ACCENT_COLOR       BLACK

// Dark mode
// #define DEFAULT_TEXT_COLOR    WHITE
// #define ERROR_TEXT_COLOR      RED
// #define CHILL_TEXT_COLOR      CYAN

// #define DEFAULT_BG_COLOR      BLACK
// #define ARMED_BG_COLOR        BLUE
// #define CRUISE_BG_COLOR       ORANGE
// #define UI_ACCENT_COLOR       GRAY

struct UIColors {
  uint16_t default_text;
  uint16_t error_text;
  uint16_t chill_text;
  uint16_t default_bg;
  uint16_t armed_bg;
  uint16_t cruise_bg;
  uint16_t ui_accent;
};

extern float watts;
extern float wattHoursUsed;

// Set up the display and show splash screen
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config);

void displayMeta(const STR_DEVICE_DATA_140_V1& deviceData, int duration = 1500);

// Clear screen and reset properties
void resetRotation(unsigned int orientation);

// Show data on screen
void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData,
                   const STR_ESC_TELEMETRY_140& escTelemetry,
                   const STR_BMS_TELEMETRY_140& bmsTelemetry,
                   const UnifiedBatteryData& unifiedBatteryData,
                   float altitude, bool armed, bool cruising,
                   unsigned int armedStartMillis);

void setTheme(int theme);  // 0,1

#endif  // INC_SP140_DISPLAY_H_
