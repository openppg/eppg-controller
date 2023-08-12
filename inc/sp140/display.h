#ifndef INCLUDE_SP140_DISPLAY_H_
#define INCLUDE_SP140_DISPLAY_H_

#include <Arduino.h>

#include "sp140/structs.h"
#include <Adafruit_ST7735.h>
#include "utilities.h"

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

#define DEFAULT_BG_COLOR      WHITE
#define ARMED_BG_COLOR        ST77XX_CYAN
#define CRUISE_BG_COLOR       YELLOW

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

extern float watts;
extern float wattHoursUsed;

// Set up the display and show splash screen
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData);

void displayMeta(const STR_DEVICE_DATA_140_V1& deviceData, int duration = 2000);

// Clear screen and reset properties
void resetRotation(unsigned int orientation);

// Show data on screen
void updateDisplay(const STR_DEVICE_DATA_140_V1& deviceData,
                   const STR_ESC_TELEMETRY_140& escTelemetry,
                   float altitude, bool armed, bool cruising,
                   unsigned int armedStartMillis);

#endif  // INCLUDE_SP140_DISPLAY_H_
