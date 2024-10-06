#include "sp140/display.h"

#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

#include "../../inc/fonts/sans_reg10.h"
#include "../../inc/fonts/sans_reg16.h"
#include "../../inc/version.h"
#include "sp140/structs.h"

#define FONT_HEIGHT_OFFSET 14

// DEBUG WATCHDOG
#ifdef RP_PIO
  #include "hardware/watchdog.h"
  bool watchdogCausedReboot = false;
  bool watchdogEnableCausedReboot = false;
#endif

Adafruit_ST7735* display;
GFXcanvas16 canvas(160, 128);

// Light Mode Colors
UIColors lightModeColors = {
  BLACK,   // Default Text Color
  RED,     // Error Text Color
  BLUE,    // Chill Text Color
  WHITE,   // Default BG Color
  CYAN,    // Armed BG Color
  YELLOW,  // Cruise BG Color
  BLACK    // UI Accent Color
};

// Dark Mode Colors
UIColors darkModeColors = {
  WHITE,   // Default Text Color
  RED,     // Error Text Color
  CYAN,    // Chill Text Color
  BLACK,   // Default BG Color
  BLUE,    // Armed BG Color
  ORANGE,  // Cruise BG Color
  GRAY     // UI Accent Color
};

// Pointer to the current color set
UIColors *currentTheme;

/**
 * This function takes a voltage level as input and returns the corresponding
 * battery percentage. It uses a lookup table (`batteryLevels`) to map voltage
 * levels to percentages. Thats based  on a simple set of data points from load testing.
 * If the input voltage is greater than a threshold
 * voltage, linear interpolation is used to calculate the percentage between
 * two consecutive voltage-percentage mappings.
 *
 * @param voltage The input voltage level.
 * @return The calculated battery percentage (0-100).
 */
float getBatteryPercent(float voltage) {
    // Calculate the number of voltage-percentage mappings
    int numLevels = sizeof(batteryLevels) / sizeof(BatteryVoltagePoint);

    // Handle edge cases where the voltage is outside the defined range
    if (voltage >= batteryLevels[0].voltage) {
        return batteryLevels[0].percent;
    } else if (voltage <= batteryLevels[numLevels - 1].voltage) {
        return batteryLevels[numLevels - 1].percent;
    }

    // Iterate through the voltage-percentage mappings
    for (int i = 0; i < numLevels - 1; i++) {
        // Check if the input voltage is between the current and next mapping
        if (voltage <= batteryLevels[i].voltage && voltage > batteryLevels[i + 1].voltage) {
            // Interpolate the percentage between the current and next mapping
            return mapd(voltage, batteryLevels[i + 1].voltage, batteryLevels[i].voltage,
                        batteryLevels[i + 1].percent, batteryLevels[i].percent);
        }
    }

    return 0;  // Fallback, should never reach here
}
// Clears screen and resets properties
void resetRotation(unsigned int rotation) {
  display->setRotation(rotation);  // 1=right hand, 3=left hand
}

// Show splash screen with animated text
void displayMeta(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  display->fillScreen(currentTheme->default_bg);
  display->setTextSize(1);
  display->setFont(&FreeSansBold12pt7b);
  display->setTextColor(currentTheme->default_text);

  // Animate "OpenPPG" text
  const char* text = "OpenPPG";
  int x = 25;
  int y = 30;
  for (int i = 0; text[i] != '\0'; i++) {
    display->setCursor(x, y);
    display->print(text[i]);
    x += display->getCursorX() - x;  // Move cursor to the right
    delay(100);  // Adjust delay for animation speed
  }

  display->setFont(&Open_Sans_Reg_16);
  display->setTextSize(1);
  display->setCursor(60, 60);
  display->printf("v%d.%d", VERSION_MAJOR, VERSION_MINOR);
#ifdef RP_PIO
  display->print("R");
#endif
  // Total armed time
  display->setCursor(60, 90);

  //const int hours = deviceData.armed_time / 60;
  //const int minutes = deviceData.armed_time % 60;
  //display->printf("%02d:%02d", hours, minutes);
  delay(duration);
}

void displayMeta(int duration) {
  display->fillScreen(WHITE);
  display->setTextSize(1);
  display->setFont(&FreeSansBold12pt7b);
  display->setTextColor(BLACK);

  // Animate "OpenPPG" text
  const char* text = "OpenPPG";
  int x = 25;
  int y = 30;
  for (int i = 0; text[i] != '\0'; i++) {
    display->setCursor(x, y);
    display->print(text[i]);
    x += display->getCursorX() - x;  // Move cursor to the right
    delay(100);  // Adjust delay for animation speed
  }

  display->setFont(&Open_Sans_Reg_16);
  display->setTextSize(1);
  display->setCursor(60, 60);
  display->print(VERSION_STRING);
#ifdef RP_PIO
  display->print("R");
#endif
  delay(duration);
}

void setupDisplay() {
  USBSerial.println("setupDisplay");

  //display = new Adafruit_ST7735(board_config.tft_cs, board_config.tft_dc, board_config.tft_rst);
  display = new Adafruit_ST7735(13, 14, 4, 2, 15);
  //Adafruit_ST7735::Adafruit_ST7735(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst)

  display->initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  //pinMode(board_config.tft_lite, OUTPUT);
  //digitalWrite(board_config.tft_lite, HIGH);  // Backlight on
  resetRotation(1);
  //setTheme(deviceData.theme);  // 0=light, 1=dark
  display->fillScreen(ST77XX_BLACK);
  displayMeta();
}


// inital screen setup and config with devicedata and boardconfig passed in
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config) {
  USBSerial.println("setupDisplay");
#ifdef RP_PIO
  watchdogCausedReboot = watchdog_caused_reboot();
  watchdogEnableCausedReboot = watchdog_enable_caused_reboot();
#endif

  //display = new Adafruit_ST7735(board_config.tft_cs, board_config.tft_dc, board_config.tft_rst);
  display = new Adafruit_ST7735(13, 14, 4, 2, 15);
  //Adafruit_ST7735::Adafruit_ST7735(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst)

  display->initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  //pinMode(board_config.tft_lite, OUTPUT);
  //digitalWrite(board_config.tft_lite, HIGH);  // Backlight on
  //resetRotation(deviceData.screen_rotation);
  //setTheme(deviceData.theme);  // 0=light, 1=dark
  display->fillScreen(ST77XX_BLACK);
  displayMeta(deviceData);
}

void updateDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
  ) {
  canvas.fillScreen(currentTheme->default_bg);
  canvas.setTextWrap(false);
  canvas.setFont(&Open_Sans_Reg_16);
  canvas.setTextSize(1);

  const unsigned int nowMillis = millis();

  // Display region lines
  canvas.drawFastHLine(0, 32, 160, currentTheme->ui_accent);
  canvas.drawFastVLine(100, 0, 32, currentTheme->ui_accent);
  canvas.drawFastHLine(0, 80, 160, currentTheme->ui_accent);
  canvas.drawFastHLine(0, 92, 160, currentTheme->ui_accent);

  // Display battery level and status
  const float batteryPercent = getBatteryPercent(escTelemetry.volts);

  //   Display battery bar
  if (batteryPercent > 0) {
    unsigned int batteryColor = RED;
    if (batteryPercent >= 30) batteryColor = GREEN;
    else if (batteryPercent >= 15) batteryColor = YELLOW;
    int batteryPercentWidth = map(static_cast<int>(batteryPercent), 0, 100, 0, 100);
    canvas.fillRect(0, 0, batteryPercentWidth, 32, batteryColor);
  } else {
    canvas.setTextColor(currentTheme->error_text);
    if (escTelemetry.volts > 10) {
      canvas.setCursor(6, 15);
      canvas.print("BATTERY");
      canvas.setCursor(6, 17 + FONT_HEIGHT_OFFSET);
      canvas.print(" DEAD");
    } else {
      canvas.setCursor(10, 15);
      canvas.print("NO");
      canvas.setCursor(10, 17 + FONT_HEIGHT_OFFSET);
      canvas.print("TELEM");
    }
  }
  // Draw ends of battery outline
  canvas.fillRect(97, 0, 3, 9, currentTheme->ui_accent);
  canvas.fillRect(97, 23, 3, 9, currentTheme->ui_accent);
  canvas.fillRect(0, 0, 1, 2, currentTheme->ui_accent);
  canvas.fillRect(0, 30, 1, 2, currentTheme->ui_accent);

  // Display battery percent
  canvas.setCursor(108, 10 + FONT_HEIGHT_OFFSET);
  canvas.setTextColor(currentTheme->default_text);
  if (batteryPercent > 0) {
    canvas.printf("%3d%%", static_cast<int>(batteryPercent));
  } else {
    canvas.print(" ?%");
  }

  float kWatts = constrain(watts / 1000.0, 0, 50);
  float volts = escTelemetry.volts;
  float kWh = wattHoursUsed / 1000.0;
  float amps = escTelemetry.amps;

  // for testing
  // float kWatts = 15.1;
  // float volts = 98.7;
  // float kWh = 3.343;
  // float amps = 10.35;

  canvas.setCursor(1, 40 + FONT_HEIGHT_OFFSET);
  canvas.printf(kWatts < 10 ? "  %4.1fkW" : "%4.1fkW", kWatts);

  canvas.setCursor(100, 40 + FONT_HEIGHT_OFFSET);
  canvas.printf(volts > 99.9 ? "%3.0fV" : "%4.1fV", volts);

  canvas.setCursor(1, 61 + FONT_HEIGHT_OFFSET);
  canvas.printf(kWh > 99.9 ? "%3.0fkWh" : "%4.1fkWh", kWh);

  canvas.setCursor(100, 61 + FONT_HEIGHT_OFFSET);
  canvas.printf(amps > 99.9 ? "%3.0fA" : "%4.1fA", amps);

  // Display modes
  canvas.setCursor(8, 90);
  canvas.setFont(&Open_Sans_Reg_10);
  if (deviceData.performance_mode == 0) {
    canvas.setTextColor(currentTheme->chill_text);
    canvas.print("CHILL");
  } else {
    canvas.setTextColor(RED);
    canvas.print("SPORT");
  }

  // canvas.setCursor(46, 83);
  // if (armed) {
  //   canvas.setTextColor(BLACK, CYAN);
  //   canvas.print("ARMED");
  // } else {
  //   canvas.setTextColor(BLACK, GREEN);
  //   canvas.print("SAFED");
  // }

  // if (cruising) {
  //   canvas.setCursor(84, 83);
  //   canvas.setTextColor(BLACK, YELLOW);
  //   canvas.print("CRUISE");
  // }

  // canvas.setCursor(124, 83);
  // canvas.setTextColor(BLACK);
  // canvas.printf("FLAG%2d", escTelemetry.statusFlag);

  // Display statusbar
  unsigned int statusBarColor = currentTheme->default_bg;
  if (cruising) statusBarColor = currentTheme->cruise_bg;
  else if (armed) statusBarColor = currentTheme->armed_bg;
  canvas.fillRect(0, 93, 160, 40, statusBarColor);

  // Display armed time for the current session
  canvas.setTextColor(currentTheme->default_text);
  canvas.setFont(&Open_Sans_Reg_16);
  canvas.setCursor(8, 102 + FONT_HEIGHT_OFFSET);
  static unsigned int _lastArmedMillis = 0;
  if (armed) _lastArmedMillis = nowMillis;
  const int sessionSeconds = (_lastArmedMillis - armedStartMillis) / 1000.0;
  canvas.printf("%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);

  // Display altitude
  canvas.setCursor(80, 102 + FONT_HEIGHT_OFFSET);
  if (altitude == __FLT_MIN__) {
    canvas.setTextColor(currentTheme->error_text);
    canvas.print(F("ALTERR"));
  } else {
    canvas.setTextColor(currentTheme->default_text);
    if (deviceData.metric_alt) {
      canvas.printf("%6.1fm", altitude);
    } else {
      canvas.printf("%5dft", static_cast<int>(round(altitude * 3.28084)));
    }
  }

  // ESC temperature
  canvas.setCursor(100, 90);
  canvas.setTextColor(currentTheme->default_text);
  canvas.setFont(&Open_Sans_Reg_10);
  canvas.print("ESC ");
  float escTemp;
  escTemp = escTelemetry.temperatureC;

  if (escTemp >= 100) { canvas.setTextColor(currentTheme->error_text); }  // If temperature is 100C+, display in red.
  if (escTemp == __FLT_MIN__ || escTemp == 0.0) {  // If temperature is not available, display a question mark.
    canvas.printf("?%c", 247);
  } else {  // Otherwise, display the temperature. (in degrees C)
    canvas.printf("%0.0fC", escTemp);
  }

//  // DEBUG TIMING
//  canvas.setTextSize(1);
//  canvas.setCursor(4, 118);
//  static unsigned int lastDisplayMillis = 0;
//  canvas.printf("%5d  %5d", nowMillis - escTelemetry.lastUpdateMillis, nowMillis - lastDisplayMillis);
//  lastDisplayMillis = nowMillis;
//
//  canvas.printf("  %3d %2d %2d", escTelemetry.lastReadBytes, escTelemetry.errorStopBytes, escTelemetry.errorChecksum);

//  // DEBUG WATCHDOG
//  #ifdef RP_PIO
//    canvas.setTextSize(1);
//    canvas.setCursor(4, 118);
//    canvas.printf("watchdog %d %d", watchdogCausedReboot, watchdogEnableCausedReboot);
//  #endif
//
//  // DEBUG FREE MEMORY
//  #ifdef RP_PIO
//    canvas.printf("  mem %d", rp2040.getFreeHeap());
//  #endif


  // Draw the canvas to the display->
  display->drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
}

// Set the theme
void setTheme(int theme) {
  if (theme == 1) {
    currentTheme = &darkModeColors;
    // Serial.println("Switched to Dark Mode");
  } else {
    currentTheme = &lightModeColors;
    // Serial.println("Switched to Light Mode");
  }
}
