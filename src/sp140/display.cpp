#include "sp140/display.h"

#include "../../inc/sans_reg10.h"
#include "../../inc/sans_reg16.h"
#include "../../inc/version.h"
#include "sp140/structs.h"

#define FONT_HEIGHT_OFFSET 14

// DEBUG WATCHDOG
#ifdef RP_PIO
  #include "hardware/watchdog.h"
  bool watchdogCausedReboot = false;
  bool watchdogEnableCausedReboot = false;
#endif

Arduino_DataBus *bus;
Arduino_GFX *display;
Arduino_GFX *canvas;

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

// Map volts to battery percentage, based on a
// simple set of data points from load testing.
float getBatteryPercent(float volts) {
  float battPercent = 0;
  if (volts > 94.8) {
    battPercent = mapd(volts, 94.8, 99.6, 90, 100);
  } else if (volts > 93.36) {
    battPercent = mapd(volts, 93.36, 94.8, 80, 90);
  } else if (volts > 91.68) {
    battPercent = mapd(volts, 91.68, 93.36, 70, 80);
  } else if (volts > 89.76) {
    battPercent = mapd(volts, 89.76, 91.68, 60, 70);
  } else if (volts > 87.6) {
    battPercent = mapd(volts, 87.6, 89.76, 50, 60);
  } else if (volts > 85.2) {
    battPercent = mapd(volts, 85.2, 87.6, 40, 50);
  } else if (volts > 82.32) {
    battPercent = mapd(volts, 82.32, 85.2, 30, 40);
  } else if (volts > 80.16) {
    battPercent = mapd(volts, 80.16, 82.32, 20, 30);
  } else if (volts > 78) {
    battPercent = mapd(volts, 78, 80.16, 10, 20);
  } else if (volts > 60.96) {
    battPercent = mapd(volts, 60.96, 78, 0, 10);
  }
  return constrain(battPercent, 0, 100);
}

// Clears screen and resets properties
void resetRotation(unsigned int rotation) {
  display->setRotation(rotation);  // 1=right hand, 3=left hand
}

// Show splash screen
void displayMeta(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  Serial.println("Displaying Meta");
  return;
  display->fillScreen(currentTheme->default_bg);
  display->setTextSize(1);
  display->setFont(&Open_Sans_Reg_16);
  display->setTextColor(currentTheme->default_text);
  display->setCursor(25, 30);
  display->println("OpenPPG");
  display->setFont(&Open_Sans_Reg_16);
  display->setTextSize(1);
  display->setCursor(60, 60);
  display->printf("v%d.%d", VERSION_MAJOR, VERSION_MINOR);
#ifdef RP_PIO
  display->print("R");
#endif
  // Total armed time
  display->setCursor(54, 90);

  const int hours = deviceData.armed_time / 60;
  const int minutes = deviceData.armed_time % 60;
  display->printf("%02d:%02d", hours, minutes);
  display->flush();

  delay(duration);
  Serial.println("Done Meta");

}

// inital screen setup and config with devicedata and boardconfig passed in
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config) {
#ifdef RP_PIO
  watchdogCausedReboot = watchdog_caused_reboot();
  watchdogEnableCausedReboot = watchdog_enable_caused_reboot();
#endif

  bus = new Arduino_HWSPI(board_config.tft_dc /* DC */, board_config.tft_cs /* CS */);

  display = new Arduino_ST7735(
    bus, board_config.tft_rst /* RST */, 0 /* rotation */, false /* IPS */,
    128 /* width */, 160 /* height */,
    2 /* col offset 1 */, 1 /* row offset 1 */,
    2 /* col offset 2 */, 1 /* row offset 2 */,
    false /* BGR */);

  canvas = new Arduino_Canvas(160 /* width */, 128 /* height */, display);

  pinMode(board_config.tft_lite, OUTPUT);
  digitalWrite(board_config.tft_lite, HIGH);  // Backlight on
  display->begin();
  resetRotation(deviceData.screen_rotation);
  setTheme(deviceData.theme);  // 0=light, 1=dark
  displayMeta(deviceData);
}
void updateDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
  ) {
  unsigned long startMillis = millis();  // Capture start time

  canvas->fillScreen(currentTheme->default_bg);
  canvas->setTextWrap(false);
  canvas->setFont(&Open_Sans_Reg_16);
  canvas->setTextSize(1);

  const unsigned int nowMillis = millis();

  display->setCursor(5, 5);
  display->printf("Hello World");
 // removed code goes here

  // Draw the canvas to the display
  canvas->flush();
  unsigned long endMillis = millis();  // Capture end time
  unsigned long executionTime = endMillis - startMillis;  // Calculate execution time
  Serial.print("Screen update execution time: ");
  Serial.println(executionTime);  // Print execution time to the Serial monitor
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
