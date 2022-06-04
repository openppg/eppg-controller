// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include "../../lib/crc.c"       // packet error checking
#include "../../inc/x4/config.h"          // device config
#include "../../inc/x4/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include <bluefruit.h>
#include <Adafruit_DRV2605.h>    // haptic controller
#include <Adafruit_SSD1306.h>    // screen
#include <AdjustableButtonConfig.h>
#include <ArduinoJson.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <SPI.h>
#include <StaticThreadController.h>
#include <Thread.h>   // run tasks at different intervals
//#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>
#include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM

using namespace ace_button;
using namespace Adafruit_LittleFS_Namespace;

#define FILENAME "/openppg.bin"

#include <Adafruit_GFX.h>
#include <gfxfont.h>

const uint8_t SymbolBitmaps[] PROGMEM = {
// Power symbol
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40,
/*| . . . X X . . X . . X X . . . . |*/  0x19,0x30,
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08,
/*| . X . . . . . X . . . . . X . . |*/  0x41,0x04,
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02,
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02,
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02,
/*| X . . . . . . , . . . . . . X . |*/  0x80,0x02,
/*| X . . . . . . , . . . . . . X . |*/  0x80,0x02,
/*| . X . . . . . , . . . . . X . . |*/  0x40,0x04,
/*| . . X . . . . , . . . . X . . . |*/  0x20,0x08,
/*| . . . X X . . , . . X X . . . . |*/  0x18,0x30,
/*| . . . . . X X X X X . . . . . . |*/  0x07,0xc0,
// Bluetooth
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X X . . . . . . . |*/  0x01,0x80,
/*| . . . . . . . X . X . . . . . . |*/  0x01,0x40,
/*| . . . . . . . X . . X . . . . . |*/  0x01,0x20,
/*| . . . . . . . X . . . X . . . . |*/  0x01,0x10,
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08,
/*| . . . X . . . X . . . X . . . . |*/  0x11,0x10,
/*| . . . . X . . X . . X . . . . . |*/  0x09,0x20,
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40,
/*| . . . . . . X X X . . . . . . . |*/  0x03,0x80,
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40,
/*| . . . . X . . X . . X . . . . . |*/  0x09,0x20,
/*| . . . X . . . X . . . X . . . . |*/  0x11,0x10,
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08,
/*| . . . . . . . X . . . X . . . . |*/  0x01,0x10,
/*| . . . . . . . X . . X . . . . . |*/  0x01,0x20,
/*| . . . . . . . X . X . . . . . . |*/  0x01,0x40,
/*| . . . . . . . X X . . . . . . . |*/  0x01,0x80,
// Bluetooth mini
/*| 8 4 2 1 8 4 2 1 |*/
/*| . . . X X . . . |*/  0x18,
/*| . . . X . X . . |*/  0x14,
/*| x . . X . . X . |*/  0x92,
/*| . X . X . X . . |*/  0x54,
/*| . . X X X . . . |*/  0x38,
/*| . x . X . X . . |*/  0x54,
/*| x . . X . . X . |*/  0x92,
/*| . . . X . X . . |*/  0x14,
/*| . . . X X . . . |*/  0x18,

0x00};//One more byte just in case

const GFXglyph SymbolGlyphs[] PROGMEM = {
  //Index,  W, H,xAdv,dX, dY
  {  0, 16,16, 21, 3,-17 }, // 00 power symbol
  {  32, 16, 17, 21, 3,-18}, // 01 Bluetooth
  {  66, 8, 9, 21, 3,-2 } // 02 smaller bluetooth
};
//Index,  W, H,xAdv,dX, dY
const GFXfont Symbol18 PROGMEM = {
  (uint8_t  *)SymbolBitmaps,
  (GFXglyph *)SymbolGlyphs,
  0,48, 35 //ASCII start, ASCII stop,y Advance
};

Adafruit_SSD1306 display(128, 64, &Wire, 4);
Adafruit_DRV2605 vibe;

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button_top(BUTTON_TOP);
AceButton button_side(BUTTON_SIDE);
AdjustableButtonConfig buttonConfig;
File file(InternalFS);

const int bgInterval = 100;  // background updates (milliseconds)

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread buttonThread = Thread();
StaticThreadController<4> threads(&ledBlinkThread, &displayThread,
                                  &throttleThread, &buttonThread);

bool armed = false;
bool use_hub_v2 = true;
int page = 0;
int armAltM = 0;
uint32_t armedAtMilis = 0;
uint32_t cruisedAtMilis = 0;
unsigned int armedSecs = 0;
unsigned int last_throttle = 0;

BLEBas  blebas;  // battery
BLEDis  bledis;  // device information

#pragma message "Warning: OpenPPG software is in beta"

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.print(F("Booting up (USB) V"));
  Serial.println(VERSION_MAJOR + "." + VERSION_MINOR);

  pinMode(LED_SW, OUTPUT);      // set up the external LED pin
  pinMode(LED_2, OUTPUT);       // set up the internal LED2 pin
  pinMode(LED_3, OUTPUT);       // set up the internal LED3 pin
  //pinMode(LED_TESTING, OUTPUT);       // set up the internal LED3 pin

  pinMode(PIN_BSTAT1, INPUT_PULLUP);
  pinMode(PIN_BSTAT2, INPUT_PULLUP);

  analogReadResolution(12);     // chip provides 12bit resolution
  pot.setAnalogResolution(4096);
  unsigned int startup_vibes[] = { 27, 27, 0 };
  //runVibe(startup_vibes, 3);

  initButtons();
  initDisplay();

  ledBlinkThread.onRun(blinkLED);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(updateDisplay);
  displayThread.setInterval(100);

  buttonThread.onRun(checkButtons);
  buttonThread.setInterval(25);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(200);

  InternalFS.begin();
  refreshDeviceData();

  setupBle();
}

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

// main loop - everything runs in threads
void loop() {
  Watchdog.reset();
  // from WebUSB to both Serial & webUSB
  if (usb_web.available()) parse_usb_serial();
  threads.run();
  checkBatt();
}

void checkButtons() {
  button_side.check();
  button_top.check();
}

byte getBatteryPercent() {
  float voltage = hubData.voltage / VOLTAGE_DIVIDE;
  // TODO(zach): LiPo curve
  float percent = mapf(voltage, deviceData.min_batt_v, deviceData.max_batt_v, 0, 100);
  percent = constrain(percent, 0, 100);

  return round(percent);
}

int getInternalBatteryPercent() {
  float measuredvbat = analogRead(PIN_VBAT);
  measuredvbat *= 2;     // we divided by 2, so multiply back
  measuredvbat *= 3.3;   // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024;  // convert to voltage

  // TODO(zach): LiPo curve
  float percent = mapf(measuredvbat, 3.3, 4.2, 0, 100);
  percent = constrain(percent, 0, 100);
  int rounded = round(percent);
  blebas.write(rounded);

  return rounded;
}

void disarmSystem() {
  unsigned int disarm_melody[] = { 2093, 1976, 880 };
  unsigned int disarm_vibes[] = { 70, 33, 0 };

  armed = false;
  ledBlinkThread.enabled = true;
  updateDisplay();
  //runVibe(disarm_vibes, 3);
  playMelody(disarm_melody, 3);
  // update armed_time
  refreshDeviceData();
  Serial.println("disarming");
  Serial.println(armedSecs);
  deviceData.armed_time += round(armedSecs);  // convert to mins // TODO
  writeDeviceData();

  //delay(500); //TODO // dont allow immediate rearming
}

// inital button setup and config
void initButtons() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);
  pinMode(BUTTON_SIDE, INPUT_PULLUP);

  pinMode(BUTTON_PWR, INPUT_PULLUP); // PWR
  pinMode(PIN_BSTAT1, INPUT_PULLUP); // Stat 1
  pinMode(PIN_BSTAT1, INPUT_PULLUP); // Stat 2

  button_side.setButtonConfig(&buttonConfig);
  button_top.setButtonConfig(&buttonConfig);
  buttonConfig.setEventHandler(handleButtonEvent);
  buttonConfig.setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig.setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig.setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
}

// inital screen setup and config
void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setRotation(deviceData.screen_rotation);
  display.setTextSize(3);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(0, 0);
  display.println(F("OpenPPG"));
  display.print(F("V"));
  display.print(VERSION_MAJOR);
  display.print(F("."));
  display.print(VERSION_MINOR);
  display.display();
  display.clearDisplay();
}

// read throttle and send to hub
void handleThrottle() {
  // handleHubResonse();
  pot.update();
  int rawval = pot.getValue();
  // print out the value you read:
  //Serial.print("throttle % is ");
  int val = map(rawval, 0, 4095, 0, 1000);  // mapping val to minimum and maximum
  //Serial.println(val);

  sendToHub(val);
}

// format and transmit data to hub
void sendToHub(int throttle_val) {
  memset((uint8_t*)&controlData, 0, sizeof(STR_BLE_TCTRL2REC_MSG));

  controlData.version = BLE_CTRL_VER;
  controlData.armed = armed;
  controlData.throttlePercent = throttle_val;
  throttleBleLoop(controlData);
}

// read hub data if available and have it converted
void handleHubResonse() {
  int readSize = sizeof(STR_HUB2CTRL_MSG_V2);
  uint8_t serialData[readSize];
}

// convert hub data packets into readable structs
void receiveHubData(uint8_t *buf, uint32_t size) {
  uint16_t crc;
  if (size == sizeof(STR_HUB2CTRL_MSG_V2)) {
    memcpy((uint8_t*)&hubData, buf, sizeof(STR_HUB2CTRL_MSG_V2));
    crc = crc16((uint8_t*)&hubData, sizeof(STR_HUB2CTRL_MSG_V2) - 2);
    use_hub_v2 = true;
  } else {
    Serial.print("wrong size ");
    Serial.print(size);
    Serial.print(" should be ");
    Serial.println(sizeof(STR_HUB2CTRL_MSG_V2));
    return;
  }

  if (crc != hubData.crc) {
    Serial.print(F("hub crc mismatch"));
    return;
  }
  if (hubData.totalCurrent > MAMP_OFFSET) {hubData.totalCurrent -= MAMP_OFFSET;}
}

// get the PPG ready to fly
bool armSystem() {
  unsigned int arm_melody[] = { 1760, 1976, 2093 };
  unsigned int arm_vibes[] = { 70, 33, 0 };
  unsigned int arm_fail_vibes[] = { 14, 3, 0 };

  armed = true;
  sendToHub(0);
  delay(2);  // wait for response
  handleHubResonse();

  if (hubData.armed == 0 && ARM_VERIFY) {
    //runVibe(arm_fail_vibes, 3);
    handleArmFail();
    armed = false;
    return false;
  }
  ledBlinkThread.enabled = false;
  armedAtMilis = millis();
  //armAltM = getAltitudeM();

  //runVibe(arm_vibes, 3);
  setLEDs(HIGH);
  playMelody(arm_melody, 3);
  return true;
}

void formatMemory(){
  InternalFS.begin();

  Serial.print("Formating ... ");
  delay(1);  // for message appear on monitor

  // Format
  InternalFS.format();

  Serial.println("Done");
}

// The event handler for the the buttons
void handleButtonEvent(AceButton *button, uint8_t eventType, uint8_t btnState) {
  uint8_t pin = button->getPin();

  switch (eventType) {
  case AceButton::kEventReleased:
    if (pin == BUTTON_SIDE) nextPage();
    break;
  case AceButton::kEventDoubleClicked:
    if (pin == BUTTON_SIDE) {
      Serial.println("side double");
    } else if (pin == BUTTON_TOP) {
      Serial.println("top double");
      if (armed) {
        disarmSystem();
      } else if (throttleSafe()) {
        armSystem();
      } else {
        handleArmFail();
      }
    }
    break;
  case AceButton::kEventLongPressed:
    int side_state = digitalRead(BUTTON_SIDE);
    int top_state = digitalRead(BUTTON_TOP);

    if (top_state == LOW && side_state == LOW) {
      page = 4;
    }
    break;
  }
}

// Returns true if the throttle/pot is below the safe threshold
bool throttleSafe() {
  // TODO REMOVE
  return true;
  pot.update();
  if (pot.getValue() < 100) {
    return true;
  }
  return false;
}

// convert barometer data to altitude in meters
float getAltitudeM() {
  // from https://github.com/adafruit/Adafruit_BMP3XX/blob/master/Adafruit_BMP3XX.cpp#L208
  float seaLevel = deviceData.sea_pressure;
  float atmospheric = hubData.baroPressure / 100.0F;
  if (hubData.baroPressure < 1) { return 0.0; }
  // convert to fahrenheit if not using metric
  float altitudeM = 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
  return altitudeM;
}

/********
 *
 * Display logic
 *
 *******/

// show data on screen and handle different pages
void updateDisplay() {
  byte percentage;
  String status;

  if (armed) {
    status = F("Armed");
    display.fillCircle(122, 5, 5, SSD1306_WHITE);
    armedSecs = (millis() - armedAtMilis) / 1000;  // update time while armed
  } else {
    status = F("Disarmed");
    display.drawCircle(122, 5, 5, SSD1306_WHITE);
  }

  drawSymbol(100, 3, 2, SSD1306_WHITE, SSD1306_BLACK, 1);
  display.setFont();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(status);
  display.setTextSize(3);

  switch (page) {
  case 0:  // shows current voltage and amperage
    displayPage0();
    break;
  case 1:  // shows total amp hrs and timer
    displayPage1();
    break;
  case 2:  // shows volts and kw
    displayPage2();
    break;
  case 3:  // shows altitude and temp
    displayPage3();
    break;
  case 4:  // shows version and hour meter
    displayVersions();
    break;
  default:
    display.println(F("Dsp Err"));  // should never hit this
    break;
  }
  display.display();
  display.clearDisplay();
}

// displays number of minutes and seconds (since armed)
void displayTime(int val) {
  int minutes = val / 60;  // numberOfMinutes(val);
  int seconds = 33;

  display.print(convertToDigits(minutes));
  display.print(F(":"));
  display.print(convertToDigits(seconds));
}

// display altitude data on screen
void displayAlt() {
  int altiudeM = 0;
  if (armAltM > 0 && deviceData.sea_pressure != DEFAULT_SEA_PRESSURE) {  // MSL
    altiudeM = getAltitudeM();
  } else {  // AGL
    altiudeM = getAltitudeM() - armAltM;
  }

  // convert to ft if not using metric
  int alt = deviceData.metric_alt ? (int)altiudeM : (round(altiudeM * 3.28084));
  display.print(alt, 1);
  display.setTextSize(2);
  display.println(deviceData.metric_alt ? F("m") : F("ft"));
}

// display temperature data on screen
void displayTemp() {
  int tempC = hubData.baroTemp / 100.0F;
  int tempF = tempC * 9/5 + 32;

  display.print(deviceData.metric_temp ? tempC : tempF, 1);
  display.println(deviceData.metric_temp ? F("c") : F("f"));
}

// display first page (voltage and current)
void displayPage0() {
  float voltage = hubData.voltage / VOLTAGE_DIVIDE;
  float current = hubData.totalCurrent / CURRENT_DIVIDE;
  display.print(voltage, 1);
  display.setTextSize(2);
  display.println(F("V"));
  addVSpace();
  display.setTextSize(3);
  display.print(current, 0);
  display.setTextSize(2);
  display.println(F("A"));
}

// display second page (mAh and armed time)
void displayPage1() {
  float amph = hubData.totalMah / 10;
  display.print(amph, 1);
  display.setTextSize(2);
  display.println(F("ah"));
  addVSpace();
  display.setTextSize(3);
  displayTime(armedSecs);
}

// display third page (battery percent and kw)
void displayPage2() {
  float voltage = hubData.voltage / VOLTAGE_DIVIDE;
  float current = hubData.totalCurrent / CURRENT_DIVIDE;
  float kw = (voltage * current) / 1000;
  display.print(getBatteryPercent());
  display.setTextSize(2);
  display.println(F("%"));
  addVSpace();
  display.setTextSize(3);
  display.print(kw, 2);
  display.setTextSize(2);
  display.println(F("kw"));
}

// display fourth page (if compatible) (temperature and altitude)
void displayPage3() {
  if (!use_hub_v2) {
    display.setTextSize(3);
    display.println(F("update"));
    return;
  }
  display.setTextSize(2);
  displayTemp();
  addVSpace();
  display.setTextSize(3);
  displayAlt();
}

// display hidden page (firmware version and total armed time)
void displayVersions() {
  display.setTextSize(2);
  display.print(F("v"));
  display.print(VERSION_MAJOR);
  display.print(F("."));
  display.println(VERSION_MINOR);
  addVSpace();
  display.setTextSize(2);
  displayTime(deviceData.armed_time);
  display.print(F(" h:m"));
  // addVSpace();
  // display.print(chipId()); // TODO: trim down
}

// display hidden page (firmware version and total armed time)
void displayMessage(char *message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(message);
  display.display();
}
