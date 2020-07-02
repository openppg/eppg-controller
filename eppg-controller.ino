// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include "libraries/crc.c"       // packet error checking
#include "inc/config.h"          // device config
#include "inc/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include <Adafruit_DRV2605.h>    // haptic controller
#include <Adafruit_SSD1306.h>    // screen
#include <Adafruit_SleepyDog.h>  // watchdog
#include "Adafruit_TinyUSB.h"
#include <AdjustableButtonConfig.h>
#include <ArduinoJson.h>
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <SPI.h>
#include <StaticThreadController.h>
#include <Thread.h>   // run tasks at different intervals
#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>
#include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM

using namespace ace_button;

Adafruit_SSD1306 display(128, 64, &Wire, 4);
Adafruit_DRV2605 vibe;

// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "openppg.github.io/openppg-config");

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button_top(BUTTON_TOP);
AceButton button_side(BUTTON_SIDE);
AdjustableButtonConfig buttonConfig;
extEEPROM eep(kbits_64, 1, 64);

const int bgInterval = 100;  // background updates (milliseconds)

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread butttonThread = Thread();
StaticThreadController<4> threads(&ledBlinkThread, &displayThread,
                                  &throttleThread, &butttonThread);

bool armed = false;
bool use_hub_v2 = true;
int page = 0;
int armAltM = 0;
uint32_t armedAtMilis = 0;
uint32_t cruisedAtMilis = 0;
unsigned int armedSecs = 0;
unsigned int last_throttle = 0;

#pragma message "Warning: OpenPPG software is in beta"

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_SW, OUTPUT);

  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);

  Serial.begin(115200);
  Serial5.begin(115200);
  Serial5.setTimeout(5);

  Serial.print(F("Booting up (USB) V"));
  Serial.print(VERSION_MAJOR + "." + VERSION_MINOR);

  pinMode(LED_SW, OUTPUT);      // set up the external LED pin
  pinMode(LED_2, OUTPUT);       // set up the internal LED2 pin
  pinMode(LED_3, OUTPUT);       // set up the internal LED3 pin
  pinMode(RX_TX_TOGGLE, OUTPUT);

  analogReadResolution(12);     // M0 chip provides 12bit resolution
  pot.setAnalogResolution(4096);
  unsigned int startup_vibes[] = { 27, 27, 0 };
  runVibe(startup_vibes, 3);
  digitalWrite(RX_TX_TOGGLE, LOW);

  initButtons();
  initDisplay();

  ledBlinkThread.onRun(blinkLED);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(updateDisplay);
  displayThread.setInterval(100);

  butttonThread.onRun(checkButtons);
  butttonThread.setInterval(25);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  int countdownMS = Watchdog.enable(4000);
  uint8_t eepStatus = eep.begin(eep.twiClock100kHz);
  refreshDeviceData();
}

// function to echo to both Serial and WebUSB
void echo_all(char chr) {  // from adafruit example
  Serial.write(chr);
  if ( chr == '\r' ) Serial.write('\n');

  usb_web.write(chr);
}

// main loop - everything runs in threads
void loop() {
  Watchdog.reset();
  // from WebUSB to both Serial & webUSB
  if (usb_web.available()) parse_usb_serial();
  // From Serial to both Serial & webUSB
  if (Serial.available())  echo_all(Serial.read());
  threads.run();
}

void checkButtons() {
  button_side.check();
  button_top.check();
}

byte getBatteryPercent() {
  float voltage = hubData.voltage / VOLTAGE_DIVIDE;
  // TODO(zach): LiPo curve
  voltage = constrain(voltage, deviceData.min_batt_v, deviceData.max_batt_v);
  int voltage_curved = sigmoidal(voltage, deviceData.min_batt_v, deviceData.max_batt_v);
  Serial.print("curved = ");
  Serial.print(voltage_curved);
  Serial.print(" flat = ");
  float percent = mapf(voltage, deviceData.min_batt_v, deviceData.max_batt_v, 0, 100);
  Serial.println(percent);

  return round(percent);

  // TODO handle low battery
}

void disarmSystem() {
  unsigned int disarm_melody[] = { 2093, 1976, 880 };
  unsigned int disarm_vibes[] = { 70, 33, 0 };

  armed = false;
  ledBlinkThread.enabled = true;
  updateDisplay();
  runVibe(disarm_vibes, 3);
  playMelody(disarm_melody, 3);
  // update armed_time
  refreshDeviceData();
  deviceData.armed_time += round(armedSecs / 60);  // convert to mins
  writeDeviceData();

  delay(1500);  // dont allow immediate rearming
}

// inital button setup and config
void initButtons() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);
  pinMode(BUTTON_SIDE, INPUT_PULLUP);

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
  handleHubResonse();
  pot.update();
  int rawval = pot.getValue();
  int val = map(rawval, 0, 4095, 0, 1000);  // mapping val to min and max
  sendToHub(val);
}

// format and transmit data to hub
void sendToHub(int throttle_val) {
  memset((uint8_t*)&controlData, 0, sizeof(STR_CTRL2HUB_MSG));

  controlData.version = CTRL_VER;
  controlData.id = CTRL2HUB_ID;
  controlData.length = sizeof(STR_CTRL2HUB_MSG);
  controlData.armed = armed;
  controlData.throttlePercent = throttle_val;
  controlData.crc = crc16((uint8_t*)&controlData, sizeof(STR_CTRL2HUB_MSG) - 2);

  digitalWrite(RX_TX_TOGGLE, HIGH);
  Serial5.write((uint8_t*)&controlData, 8);  // send to hub
  Serial5.flush();
  digitalWrite(RX_TX_TOGGLE, LOW);
}

// read hub data if available and have it converted
void handleHubResonse() {
  int readSize = sizeof(STR_HUB2CTRL_MSG_V2);
  uint8_t serialData[readSize];

  while (Serial5.available() > 0) {
    memset(serialData, 0, sizeof(serialData));
    int size = Serial5.readBytes(serialData, sizeof(STR_HUB2CTRL_MSG_V2));
    receiveHubData(serialData, size);
  }
  Serial5.flush();
}

// convert hub data packets into readable structs
void receiveHubData(uint8_t *buf, uint32_t size) {
  uint16_t crc;
  if (size == sizeof(STR_HUB2CTRL_MSG_V2)) {
    memcpy((uint8_t*)&hubData, buf, sizeof(STR_HUB2CTRL_MSG_V2));
    crc = crc16((uint8_t*)&hubData, sizeof(STR_HUB2CTRL_MSG_V2) - 2);
    use_hub_v2 = true;
  } else if (size == sizeof(STR_HUB2CTRL_MSG_V1)) {
    memcpy((uint8_t*)&hubData, buf, sizeof(STR_HUB2CTRL_MSG_V1));
    crc = crc16((uint8_t*)&hubData, sizeof(STR_HUB2CTRL_MSG_V1) - 2);
    use_hub_v2 = false;
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
    runVibe(arm_fail_vibes, 3);
    handleArmFail();
    armed = false;
    return false;
  }
  ledBlinkThread.enabled = false;
  armedAtMilis = millis();
  armAltM = getAltitudeM();

  runVibe(arm_vibes, 3);
  setLEDs(HIGH);
  playMelody(arm_melody, 3);
  return true;
}

// The event handler for the the buttons
void handleButtonEvent(AceButton *button, uint8_t eventType, uint8_t btnState) {
  uint8_t pin = button->getPin();

  switch (eventType) {
  case AceButton::kEventReleased:
    if (pin == BUTTON_SIDE) nextPage();
    break;
  case AceButton::kEventDoubleClicked:
    if (pin == BUTTON_SIDE) {}
    else if (pin == BUTTON_TOP) {
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
  int seconds = numberOfSeconds(val);

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

uint8_t sigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
	// slow
	// uint8_t result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

	// steep
	// uint8_t result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

	// normal
	uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
	return result >= 100 ? 100 : result;
}
