// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include "../../lib/crc.c"       // packet error checking
#include "../../inc/sp140/config.h"          // device config
#include "../../inc/sp140/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include "Adafruit_TinyUSB.h"
#include <Adafruit_BMP3XX.h>     // barometer
#include <Adafruit_DRV2605.h>    // haptic controller
#include <Adafruit_ST7735.h>     // screen
#include <Adafruit_SleepyDog.h>  // watchdog
#include <ArduinoJson.h>
#include <CircularBuffer.h>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <Servo.h>               // to control ESCs
#include <SPI.h>
#include <StaticThreadController.h>
#include <Thread.h>   // run tasks at different intervals
#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>
#include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM

#include <Fonts/FreeSansBold12pt7b.h>

#include "../../inc/sp140/globals.h"  // device config

using namespace ace_button;

Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_DRV2605 vibe;

// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button_top(BUTTON_TOP);
ButtonConfig* buttonConfig = button_top.getButtonConfig();
extEEPROM eep(kbits_64, 1, 64);
CircularBuffer<float, 50> voltageBuffer;
CircularBuffer<int, 8> potBuffer;

Thread ledBlinkThread = Thread();
Thread displayThread = Thread();
Thread throttleThread = Thread();
Thread buttonThread = Thread();
Thread telemetryThread = Thread();
Thread counterThread = Thread();
StaticThreadController<6> threads(&ledBlinkThread, &displayThread, &throttleThread,
                                  &buttonThread, &telemetryThread, &counterThread);

bool armed = false;
bool use_hub_v2 = true;
int page = 0;
float armAltM = 0;
uint32_t armedAtMilis = 0;
uint32_t cruisedAtMilisMilis = 0;
unsigned int armedSecs = 0;
unsigned int last_throttle = 0;

#pragma message "Warning: OpenPPG software is in beta"

// the setup function runs once when you press reset or power the board
void setup() {
  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);

  Serial.begin(115200);
  Serial5.begin(ESC_BAUD_RATE);
  Serial5.setTimeout(ESC_TIMEOUT);

  //Serial.print(F("Booting up (USB) V"));
  //Serial.print(VERSION_MAJOR + "." + VERSION_MINOR);

  pinMode(LED_SW, OUTPUT);   // set up the internal LED2 pin

  analogReadResolution(12);     // M0 chip provides 12bit resolution
  pot.setAnalogResolution(4096);
  unsigned int startup_vibes[] = { 27, 27, 0 };
  runVibe(startup_vibes, 3);

  initButtons();

  ledBlinkThread.onRun(blinkLED);
  ledBlinkThread.setInterval(500);

  displayThread.onRun(updateDisplay);
  displayThread.setInterval(200);

  buttonThread.onRun(checkButtons);
  buttonThread.setInterval(5);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  telemetryThread.onRun(handleTelemetry);
  telemetryThread.setInterval(100);

  counterThread.onRun(trackPower);
  counterThread.setInterval(250);

  Watchdog.enable(5000);
  uint8_t eepStatus = eep.begin(eep.twiClock100kHz);
  refreshDeviceData();
  setup140();
  initDisplay();
}

void setup140() {
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  buzzInit(ENABLE_BUZ);
  initBmp();
  getAltitudeM(); // throw away first value
  vibe.begin();
  vibe.selectLibrary(1);
  vibe.setMode(DRV2605_MODE_INTTRIG);

  vibrateNotify();

  if (button_top.isPressedRaw()) {
    // Switch modes
    // 0=CHILL 1=SPORT 2=LUDICROUS?!
    if (deviceData.performance_mode == 0) {
      deviceData.performance_mode = 1;
    } else {
      deviceData.performance_mode = 0;
    }
    writeDeviceData();
    unsigned int notify_melody[] = { 900, 1976 };
    playMelody(notify_melody, 2);
  }
}

// main loop - everything runs in threads
void loop() {
  Watchdog.reset();
  // from WebUSB to both Serial & webUSB
  if (usb_web.available()) parse_usb_serial();
  threads.run();
}

void checkButtons() {
  button_top.check();
}

// disarm, remove cruise, alert, save updated stats
void disarmSystem() {
  throttlePWM = ESC_DISARMED_PWM;
  esc.writeMicroseconds(ESC_DISARMED_PWM);
  //Serial.println(F("disarmed"));

  // reset smoothing
  potBuffer.clear();
  prevPotLvl = 0;

  unsigned int disarm_melody[] = { 2093, 1976, 880 };
  unsigned int disarm_vibes[] = { 70, 33, 0 };

  armed = false;
  removeCruise(false);

  ledBlinkThread.enabled = true;
  updateDisplay();
  runVibe(disarm_vibes, 3);
  playMelody(disarm_melody, 3);

  bottom_bg_color = DEFAULT_BG_COLOR;
  display.fillRect(0, 93, 160, 40, bottom_bg_color);

  // update armed_time
  refreshDeviceData();
  deviceData.armed_time += round(armedSecs / 60);  // convert to mins
  writeDeviceData();
  delay(1000);  // TODO just disable button thread // dont allow immediate rearming
}

// The event handler for the the buttons
void handleButtonEvent(AceButton* /* btn */, uint8_t eventType, uint8_t /* st */) {
  switch (eventType) {
  case AceButton::kEventDoubleClicked:
    if (armed) {
      disarmSystem();
    } else if (throttleSafe()) {
      armSystem();
    } else {
      handleArmFail();
    }
    break;
  case AceButton::kEventLongPressed:
    if (armed) {
      if (cruising) {
        removeCruise(true);
      } else {
        setCruise();
      }
    } else {
      // show stats screen?
    }
    break;
  case AceButton::kEventLongReleased:
    break;
  }
}

// inital button setup and config
void initButtons() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);

  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2500);
  buttonConfig->setDoubleClickDelay(600);
}

// inital screen setup and config
void initDisplay() {
  display.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab

  pinMode(TFT_LITE, OUTPUT);
  resetDisplay();
  displayMeta();
  digitalWrite(TFT_LITE, HIGH);  // Backlight on
  delay(2500);
}

void resetDisplay() {
  display.fillScreen(DEFAULT_BG_COLOR);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextWrap(true);

  display.setRotation(deviceData.screen_rotation);  // 1=right hand, 3=left hand
}

// read throttle and send to hub
void handleThrottle() {
  if (!armed) return;  // safe

  armedSecs = (millis() - armedAtMilis) / 1000;  // update time while armed

  static int maxPWM = ESC_MAX_PWM;
  pot.update();
  int potRaw = pot.getValue();

  if (cruising) {
    unsigned long cruisingSecs = (millis() - cruisedAtMilis) / 1000;

    if (cruisingSecs >= CRUISE_GRACE && potRaw > POT_SAFE_LEVEL) {
      removeCruise(true);  // deactivate cruise
    } else {
      throttlePWM = mapf(cruisedPotVal, 0, 4095, ESC_MIN_PWM, maxPWM);
    }
  } else {
    // no need to save & smooth throttle etc when in cruise mode (& pot == 0)
    potBuffer.push(potRaw);

    int potLvl = 0;
    for (decltype(potBuffer)::index_t i = 0; i < potBuffer.size(); i++) {
      potLvl += potBuffer[i] / potBuffer.size();  // avg
    }

  // runs ~40x sec
  // 1000 diff in pwm from 0
  // 1000/6/40
    if (deviceData.performance_mode == 0) {  // chill mode
      potLvl = limitedThrottle(potLvl, prevPotLvl, 50);
      maxPWM = 1850;  // 85% interpolated from 1030 to 1990
    } else {
      potLvl = limitedThrottle(potLvl, prevPotLvl, 120);
      maxPWM = ESC_MAX_PWM;
    }
    // mapping val to min and max pwm
    throttlePWM = mapf(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  }

  esc.writeMicroseconds(throttlePWM);  // using val as the signal to esc
}

// get the PPG ready to fly
bool armSystem() {
  unsigned int arm_melody[] = { 1760, 1976, 2093 };
  unsigned int arm_vibes[] = { 70, 33, 0 };

  armed = true;
  esc.writeMicroseconds(ESC_DISARMED_PWM);  // initialize the signal to low

  ledBlinkThread.enabled = false;
  armedAtMilis = millis();
  armAltM = getAltitudeM();

  setLEDs(HIGH);
  runVibe(arm_vibes, 3);
  playMelody(arm_melody, 3);

  bottom_bg_color = ARMED_BG_COLOR;
  display.fillRect(0, 93, 160, 40, bottom_bg_color);

  return true;
}


// Returns true if the throttle/pot is below the safe threshold
bool throttleSafe() {
  pot.update();
  if (pot.getValue() < POT_SAFE_LEVEL) {
    return true;
  }
  return false;
}

// convert barometer data to altitude in meters
float getAltitudeM() {
  bmp.performReading();
  ambientTempC = bmp.temperature;
  float altitudeM = bmp.readAltitude(deviceData.sea_pressure);
  aglM = altitudeM - armAltM;
  return altitudeM;
}

/********
 *
 * Display logic
 *
 *******/
bool screen_wiped = false;

// show data on screen and handle different pages
void updateDisplay() {
  if (!screen_wiped) {
    display.fillScreen(WHITE);
    screen_wiped = true;
  }
  //Serial.print("v: ");
  //Serial.println(volts);

  displayPage0();
  //dispValue(kWatts, prevKilowatts, 4, 1, 10, /*42*/55, 2, BLACK, DEFAULT_BG_COLOR);
  //display.print("kW");

  display.setTextColor(BLACK);
  float avgVoltage = getBatteryVoltSmoothed();
  batteryPercent = getBatteryPercent(avgVoltage);  // multi-point line
  // change battery color based on charge
  int batt_width = map((int)batteryPercent, 0, 100, 0, 108);
  display.fillRect(0, 0, batt_width, 36, batt2color(batteryPercent));

  if (avgVoltage < BATT_MIN_V) {
    if (batteryFlag) {
      batteryFlag = false;
      display.fillRect(0, 0, 108, 36, DEFAULT_BG_COLOR);
    }
    display.setCursor(12, 3);
    display.setTextSize(2);
    display.setTextColor(RED);
    display.println("BATTERY");

    if ( avgVoltage < 10 ) {
      display.print(" ERROR");
    } else {
      display.print(" DEAD");
    }
  } else {
    batteryFlag = true;
    display.fillRect(map(batteryPercent, 0,100, 0,108), 0, map(batteryPercent, 0,100, 108,0), 36, DEFAULT_BG_COLOR);
  }
  // cross out battery box if battery is dead
  if (batteryPercent <= 5) {
    display.drawLine(0, 1, 106, 36, RED);
    display.drawLine(0, 0, 108, 36, RED);
    display.drawLine(1, 0, 110, 36, RED);
  }
  dispValue(batteryPercent, prevBatteryPercent, 3, 0, 108, 10, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("%");

  // battery shape end
  //display.fillRect(102, 0, 6, 9, BLACK);
  //display.fillRect(102, 27, 6, 10, BLACK);

  display.fillRect(0, 36, 160, 1, BLACK);
  display.fillRect(108, 0, 1, 36, BLACK);
  display.fillRect(0, 92, 160, 1, BLACK);

  displayAlt();

  //dispValue(ambientTempF, prevAmbTempF, 3, 0, 10, 100, 2, BLACK, DEFAULT_BG_COLOR);
  //display.print("F");

  handleFlightTime();
  displayTime(throttleSecs, 8, 102, bottom_bg_color);

  //dispPowerCycles(104,100,2);
}

// display first page (voltage and current)
void displayPage0() {
  float avgVoltage = getBatteryVoltSmoothed();

  dispValue(avgVoltage, prevVolts, 5, 1, 84, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("V");

  dispValue(telemetryData.amps, prevAmps, 3, 0, 108, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("A");

  float kWatts = watts / 1000.0;
  kWatts = constrain(kWatts, 0, 50);

  dispValue(kWatts, prevKilowatts, 4, 1, 10, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kW");

  float kwh = wattsHoursUsed / 1000;
  dispValue(kwh, prevKwh, 4, 1, 10, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kWh");

  display.setCursor(30, 60);
  display.setTextSize(1);
  if (deviceData.performance_mode == 0) {
    display.setTextColor(BLUE);
    display.print("CHILL");
  } else {
    display.setTextColor(RED);
    display.print("SPORT");
  }
}

// display second page (mAh and armed time)
void displayPage1() {
  dispValue(telemetryData.volts, prevVolts, 5, 1, 84, 42, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("V");

  dispValue(telemetryData.amps, prevAmps, 3, 0, 108, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("A");

  float kwh = wattsHoursUsed / 1000;
  dispValue(kwh, prevKilowatts, 4, 1, 10, 71, 2, BLACK, DEFAULT_BG_COLOR);
  display.print("kWh");

  display.setCursor(30, 60);
  display.setTextSize(1);
  if (deviceData.performance_mode == 0) {
    display.setTextColor(BLUE);
    display.print("CHILL");
  } else {
    display.setTextColor(RED);
    display.print("SPORT");
  }
}

// displays number of minutes and seconds (since armed)
void displayTime(int val) {
  int minutes = val / 60;  // numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

  display.print(convertToDigits(minutes));
  display.print(":");
  display.print(convertToDigits(seconds));
}

// display altitude data on screen
void displayAlt() {
  float altiudeM = 0;
  // TODO make MSL explicit?
  if (armAltM > 0 && deviceData.sea_pressure != DEFAULT_SEA_PRESSURE) {  // MSL
    altiudeM = getAltitudeM();
  } else {  // AGL
    altiudeM = getAltitudeM() - armAltM;
  }

  // convert to ft if not using metric
  float alt = deviceData.metric_alt ? altiudeM : (round(altiudeM * 3.28084));

  dispValue(alt, lastAltM, 5, 0, 70, 102, 2, BLACK, bottom_bg_color);

  display.print(deviceData.metric_alt ? F("m") : F("ft"));
  lastAltM = alt;
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
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(message);
}

void setCruise() {
  // IDEA: fill a "cruise indicator" as long press activate happens
  // or gradually change color from blue to yellow with time
  if (!throttleSafe()) {  // using pot/throttle
    cruisedPotVal = pot.getValue();  // save current throttle val
    cruising = true;
    vibrateNotify();

    // update display to show cruise
    display.setCursor(70, 60);
    display.setTextSize(1);
    display.setTextColor(RED);
    display.print("CRUISE");

    unsigned int notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);

    bottom_bg_color = YELLOW;
    display.fillRect(0, 93, 160, 40, bottom_bg_color);

    cruisedAtMilis = millis();  // start timer
  }
}

void removeCruise(bool alert) {
  cruising = false;

  if (armed) {
    bottom_bg_color = ARMED_BG_COLOR;
  } else {
    bottom_bg_color = DEFAULT_BG_COLOR;
  }
  display.fillRect(0, 93, 160, 40, bottom_bg_color);

  display.setCursor(70, 60);
  display.setTextSize(1);
  display.setTextColor(DEFAULT_BG_COLOR);
  display.print("CRUISE");
  if (alert) {
    vibrateNotify();

    if (ENABLE_BUZ) {
      tone(BUZ_PIN, 500, 100);
      delay(250);
      tone(BUZ_PIN, 500, 100);
    }
  }
}

unsigned long prevPwrMillis = 0;

void trackPower() {
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (armed) {
    wattsHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}
