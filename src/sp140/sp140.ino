// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "Arduino.h"

#include "../../lib/crc.c"       // packet error checking
#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"          // device config
  // TODO find best SAMD21 FreeRTOS port
#else
  #include "../../inc/sp140/rp2040-config.h"         // device config
  #include <FreeRTOS.h>
  #include <task.h>
  #include <semphr.h>
  #include <map>
#endif

#include "../../inc/sp140/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include <Adafruit_DRV2605.h>    // haptic controller
#include <ArduinoJson.h>
#include <CircularBuffer.hpp>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <Servo.h>               // to control ESCs
#include <SPI.h>
#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>
#ifdef USE_TINYUSB
  #include "Adafruit_TinyUSB.h"
#endif

#ifdef M0_PIO
  #include <Adafruit_SleepyDog.h>  // watchdog
  #include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM
#elif RP_PIO
  // rp2040 specific libraries here
  #include <Adafruit_NeoPixel.h>
  #include <EEPROM.h>
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#endif

#include "../../inc/sp140/globals.h"  // device config

#include "../../inc/sp140/display.h"
#include "../../inc/sp140/altimeter.h"

using namespace ace_button;

UBaseType_t uxCoreAffinityMask0 = (1 << 0); // Core 0
UBaseType_t uxCoreAffinityMask1 = (1 << 1); // Core 1

HardwareConfig board_config;

Adafruit_DRV2605 vibe;

// USB WebUSB object
#ifdef USE_TINYUSB
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

ResponsiveAnalogRead* pot;
AceButton* button_top;
ButtonConfig* buttonConfig;

#ifdef M0_PIO
  extEEPROM eep(kbits_64, 1, 64);
#endif

CircularBuffer<float, 50> voltageBuffer;
CircularBuffer<int, 8> potBuffer;
# define PIN_NEOPIXEL 10

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
uint32_t led_color = LED_RED; // current LED color

bool armed = false;
uint32_t armedAtMillis = 0;
uint32_t cruisedAtMillisMilis = 0;
unsigned long armedSecs = 0;

TaskHandle_t blinkLEDTaskHandle = NULL;
TaskHandle_t throttleTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t trackPowerTaskHandle = NULL;
TaskHandle_t updateDisplayTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;

SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t tftSemaphore;


#pragma message "Warning: OpenPPG software is in beta"

void watchdogTask(void* parameter) {
  for (;;) {
    #ifndef OPENPPG_DEBUG
    watchdog_update();
    #endif
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100ms
  }
}


void blinkLEDTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    blinkLED();  // call blinkLED function
    delay(500);  // wait for 500ms
  }
  vTaskDelete(NULL); // should never reach this
}

void throttleTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    handleThrottle();  //
    delay(22);  // wait for 22ms
  }
  vTaskDelete(NULL); // should never reach this
}

void telemetryTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    handleTelemetry();
    delay(50);  // wait for 50ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void trackPowerTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    trackPower();
    delay(250);  // wait for 250ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void updateDisplayTask(void *pvParameters) { //TODO set core affinity to one core only
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    // TODO separate alt reading out to its own task. Avoid blocking display updates when alt reading is slow etc
    // TODO use queues to pass data between tasks (xQueueOverwrite)
    const float altitude = getAltitude(deviceData);
    updateDisplay(deviceData, telemetryData, altitude, armed, cruising, armedAtMillis);
    delay(250);  // wait for 250ms
  }
  vTaskDelete(NULL);  // should never reach this
}


void loadHardwareConfig() {
  if (deviceData.revision == 1) {
    board_config = v1_config;
  } else if (deviceData.revision == 2) {
    board_config = v2_config;
  } else {
    // Handle other cases or throw an error
  }
  pot = new ResponsiveAnalogRead(board_config.throttle_pin, false);
  button_top = new AceButton(board_config.button_top);
  buttonConfig = button_top->getButtonConfig();
}

void setupSerial() {
  Serial.begin(115200);
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
}

#ifdef USE_TINYUSB
void setupUSBWeb() {
  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
}
#endif

void printBootMessage() {
  Serial.print(F("Booting up (USB) V"));
  Serial.print(VERSION_MAJOR + "." + VERSION_MINOR);
}

void setupBarometer() {
  const int bmp_enabler = 9;
  pinMode(bmp_enabler, OUTPUT);
  digitalWrite(bmp_enabler, HIGH); // barometer fix for V2 board
}

void setupEEPROM() {
 #ifdef M0_PIO
  uint8_t eepStatus = eep.begin(eep.twiClock100kHz);
#elif RP_PIO
  EEPROM.begin(255);
#endif
}

void setupLED() {
  pinMode(board_config.led_sw, OUTPUT);   // set up the internal LED2 pin
  if(board_config.enable_neopixel){
    pixels.begin();
    setLEDColor(led_color);
  }
}

void setupAnalogRead() {
  analogReadResolution(12);   // M0 family chips provides 12bit ADC resolution
  pot->setAnalogResolution(4096);
}

void setupWatchdog() {
#ifndef OPENPPG_DEBUG
  #ifdef M0_PIO
    Watchdog.enable(5000);
  #elif RP_PIO
    watchdog_enable(4000, 1);
  #endif
#endif // OPENPPG_DEBUG
}


void upgradeDeviceRevisionInEEPROM() {
#ifdef RP_PIO
  if (deviceData.revision == M0) { // onetime conversion because default is 1
    deviceData.revision = V1;
    writeDeviceData();
  }
#endif
}

/**
 * Initializes the necessary components and configurations for the device setup.
 * This function is called once at the beginning of the program execution.
 */
void setup() {
  setupSerial();
#ifdef USE_TINYUSB
  setupUSBWeb();
#endif
  printBootMessage();
  setupBarometer();
  setupEEPROM();
  refreshDeviceData();
  upgradeDeviceRevisionInEEPROM();
  loadHardwareConfig();
  setupLED();
  setupAnalogRead();
  initButtons();
  setupTasks();
  setupWatchdog();
  setup140();
#ifdef M0_PIO
  Watchdog.reset();
#endif
  setLEDColor(LED_YELLOW);
  setupDisplay(deviceData, board_config);
  if (button_top->isPressedRaw()) {
    modeSwitch(false);
  }
  vTaskResume(updateDisplayTaskHandle);
  setLEDColor(LED_GREEN);
}

// set up all the main threads/tasks with core 0 affinity
void setupTasks() {
  xTaskCreateAffinitySet(blinkLEDTask, "blinkLed", 200, NULL, 1, uxCoreAffinityMask1, &blinkLEDTaskHandle);
  xTaskCreateAffinitySet(throttleTask, "throttle", 1000, NULL, 3, uxCoreAffinityMask0, &throttleTaskHandle);
  xTaskCreateAffinitySet(telemetryTask, "TelemetryTask", 2048, NULL, 2, uxCoreAffinityMask0, &telemetryTaskHandle);
  xTaskCreateAffinitySet(trackPowerTask, "trackPower", 500, NULL, 2, uxCoreAffinityMask0, &trackPowerTaskHandle);
  xTaskCreateAffinitySet(updateDisplayTask, "updateDisplay", 2000, NULL, 1, uxCoreAffinityMask0, &updateDisplayTaskHandle);
  xTaskCreateAffinitySet(watchdogTask, "watchdog", 1000, NULL, 4, uxCoreAffinityMask0, &watchdogTaskHandle);

  if (updateDisplayTaskHandle != NULL) {
    vTaskSuspend(updateDisplayTaskHandle);  // Suspend the task immediately after creation
  }

  eepromSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(eepromSemaphore);
}

std::map<eTaskState, const char *> eTaskStateName { {eReady, "Ready"}, { eRunning, "Running" }, {eBlocked, "Blocked"}, {eSuspended, "Suspended"}, {eDeleted, "Deleted"} };
void ps() {
  int tasks = uxTaskGetNumberOfTasks();
  TaskStatus_t *pxTaskStatusArray = new TaskStatus_t[tasks];
  unsigned long runtime;
  tasks = uxTaskGetSystemState(pxTaskStatusArray, tasks, &runtime);
  Serial.printf("# Tasks: %d\n", tasks);
  Serial.println("ID, NAME, STATE, PRIO, CYCLES");
  for (int i=0; i < tasks; i++) {
    Serial.printf("%d: %-16s %-10s %d %lu\n", i, pxTaskStatusArray[i].pcTaskName, eTaskStateName[pxTaskStatusArray[i].eCurrentState], (int)pxTaskStatusArray[i].uxCurrentPriority, pxTaskStatusArray[i].ulRunTimeCounter);
  }
  delete[] pxTaskStatusArray;
}


void printTaskMemoryUsage(TaskHandle_t taskHandle, const char* taskName) {
    UBaseType_t stackHighWaterMark;
    if (taskHandle == NULL) {
        taskHandle = xTaskGetCurrentTaskHandle(); // Get current task handle if NULL is passed
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    printf("Task: %s\n", taskName);
    printf("Stack High Watermark: %lu words\n", stackHighWaterMark);
    printf("Estimated Memory Usage: %lu bytes\n", (configTOTAL_HEAP_SIZE - stackHighWaterMark * sizeof(StackType_t)));
    printf("-------------------------\n");
}

void psTop() {
  // Print memory usage for each task
  Serial.printf("\nLoop() - Free Stack Space: %d", uxTaskGetStackHighWaterMark(NULL));
  Serial.printf("\nblinkLEDTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(blinkLEDTaskHandle));
  Serial.printf("\nthrottleTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(throttleTaskHandle));
  Serial.printf("\ntelemetryTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(telemetryTaskHandle));
  Serial.printf("\ntrackPowerTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(trackPowerTaskHandle));
  Serial.printf("\nupdateDisplayTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(updateDisplayTaskHandle));
  Serial.println("");
}

void setup140() {
  esc.attach(board_config.esc_pin);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  initBuzz();
  Wire1.setSDA(A0); // Have to use Wire1 because pins are assigned that in hardware
  Wire1.setSCL(A1);
  setupAltimeter(board_config.alt_wire);
  vibePresent = initVibe();
}

// main loop - everything runs in threads
void loop() {

#ifdef M0_PIO
  Watchdog.reset(); // reset the watchdog timer (done in task for RP2040)
#endif

  // from WebUSB to both Serial & webUSB
#ifdef USE_TINYUSB
  if (!armed && usb_web.available()) parse_usb_serial();
#endif

  // more stable in main loop
  checkButtons();
  delay(5);
  //ps();
  //psTop();
}

void checkButtons() {
  button_top->check();
}

void printTime(const char* label) {
  Serial.print(label);
  Serial.println(millis());
}

void disarmESC() {
  throttlePWM = ESC_DISARMED_PWM;
  esc.writeMicroseconds(ESC_DISARMED_PWM);
}

// reset smoothing
void resetSmoothing() {
  potBuffer.clear();
  prevPotLvl = 0;
}

void resumeLEDTask() {
  vTaskResume(blinkLEDTaskHandle);  // blink LED while disarmed
}

void runDisarmAlert() {
  u_int16_t disarm_melody[] = { 2093, 1976, 880 };
  const unsigned int disarm_vibes[] = { 100, 0 };
  runVibe(disarm_vibes, 3);
  playMelody(disarm_melody, 3);
}

void updateArmedTime() {
  uint16_t newArmedTime = round(armedSecs / 60);
  if (newArmedTime <= UINT16_MAX - deviceData.armed_time) {
    deviceData.armed_time += newArmedTime;
  } else {
    // If adding the new time would cause an overflow, set the value to the maximum
    deviceData.armed_time = UINT16_MAX;
  }
}

void disarmSystem() {
  disarmESC(); // disarm the ESC by setting the signal to low value

  resetSmoothing(); // reset throttle smoothing values

  armed = false;
  removeCruise(false);

  resumeLEDTask(); // resume LED blinking

  runDisarmAlert();

  updateArmedTime();
  writeDeviceData();

  delay(500);  // TODO just disable button thread // dont allow immediate rearming
}

void toggleArm() {
  if (armed) {
    disarmSystem();
  } else if (throttleSafe()) {
    armSystem();
  } else {
    handleArmFail();
  }
}

void toggleCruise() {
  if (armed) {
    if (cruising) {
      removeCruise(true);
    } else if (throttleSafe()) {
      modeSwitch(true);
    } else {
      setCruise();
    }
  } else {
    // show stats screen
    vTaskSuspend(updateDisplayTaskHandle);
    displayMeta(deviceData);
    vTaskResume(updateDisplayTaskHandle);
  }
}

// Variable to track if a button was clicked
bool wasClicked = false;

// Timestamp when the button was released
unsigned long releaseTime = 0;

// Time threshold for LongClick after release (in milliseconds)
const unsigned long longClickThreshold = 3500; // adjust as necessary

void handleButtonEvent(AceButton* btn, uint8_t eventType, uint8_t /* st */) {
  switch (eventType) {
  case AceButton::kEventClicked:
    wasClicked = true;
    break;
  case AceButton::kEventReleased:
    if (wasClicked) {
      releaseTime = millis();
      wasClicked = false;
      //Serial.println("Button Released after Click");
    }
    break;
  case AceButton::kEventDoubleClicked:
    break;
  case AceButton::kEventLongPressed:
    if (!wasClicked && (millis() - releaseTime <= longClickThreshold)) {
      toggleArm(); //
      //Serial.println("Long Press after Click and Release");
    } else {
      toggleCruise();
      //Serial.println("Long Press");
    }
    break;
  }
}

// initial button setup and config
void initButtons() {
  pinMode(board_config.button_top, INPUT_PULLUP);

  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2000);
  buttonConfig->setClickDelay(300);
  //buttonConfig->setDoubleClickDelay(900);
  //buttonConfig->setDebounceDelay(100);
}

// read throttle and send to hub
// read throttle
void handleThrottle() {
  if (!armed) return;  // safe

  armedSecs = (millis() - armedAtMillis) / 1000;  // update time while armed

  static int maxPWM = ESC_MAX_PWM;
  pot->update();
  int potRaw = pot->getValue();

  if (cruising) {
    unsigned long cruisingSecs = (millis() - cruisedAtMillis) / 1000;

    if (cruisingSecs >= CRUISE_GRACE && potRaw > POT_SAFE_LEVEL) {
      removeCruise(true);  // deactivate cruise
    } else {
      throttlePWM = mapd(cruisedPotVal, 0, POT_MAX_VALUE, ESC_MIN_PWM, maxPWM);
    }
  } else {
    // no need to save & smooth throttle etc when in cruise mode (& pot == 0)
    potBuffer.push(potRaw);

    int potLvl = averagePotBuffer();

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
    throttlePWM = mapd(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  }

  esc.writeMicroseconds(throttlePWM);  // using val as the signal to esc
}

int averagePotBuffer() {
  int sum = 0;
  for (decltype(potBuffer)::index_t i = 0; i < potBuffer.size(); i++) {
    sum += potBuffer[i];
  }
  return sum / potBuffer.size();
}

// get the PPG ready to fly
bool armSystem() {
  uint16_t arm_melody[] = { 1760, 1976, 2093 };
  const unsigned int arm_vibes[] = { 70, 33, 0 };

  armed = true;
  esc.writeMicroseconds(ESC_DISARMED_PWM);  // initialize the signal to low

  //ledBlinkThread.enabled = false;
  armedAtMillis = millis();
  setGroundAltitude(deviceData);

  vTaskSuspend(blinkLEDTaskHandle);
  setLEDs(HIGH); // solid LED while armed
  runVibe(arm_vibes, 3);
  playMelody(arm_melody, 3);

  return true;
}


// Returns true if the throttle/pot is below the safe threshold
bool throttleSafe() {
  pot->update();
  if (pot->getValue() < POT_SAFE_LEVEL) {
    return true;
  }
  return false;
}


void setCruise() {
  // IDEA: fill a "cruise indicator" as long press activate happens
  // or gradually change color from blue to yellow with time
  if (!throttleSafe()) {  // using pot/throttle
    cruisedPotVal = pot->getValue();  // save current throttle val
    cruisedAtMillis = millis();  // start timer
    // throttle handle runs fast and a lot. need to set the timer before
    // setting cruise so its updated in time
    // TODO since these values are accessed in another task make sure memory safe
    cruising = true;
    vibrateNotify();

    uint16_t notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);
  }
}

void removeCruise(bool alert) {
  cruising = false;

  if (alert) {
    vibrateNotify();

    if (ENABLE_BUZ) {
      uint16_t notify_melody[] = { 500, 500 };
      playMelody(notify_melody, 2);
    }
  }
}

unsigned long prevPwrMillis = 0;

void trackPower() {
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (armed) {
    wattHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}
