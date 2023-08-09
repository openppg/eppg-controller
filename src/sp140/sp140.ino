// Copyright 2020 <Zach Whitehead>
// OpenPPG

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
#include <CircularBuffer.h>      // smooth out readings
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
  #include <EEPROM.h>
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#endif


#include "../../inc/sp140/globals.h"  // device config

//#include "../../inc/sp140/utilities.h"
#include "../../inc/sp140/display.h"
#include "../../inc/sp140/altimeter.h"

using namespace ace_button;

Adafruit_DRV2605 vibe;

// USB WebUSB object
#ifdef USE_TINYUSB
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button_top(BUTTON_TOP);
ButtonConfig* buttonConfig = button_top.getButtonConfig();
#ifdef M0_PIO
  extEEPROM eep(kbits_64, 1, 64);
#endif

CircularBuffer<float, 50> voltageBuffer;
CircularBuffer<int, 8> potBuffer;

bool armed = false;
bool use_hub_v2 = true;
int page = 0;
uint32_t armedAtMillis = 0;
uint32_t cruisedAtMilisMilis = 0;
unsigned int armedSecs = 0;
unsigned int last_throttle = 0;

TaskHandle_t checkButtonTaskHandle = NULL;
TaskHandle_t blinkLEDTaskHandle = NULL;
TaskHandle_t throttleTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t trackPowerTaskHandle = NULL;
TaskHandle_t updateDisplayTaskHandle = NULL;

SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t tftSemaphore;

#pragma message "Warning: OpenPPG software is in beta"

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
  vTaskDelete(NULL); // should never reach this
}

void trackPowerTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    trackPower();  
    delay(250);  // wait for 250ms
  }
  vTaskDelete(NULL); // should never reach this
}

void checkButtonTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    checkButtons();  
    delay(5);  // wait for 5ms
  }
  vTaskDelete(NULL); // should never reach this
}

void updateDisplayTask(void *pvParameters) { //TODO set core affinity to one core only
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    // TODO separate alt reading out to its own task. Avoid blocking display updates when alt reading is slow etc 
    // TODO use queues to pass data between tasks (xQueueOverwrite)
    const float altitude = getAltitude(deviceData); 
    updateDisplay( deviceData, telemetryData, altitude, armed, cruising, armedAtMillis);
    delay(250);  // wait for 250ms
  }
  vTaskDelete(NULL); // should never reach this
}

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);

#ifdef USE_TINYUSB
  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
#endif

  //Serial.print(F("Booting up (USB) V"));
  //Serial.print(VERSION_MAJOR + "." + VERSION_MINOR);

  pinMode(LED_SW, OUTPUT);   // set up the internal LED2 pin

  analogReadResolution(12);     // M0 family chip provides 12bit resolution
  pot.setAnalogResolution(4096);
  unsigned int startup_vibes[] = { 27, 27, 0 };
  initButtons();
  setupTasks();

#ifdef M0_PIO
  Watchdog.enable(5000);
  uint8_t eepStatus = eep.begin(eep.twiClock100kHz);
#elif RP_PIO
  watchdog_enable(5000, 1);
  EEPROM.begin(512);
#endif
  refreshDeviceData();
  setup140();
#ifdef M0_PIO
  Watchdog.reset();
#endif
  setupAltimeter();

  setupDisplay(deviceData);
  if (button_top.isPressedRaw()) {
    modeSwitch(false);
  }
  delay(2000);
  vTaskResume(updateDisplayTaskHandle);
}

// set up all the threads/tasks
void setupTasks() {
  xTaskCreate(
    checkButtonTask,  // the function that implements the task
    "checkbutton",  // a name you can use for debugging
    1000,  // stack size in words, not bytes. For the AVR Arduino, this is the number of bytes.
    NULL,  // parameters passed into the task
    1,  // priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    &checkButtonTaskHandle);  // used to pass back a handle by which the created task can be referenced.

  xTaskCreate(blinkLEDTask, "blinkLed", 200, NULL, 4, &blinkLEDTaskHandle);
  xTaskCreate(throttleTask, "throttle", 1000, NULL, 1, &throttleTaskHandle);
  xTaskCreate(telemetryTask, "telemetry", 1000, NULL, 2, &telemetryTaskHandle);
  xTaskCreate(trackPowerTask, "trackPower", 1000, NULL, 3, &trackPowerTaskHandle);
  xTaskCreate(updateDisplayTask, "updateDisplay", 2000, NULL, 3, &updateDisplayTaskHandle);

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
  tasks = uxTaskGetSystemState( pxTaskStatusArray, tasks, &runtime );
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
  Serial.printf("\ncheckButtonTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(checkButtonTaskHandle));
  Serial.printf("\nupdateDisplayTask - Free Stack Space: %d", uxTaskGetStackHighWaterMark(updateDisplayTaskHandle));
  Serial.println("");
}

void setup140() {
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  initBuzz();
  setupAltimeter();
  vibePresent = initVibe();
}

// main loop - everything runs in threads
void loop() {
#ifdef M0_PIO
  Watchdog.reset();
#elif RP_PIO
  watchdog_update();
#endif

  // from WebUSB to both Serial & webUSB
#ifdef USE_TINYUSB
  if (!armed && usb_web.available()) parse_usb_serial();
#endif

  delay(100);
  //ps();
  //psTop();
}

long last_btn_check = 0;

void checkButtons() {
  button_top.check();
  //Serial.println(millis() - last_btn_check);
  last_btn_check = millis();
}

// disarm, remove cruise, alert, save updated stats
void disarmSystem() {
  throttlePWM = ESC_DISARMED_PWM;
  esc.writeMicroseconds(ESC_DISARMED_PWM);
  //Serial.println(F("disarmed"));

  // reset smoothing
  potBuffer.clear();
  prevPotLvl = 0;

  u_int16_t disarm_melody[] = { 2093, 1976, 880 };
  unsigned int disarm_vibes[] = { 100, 0 };

  armed = false;
  removeCruise(false);

  vTaskResume(blinkLEDTaskHandle);  // blink LED while disarmed
  runVibe(disarm_vibes, 3);
  playMelody(disarm_melody, 3);

  // update armed_time
  refreshDeviceData();
  deviceData.armed_time += round(armedSecs / 60);  // convert to mins
  xTaskCreate(writeDeviceDataTask, "EEPROMWrite", 128, NULL, 1, NULL);

  delay(1000);  // TODO just disable button thread // dont allow immediate rearming
    // probably not possible now that it takes longer to arm with button sequence?
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
    // displayMeta();
    // delay(2000);
    // resetDisplay();
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
      Serial.println("Button Released after Click");
    }
    break;
  case AceButton::kEventDoubleClicked:
    break;
  case AceButton::kEventLongPressed:
    if (!wasClicked && (millis() - releaseTime <= longClickThreshold)) {
      toggleArm(); //
      Serial.println("Long Press after Click and Release");
    }
    else {
      toggleCruise();
      Serial.println("Long Press");
    }
    break;
  }
}

// initial button setup and config
void initButtons() {
  pinMode(BUTTON_TOP, INPUT_PULLUP);

  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2500);
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
  pot.update();
  int potRaw = pot.getValue();

  if (cruising) {
    unsigned long cruisingSecs = (millis() - cruisedAtMilis) / 1000;

    if (cruisingSecs >= CRUISE_GRACE && potRaw > POT_SAFE_LEVEL) {
      removeCruise(true);  // deactivate cruise
    } else {
      throttlePWM = mapd(cruisedPotVal, 0, 4095, ESC_MIN_PWM, maxPWM);
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
    throttlePWM = mapd(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  }

  esc.writeMicroseconds(throttlePWM);  // using val as the signal to esc
}

// get the PPG ready to fly
bool armSystem() {
  uint16_t arm_melody[] = { 1760, 1976, 2093 };
  unsigned int arm_vibes[] = { 70, 33, 0 };

  armed = true;
  esc.writeMicroseconds(ESC_DISARMED_PWM);  // initialize the signal to low

  //ledBlinkThread.enabled = false;
  armedAtMillis = millis();
  setGroundAltitude(deviceData);

  setLEDs(HIGH);
  vTaskSuspend(blinkLEDTaskHandle);  // solid LED while armed

  runVibe(arm_vibes, 3);
  playMelody(arm_melody, 3);

  //bottom_bg_color = ARMED_BG_COLOR;

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


void setCruise() {
  // IDEA: fill a "cruise indicator" as long press activate happens
  // or gradually change color from blue to yellow with time
  if (!throttleSafe()) {  // using pot/throttle
    cruisedPotVal = pot.getValue();  // save current throttle val
    cruising = true;
    vibrateNotify();

    uint16_t notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);

    cruisedAtMilis = millis();  // start timer
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
