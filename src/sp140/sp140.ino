// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "Arduino.h"

#include "../../lib/crc.c"       // packet error checking
#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"          // device config
  // TODO: find best SAMD21 FreeRTOS port
#elif RP_PIO
  #include "../../inc/sp140/rp2040-config.h"         // device config
  #include <FreeRTOS.h>
  #include <task.h>
  #include <semphr.h>
  #include <map>
#elif CAN_PIO
  #include "../../inc/sp140/esp32s3-config.h"
#endif

#include "../../inc/sp140/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include <Adafruit_NeoPixel.h>   // LEDs
#include <ArduinoJson.h>
#include <CircularBuffer.hpp>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <SPI.h>
#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>
#ifdef USE_TINYUSB
  #include "Adafruit_TinyUSB.h"
#endif

#ifdef M0_PIO
  // SAMD21 specific libraries here
  #include <Adafruit_SleepyDog.h>  // watchdog
  #include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM
#elif RP_PIO
  // rp2040 specific libraries here
  #include <EEPROM.h>
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#elif CAN_PIO
  // ESP32S3 (CAN) specific libraries here
  #include "EEPROM.h"
#endif

#include "../../inc/sp140/globals.h"  // device config
#include "../../inc/sp140/esc.h"
#include "../../inc/sp140/display.h"
#include "../../inc/sp140/altimeter.h"
#include "../../inc/sp140/vibration.h"

#include <SineEsc.h>
#include <CanardAdapter.h>

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "driver/twai.h"

using namespace ace_button;

UBaseType_t uxCoreAffinityMask0 = (1 << 0); // Core 0
UBaseType_t uxCoreAffinityMask1 = (1 << 1); // Core 1

HardwareConfig board_config;

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

Adafruit_NeoPixel pixels(1, LED_BUILTIN, NEO_GRB + NEO_KHZ800);
uint32_t led_color = LED_RED; // current LED color

// New enum for device state
enum DeviceState {
  DISARMED,
  ARMED,
  ARMED_CRUISING
};

// Global variable for device state
volatile DeviceState currentState = DISARMED;

SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t tftSemaphore;
SemaphoreHandle_t stateMutex;

void changeDeviceState(DeviceState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    DeviceState oldState = currentState;
    currentState = newState;
    switch (newState) {
      case DISARMED:
        disarmSystem();
        break;
      case ARMED:
        if (oldState == DISARMED) {
          // Perform full arm sequence when transitioning from DISARMED to ARMED
          armSystem();
        } else if (oldState == ARMED_CRUISING) {
          // When transitioning from ARMED_CRUISING to ARMED, only remove cruise
          // This avoids re-arming the system unnecessarily
           afterCruiseEnd();
        }
        break;
      case ARMED_CRUISING:
        if (oldState == ARMED) {
          afterCruiseStart();
        } else {
          // Should never happen
          // Do nothing if not already in ARMED state
          currentState = oldState;  // Revert to the previous state
        }
        break;
    }
    xSemaphoreGive(stateMutex);
  }
}

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

#define RX_PIN 4
#define TX_PIN 5
#define POTENTIOMETER_PIN 8
#define LOCAL_NODE_ID 0x01

static CanardAdapter adapter;
static uint8_t memory_pool[1024] __attribute__((aligned(8)));
static SineEsc esc(adapter);
static bool twaiDriverInstalled = false;
static unsigned long lastThrottleUpdate = 0;
static const unsigned long THROTTLE_UPDATE_INTERVAL = 250; // 250ms
unsigned long lastDisarmTime = 0;
const unsigned long DISARM_COOLDOWN = 500; // 500ms cooldown

#pragma message "Warning: OpenPPG software is in beta"

void watchdogTask(void* parameter) {
  for (;;) {
    #ifndef OPENPPG_DEBUG
      #ifdef RP_PIO
      watchdog_update();
      #elif CAN_PIO
      #endif
    #endif
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100ms
  }
}


void blinkLEDTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    //blinkLED();  // call blinkLED function
    Serial.println("blinkLEDTask");
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
    readESCTelemetry();
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

void updateDisplayTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {
    // TODO: separate alt reading out to its own task. Avoid blocking display updates when alt reading is slow etc
    // TODO: use queues to pass data between tasks (xQueueOverwrite)const float altitude = getAltitude(deviceData);
    bool isArmed = (currentState != DISARMED);
    bool isCruising = (currentState == ARMED_CRUISING);
    updateDisplay(deviceData, escTelemetryData, altitude, isArmed, isCruising, armedAtMillis);
    delay(250);
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

void testTask(void *pvParameters) {
  for (;;) {
    Serial.println(".");
    delay(500);
  }
}

//just for testing
void setup() {
  Serial.begin(115200);
  delay(100);  // Give some time for USB CDC to initialize
  while (!Serial) delay(10);  // Wait for Serial to be ready
  delay(100);
  Serial.println("ESP32-S3 is ready!");
  // simple task that prints "." to serial every 1000ms
  TaskHandle_t testTaskHandle = NULL;
  xTaskCreate(testTask, "TestTask", 1000, NULL, 1, &testTaskHandle);
  setupTWAI();
}

static bool setupTWAI() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
                                        (gpio_num_t)TX_PIN,
                                        (gpio_num_t)RX_PIN,
                                        TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("Driver installed");
    } else {
        Serial.println("Failed to install driver");
        return false;
    }

    if (twai_start() == ESP_OK) {
        Serial.println("Driver started");
    } else {
        Serial.println("Failed to start driver");
        return false;
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA
                                | TWAI_ALERT_ERR_PASS
                                | TWAI_ALERT_BUS_ERROR
                                | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        Serial.println("CAN Alerts reconfigured");
    } else {
        Serial.println("Failed to reconfigure alerts");
        return false;
    }

    return true;
}

static void dumpMessages(void) {
    const SineEscModel &model = esc.getModel();

    if (model.hasGetHardwareInfoResponse) {
        Serial.println("Got HwInfo response");

        const sine_esc_GetHwInfoResponse *b = &model.getHardwareInfoResponse;
        Serial.print("\thardware_id: ");
        Serial.println(b->hardware_id, HEX);
        Serial.print("\tbootloader_version: ");
        Serial.println(b->bootloader_version, HEX);
        Serial.print("\tapp_version: ");
        Serial.println(b->app_version, HEX);
    }

    if (model.hasSetThrottleSettings2Response) {
        Serial.println("Got SetThrottleSettings2 response");
        const sine_esc_SetThrottleSettings2Response *b = &model.setThrottleSettings2Response;

        Serial.print("\trecv_pwm: ");
        Serial.println(b->recv_pwm);

        Serial.print("\tcomm_pwm: ");
        Serial.println(b->comm_pwm);

        Serial.print("\tspeed: ");
        Serial.println(b->speed);

        Serial.print("\tcurrent: ");
        Serial.println(b->current);

        Serial.print("\tbus_current: ");
        Serial.println(b->bus_current);

        Serial.print("\tvoltage: ");
        Serial.println(b->voltage);

        Serial.print("\tv_modulation: ");
        Serial.println(b->v_modulation);

        Serial.print("\tmos_temp: ");
        Serial.println(b->mos_temp);

        Serial.print("\tcap_temp: ");
        Serial.println(b->cap_temp);

        Serial.print("\tmcu_temp: ");
        Serial.println(b->mcu_temp);

        Serial.print("\trunning_error: ");
        Serial.println(b->running_error);

        Serial.print("\tselfcheck_error: ");
        Serial.println(b->selfcheck_error);

        Serial.print("\tmotor_temp: ");
        Serial.println(b->motor_temp);

        Serial.print("\ttime_10ms: ");
        Serial.println(b->time_10ms);
    }

    if (model.hasSetRotationSpeedSettingsResponse) {
        Serial.println("Got SetRotationSpeedSettings response");
    }
}

static void periodicDumpMessages(bool resetTimer, unsigned long dumpPeriod_ms) {
    static unsigned long lastMillis = millis();
    unsigned long now = millis();

    if (resetTimer) {
        lastMillis = now;
    } else if ((now - lastMillis) >= dumpPeriod_ms) {
        lastMillis = now;
        dumpMessages();
    }
}

/**
 * Initializes the necessary components and configurations for the device setup.
 * This function is called once at the beginning of the program execution.
 */
void setup2() {
  Serial.begin(115200);  // For debugging
}

void setup() {
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
  #ifdef RP_PIO
  xTaskCreateAffinitySet(blinkLEDTask, "blinkLed", 200, NULL, 1, uxCoreAffinityMask1, &blinkLEDTaskHandle);
  xTaskCreateAffinitySet(throttleTask, "throttle", 2048, NULL, 4, uxCoreAffinityMask0, &throttleTaskHandle);
  xTaskCreateAffinitySet(telemetryTask, "TelemetryTask", 2048, NULL, 3, uxCoreAffinityMask0, &telemetryTaskHandle);
  xTaskCreateAffinitySet(trackPowerTask, "trackPower", 500, NULL, 2, uxCoreAffinityMask0, &trackPowerTaskHandle);
  xTaskCreateAffinitySet(updateDisplayTask, "updateDisplay", 2000, NULL, 1, uxCoreAffinityMask0, &updateDisplayTaskHandle);
  xTaskCreateAffinitySet(watchdogTask, "watchdog", 1000, NULL, 5, uxCoreAffinityMask0, &watchdogTaskHandle);
 #else
  xTaskCreate(blinkLEDTask, "blinkLed", 200, NULL, 1, &blinkLEDTaskHandle);
  xTaskCreate(throttleTask, "throttle", 1000, NULL, 3, &throttleTaskHandle);
  xTaskCreate(telemetryTask, "TelemetryTask", 2048, NULL, 2, &telemetryTaskHandle);
  xTaskCreate(trackPowerTask, "trackPower", 500, NULL, 2, &trackPowerTaskHandle);
  xTaskCreate(updateDisplayTask, "updateDisplay", 2000, NULL, 1, &updateDisplayTaskHandle);
  xTaskCreate(watchdogTask, "watchdog", 1000, NULL, 5, &watchdogTaskHandle);
  #endif
  if (updateDisplayTaskHandle != NULL) {
    vTaskSuspend(updateDisplayTaskHandle);  // Suspend the task immediately after creation
  }

  eepromSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(eepromSemaphore);
  stateMutex = xSemaphoreCreateMutex();
}


void setup140() {
  #ifdef CAN_PIO
  // TODO: write to CAN bus
  #else
    initESC(board_config.esc_pin);
  #endif

  initBuzz();
  #ifdef RP_PIO
  Wire1.setSDA(A0); // Have to use Wire1 because pins are assigned that in hardware
  Wire1.setSCL(A1);
  #elif CAN_PIO
  const int SDA_PIN = 44;
  const int SCL_PIN = 41;
  Wire.setPins(SDA_PIN, SCL_PIN);
  #endif
  setupAltimeter(board_config.alt_wire);
  if (board_config.enable_vib) {
    initVibeMotor();
  }
}

// main loop - everything runs in threads
void loop() {

#ifdef M0_PIO
  Watchdog.reset(); // reset the watchdog timer (done in task for RP2040)
#endif

  // from WebUSB to both Serial & webUSB
#ifdef USE_TINYUSB
  if (currentState == DISARMED && usb_web.available()) parse_usb_serial();
#endif

  // more stable in main loop
  //checkButtons();
  delay(5);

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
  #ifdef CAN_PIO
  // TODO: write to CAN bus
  #else
    setESCThrottle(ESC_DISARMED_PWM);
  #endif
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
  const unsigned int disarm_vibes[] = { 78, 49 };
  runVibePattern(disarm_vibes, 2);
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
  disarmESC();
  resetSmoothing();
  resumeLEDTask();
  runDisarmAlert();
  updateArmedTime();
  writeDeviceData();

  delay(500);  // TODO: just disable button thread // dont allow immediate rearming
  // Set the last disarm time
  lastDisarmTime = millis();
}

void toggleArm() {
  if (currentState == DISARMED) {
    // Check if enough time has passed since last disarm
    if (millis() - lastDisarmTime >= DISARM_COOLDOWN) {
      if (!throttleEngaged()) {
        changeDeviceState(ARMED);
      } else {
        handleArmFail();
      }
    }
    // If we're still in the cooldown period, do nothing
  } else {
    changeDeviceState(DISARMED);
  }
}

void toggleCruise() {
  switch (currentState) {
    case ARMED:
      if (throttleEngaged()) {
        changeDeviceState(ARMED_CRUISING);
      } else {
        // Call modeSwitch when armed but throttle not engaged
        modeSwitch(false);
      }
      break;
    case ARMED_CRUISING:
      changeDeviceState(ARMED);  // This will now call removeCruise
      break;
    case DISARMED:
      // show stats screen
      vTaskSuspend(updateDisplayTaskHandle);
      displayMeta(deviceData);
      vTaskResume(updateDisplayTaskHandle);
      break;
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
  if (currentState == DISARMED) return;  // safe

  armedSecs = (millis() - armedAtMillis) / 1000;  // update time while armed

  static int maxPWM = ESC_MAX_PWM;
  pot->update();
  int potRaw = pot->getValue();

  int localThrottlePWM = ESC_DISARMED_PWM;

  if (currentState == ARMED_CRUISING) {
    unsigned long cruisingSecs = (millis() - cruisedAtMillis) / 1000;

    if (cruisingSecs >= CRUISE_GRACE && potRaw > POT_ENGAGEMENT_LEVEL) {
      changeDeviceState(ARMED);  // deactivate cruise
    } else {
      localThrottlePWM = mapd(cruisedPotVal, 0, POT_MAX_VALUE, ESC_MIN_PWM, maxPWM);
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
    localThrottlePWM = mapd(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  }
#ifdef CAN_PIO
// TODO: write to CAN bus
#else
  setESCThrottle(localThrottlePWM);  // using val as the signal to esc
#endif
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
  const unsigned int arm_vibes[] = { 1, 85, 1, 85, 1, 85, 1 };

  #ifdef CAN_PIO
  // TODO: write to CAN bus
  #else
    setESCThrottle(ESC_DISARMED_PWM);  // initialize the signal to low
  #endif

  //ledBlinkThread.enabled = false;
  armedAtMillis = millis();
  setGroundAltitude(deviceData);

  vTaskSuspend(blinkLEDTaskHandle);
  setLEDs(HIGH); // solid LED while armed
  runVibePattern(arm_vibes, 7);
  playMelody(arm_melody, 3);

  return true;
}

// Returns true if the throttle/pot is above the engagement threshold
bool throttleEngaged() {
  pot->update();
  if (pot->getRawValue() >= POT_ENGAGEMENT_LEVEL) {
    return true;
  }
  return false;
}

void afterCruiseStart() {
  cruisedPotVal = pot->getValue();
  cruisedAtMillis = millis();
  playCruiseSound();
  pulseVibeMotor();
}

void afterCruiseEnd() {
  cruisedPotVal = 0;
  playCruiseSound();
  pulseVibeMotor();
}

void playCruiseSound() {
  if (ENABLE_BUZ) {
    uint16_t notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);
  }
}

unsigned long prevPwrMillis = 0;

void trackPower() {
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (currentState != DISARMED) {
    wattHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}
