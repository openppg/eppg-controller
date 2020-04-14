// Copyright 2020 <Zach Whitehead>
// OpenPPG

#include "libraries/crc.c"       // packet error checking
#include "inc/config.h"          // device config
#include "inc/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include <bluefruit.h>
#include <Adafruit_DRV2605.h>    // haptic controller
#include <Adafruit_SSD1306.h>    // screen
#include "Adafruit_TinyUSB.h"
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

#define FILENAME "/openppg2.bin"

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

// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "openppg.github.io/openppg-config");

ResponsiveAnalogRead pot(THROTTLE_PIN, false);
AceButton button_top(BUTTON_TOP);
AceButton button_side(BUTTON_SIDE);
AdjustableButtonConfig buttonConfig;
File file(InternalFS);

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

// OTA DFU service
BLEDfu bledfu;

// Peripheral uart service
BLEUart bleuart;

// Central uart client
BLEClientUart clientUart;

BLEBas  blebas;  // battery
BLEDis  bledis;  // device information

#pragma message "Warning: OpenPPG software is in beta"

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_SW, OUTPUT);

  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);

  Serial.begin(115200);
  // while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.print(F("Booting up (USB) V"));
  Serial.println(VERSION_MAJOR + "." + VERSION_MINOR);

  pinMode(LED_SW, OUTPUT);      // set up the external LED pin
  pinMode(LED_2, OUTPUT);       // set up the internal LED2 pin
  pinMode(LED_3, OUTPUT);       // set up the internal LED3 pin

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

  butttonThread.onRun(checkButtons);
  butttonThread.setInterval(25);

  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(50);

  InternalFS.begin();
  refreshDeviceData();

  setupBluetooth();
}

void setupBluetooth() {
  // Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 1
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("OpenPPG Controller");

  bledis.setManufacturer("OpenPPG");
  bledis.setModel("Controller V2");
  bledis.begin();

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(cent_connect_callback);
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(cent_bleuart_rx_callback);

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(50);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);  // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(bleuart.uuid);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);   // 0 = Don't stop scanning after n seconds

  // Set up and start advertising
  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

// function to echo to both Serial and WebUSB
void echo_all(char chr) {  // from adafruit example
  Serial.write(chr);
  if ( chr == '\r' ) Serial.write('\n');

  usb_web.write(chr);
}

// main loop - everything runs in threads
void loop() {
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
  float percent = mapf(voltage, deviceData.min_batt_v, deviceData.max_batt_v, 0, 100);
  percent = constrain(percent, 0, 100);

  return round(percent);
}

int getInternalBatteryPercent() {
  float measuredvbat = analogRead(VBAT_PIN);
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

  delay(500);  // dont allow immediate rearming
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

  if ( clientUart.discovered() ) {
    clientUart.write((uint8_t*)&controlData, 8); // send to hub
    Serial.println("sent to hub");
  }
}

// read hub data if available and have it converted
void handleHubResonse() {
  int readSize = sizeof(STR_HUB2CTRL_MSG_V2);
  uint8_t serialData[readSize];

  while (clientUart.available() > 0) {
    memset(serialData, 0, sizeof(serialData));
    int size = clientUart.readBytes(serialData, sizeof(STR_HUB2CTRL_MSG_V2));
    receiveHubData(serialData, size);
  }
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


/*------------------------------------------------------------------*/
/* Peripheral
 *------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(uint16_t conn_handle) {
  (void) conn_handle;

  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(str);

  if ( clientUart.discovered() ) {
    clientUart.print(str);
  } else {
    bleuart.println("[Prph] Central role not connected");
  }
}

/*------------------------------------------------------------------*/
/* Central
 *------------------------------------------------------------------*/
void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}

void cent_connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Cent] Connected to ");
  Serial.println(peer_name);

  if ( clientUart.discover(conn_handle) ) {
    // Enable TXD's notify
    clientUart.enableTXD();
  } else {
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }
}

void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println("[Cent] Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param cent_uart Reference object to the service where the data
 * arrived. In this example it is clientUart
 */
void cent_bleuart_rx_callback(BLEClientUart& cent_uart) {
  char str[20+1] = { 0 };
  cent_uart.read(str, 20);

  Serial.print("[Cent] RX: ");
  Serial.println(str);

  if ( bleuart.notifyEnabled() ) {
    // Forward data from our peripheral to Mobile
    bleuart.print(str);
  } else {
    // response with no prph message
    clientUart.println("[Cent] Peripheral role not connected");
  }
}

void drawSymbol(uint16_t x, uint16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t char_size){
  if( (c>=32) && (c<=126) ){ //If it's 33-126 then use standard mono 18 font
      display.setFont();
  } else {
    display.setFont(&Symbol18);//Otherwise use special symbol font
    if (c>126) {      //Remap anything above 126 to be in the range 32 and upwards
      c-=(127-32);
    }
  }
  display.drawChar(x,y,c,color,bg,char_size);
}
