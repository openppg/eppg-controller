// Copyright 2020 <Zach Whitehead>
// OpenPPG

// ** Logic for EEPROM **
# define EEPROM_OFFSET 0  // Address of first byte of EEPROM

// Constants for device data
const unsigned int DEFAULT_SCREEN_ROTATION = 3;
const bool DEFAULT_METRIC_TEMP = true;
const bool DEFAULT_METRIC_ALT = true;
const int DEFAULT_PERFORMANCE_MODE = 0;
const int DEFAULT_THEME = 0;  // 0=light, 1=dark
const int DEFAULT_BATT_SIZE = 4000;  // 4kw

// read saved data from EEPROM
void refreshDeviceData() {
  uint8_t tempBuf[sizeof(deviceData)];
  uint16_t crc;

  readDeviceDataFromEEPROM(tempBuf);
  memcpy((uint8_t*)&deviceData, tempBuf, sizeof(deviceData));
  crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);

  // If the CRC does not match, reset the device data
  if (crc != deviceData.crc) { resetDeviceData(); }

  // Update the revision if required
  updateRevisionIfRequired();
}

// Read device data from EEPROM
void readDeviceDataFromEEPROM(uint8_t* buffer) {
  #ifdef M0_PIO
    if (0 != eep.read(EEPROM_OFFSET, buffer, sizeof(deviceData))) {
      // Serial.println(F("error reading EEPROM"));
    }
  #elif RP_PIO
    EEPROM.get(EEPROM_OFFSET, buffer);
  #endif
}

// Update the revision if required
void updateRevisionIfRequired() {
  #ifdef RP_PIO
    if (deviceData.revision == 0) {
      deviceData.revision = 1;
      writeDeviceData();  // Save the updated revision to EEPROM
    }
  #endif
}

// One time freeRTOS task that wraps writeDeviceData()
void writeDeviceDataTask(void *pvParameters) {
  if (eepromSemaphore != NULL) {
    if (xSemaphoreTake(eepromSemaphore, (TickType_t)10) == pdTRUE) {
      // Your EEPROM write logic here
      writeDeviceData();
      // Once done, release the semaphore
      xSemaphoreGive(eepromSemaphore);
    }
  }

  // Delete the task after completion
  vTaskDelete(NULL);
}

// Write to EEPROM
void writeDeviceData() {
  deviceData.crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
  #ifdef M0_PIO
    if (0 != eep.write(EEPROM_OFFSET, (uint8_t*)&deviceData, sizeof(deviceData))) {
      //Serial.println(F("error writing EEPROM"));
    }
  #elif RP_PIO
    EEPROM.put(EEPROM_OFFSET, deviceData);
    EEPROM.commit();
  #endif
}

// Reset EEPROM and deviceData to factory defaults
void resetDeviceData() {
  deviceData = STR_DEVICE_DATA_140_V1();

  // Set the revision based on the arch and board revision
  #ifdef M0_PIO
    deviceData.revision = 0;
  #elif RP_PIO
    deviceData.revision = 2; // Default to new 2040 board revision
  #endif

  deviceData.version_major = VERSION_MAJOR;
  deviceData.version_minor = VERSION_MINOR;
  deviceData.screen_rotation = DEFAULT_SCREEN_ROTATION;
  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;
  deviceData.metric_temp = DEFAULT_METRIC_TEMP;
  deviceData.metric_alt = DEFAULT_METRIC_ALT;
  deviceData.performance_mode = DEFAULT_PERFORMANCE_MODE;
  deviceData.theme = DEFAULT_THEME;
  deviceData.batt_size = DEFAULT_BATT_SIZE;
  writeDeviceData();
}

// ** Logic for WebUSB **
void line_state_callback(bool connected) {
  digitalWrite(LED_SW, connected);

  if ( connected ) send_usb_serial();
}

// customized for sp140
void parse_usb_serial() {
#ifdef USE_TINYUSB
  const size_t capacity = JSON_OBJECT_SIZE(13) + 90;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
    // display.fillScreen(DEFAULT_BG_COLOR);
    //TODO display ("BL - UF2");
    rebootBootloader();
    return;  // run only the command
  }

  if (doc["major_v"] < 5) return;

  vTaskSuspend(updateDisplayTaskHandle); // Prevent display from updating while we're changing settings

  deviceData.screen_rotation = doc["screen_rot"].as<unsigned int>();  // "3/1"
  deviceData.sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  deviceData.metric_temp = doc["metric_temp"];  // true/false
  deviceData.metric_alt = doc["metric_alt"];  // true/false
  deviceData.performance_mode = doc["performance_mode"];  // 0,1
  deviceData.batt_size = doc["batt_size"];  // 4000
  deviceData.theme = doc["theme"];  // 0,1
  sanitizeDeviceData();
  writeDeviceData();
  resetRotation(deviceData.screen_rotation);  // Screen orientation may have changed
  setTheme(deviceData.theme);  // may have changed

  vTaskResume(updateDisplayTaskHandle);

  send_usb_serial();
#endif
}

bool sanitizeDeviceData() {
  bool changed = false;
  // Ensure screen rotation is either 1 or 3, default to 3
  if (deviceData.screen_rotation == 1 || deviceData.screen_rotation == 3) {
  } else {
    deviceData.screen_rotation = 3;
    changed = true;
  }

  // Ensure sea pressure is within acceptable limits, default to 1013.25
  // 337 is the air pressure at the top of Mt. Everest
  // 1065 is the air pressure at the dead sea. Pad both a bit
  if (deviceData.sea_pressure < 300 || deviceData.sea_pressure > 1200) {
    deviceData.sea_pressure = 1013.25;
    changed = true;
  }

  // Simply force metric_temp and metric_alt to be valid bool values
  if (deviceData.metric_temp != true && deviceData.metric_temp != false) {
    deviceData.metric_temp = true;
    changed = true;
  }
  if (deviceData.metric_alt != true && deviceData.metric_alt != false) {
    deviceData.metric_alt = true;
    changed = true;
  }
  // Ensure performance_mode is either 0 or 1, default to 0
  if (deviceData.performance_mode < 0 || deviceData.performance_mode > 1) {
    deviceData.performance_mode = 0;
    changed = true;
  }
  // Ensure battery size is within acceptable limits, default to 4000
  if (deviceData.batt_size < 0 || deviceData.batt_size > 10000) {
    deviceData.batt_size = 4000;
    changed = true;
  }
  // Ensure theme is either 0 or 1, default to 0
  if (deviceData.theme < 0 || deviceData.theme > 1) {
    deviceData.theme = 0; // 0=light, 1=dark
    changed = true;
  }
  return changed;
}

void send_usb_serial() {
#ifdef USE_TINYUSB
#ifdef M0_PIO
  const size_t capacity = JSON_OBJECT_SIZE(11) + 90;
  DynamicJsonDocument doc(capacity);

  doc["major_v"] = VERSION_MAJOR;
  doc["minor_v"] = VERSION_MINOR;
  doc["arch"] = "SAMD21";
  doc["screen_rot"] = deviceData.screen_rotation;
  doc["armed_time"] = deviceData.armed_time;
  doc["metric_temp"] = deviceData.metric_temp;
  doc["metric_alt"] = deviceData.metric_alt;
  doc["performance_mode"] = deviceData.performance_mode;
  doc["sea_pressure"] = deviceData.sea_pressure;
  doc["device_id"] = chipId();

  char output[256];
  serializeJson(doc, output);
  usb_web.println(output);
#elif RP_PIO
  StaticJsonDocument<256> doc; // <- a little more than 256 bytes in the stack

  doc["mj_v"].set(VERSION_MAJOR);
  doc["mi_v"].set(VERSION_MINOR);
  doc["arch"].set("RP2040");
  doc["scr_rt"].set(deviceData.screen_rotation);
  doc["ar_tme"].set(deviceData.armed_time);
  doc["m_tmp"].set(deviceData.metric_temp);
  doc["m_alt"].set(deviceData.metric_alt);
  doc["prf"].set(deviceData.performance_mode);
  doc["sea_p"].set(deviceData.sea_pressure);
  doc["thm"].set(deviceData.theme);
  doc["rev"].set(deviceData.revision);
  //doc["id"].set(chipId()); // webusb bug prevents anything over a certain size / this extra field from being sent

  char output[256];
  serializeJson(doc, output, sizeof(output));
  usb_web.println(output);
  usb_web.flush();
  //Serial.println(chipId());
#endif // M0_PIO/RP_PIO
#endif // USE_TINYUSB
}
