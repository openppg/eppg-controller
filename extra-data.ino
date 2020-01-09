// Copyright 2020 <Zach Whitehead>
// OpenPPG

// ** Logic for EEPROM **

void refreshDeviceData() {
  int offset = 0;

  uint8_t tempBuf[sizeof(deviceData)];
  if (0 != eep.read(offset, tempBuf, sizeof(deviceData))) {
    //Serial.println(F("error reading EEPROM"));
  }
  memcpy((uint8_t*)&deviceData, tempBuf, sizeof(deviceData));
  uint16_t crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
  if(deviceData.version_major == 4 && deviceData.version_minor == 0){
    bool upgraded = upgradeDeviceData();
    // TODO: add upgrade complete melody
  } else if (crc != deviceData.crc) {
    //Serial.print(F("Memory CRC mismatch. Resetting"));
    resetDeviceData();
    return;
  }
}

bool upgradeDeviceData(){
  uint8_t tempBuf[sizeof(deviceDataV1)];

  if (0 != eep.read(0, tempBuf, sizeof(deviceDataV1))) {
    return false;
  }
  memcpy((uint8_t*)&deviceDataV1, tempBuf, sizeof(deviceDataV1));
  uint16_t crc = crc16((uint8_t*)&deviceDataV1, sizeof(deviceDataV1) - 2);
  if (crc != deviceData.crc) {
    //Serial.print(F("Memory CRC mismatch. Resetting"));
    resetDeviceData();
    return false;
  }

  deviceData = STR_DEVICE_DATA_V2();
  deviceData.version_major = VERSION_MAJOR;
  deviceData.version_minor = VERSION_MINOR;
  deviceData.screen_rotation = 2;
  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;  // 1013.25 mbar
  deviceData.metric_temp = true;
  deviceData.metric_alt = true;

  deviceData.armed_time = deviceDataV1.armed_time;

  writeDeviceData();

  return true;
}
void resetDeviceData(){
    deviceData = STR_DEVICE_DATA_V2();
    deviceData.version_major = VERSION_MAJOR;
    deviceData.version_minor = VERSION_MINOR;
    deviceData.screen_rotation = 2;
    deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;  // 1013.25 mbar
    deviceData.metric_temp = true;
    deviceData.metric_alt = true;
    deviceData.min_batt_v = BATT_MIN_V;
    deviceData.max_batt_v = BATT_MAX_V;
    writeDeviceData();
}

void writeDeviceData() {
  deviceData.crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
  int offset = 0;

  if (0 != eep.write(offset, (uint8_t*)&deviceData, sizeof(deviceData))) {
    Serial.println(F("error writing EEPROM"));
  }
}

// ** Logic for WebUSB **

void line_state_callback(bool connected) {
  digitalWrite(LED_2, connected);

  if ( connected ) send_usb_serial();
}

void parse_usb_serial() {
  const size_t capacity = JSON_OBJECT_SIZE(11) + 90;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl"){
    rebootBootloader();
    return; // run only the command
  }

  deviceData.screen_rotation = doc["screen_rot"];  // "2/0"
  deviceData.sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  deviceData.metric_temp = doc["metric_temp"];  // true/false
  deviceData.metric_alt = doc["metric_alt"];  // true/false
  deviceData.min_batt_v = doc["min_batt_v"];  // 47.2v
  deviceData.max_batt_v = doc["max_batt_v"];  // 59.2v
  initDisplay();
  writeDeviceData();
  send_usb_serial();
}

void send_usb_serial() {
  const size_t capacity = JSON_OBJECT_SIZE(11) + 90;
  DynamicJsonDocument doc(capacity);

  doc["major_v"] = VERSION_MAJOR;
  doc["minor_v"] = VERSION_MINOR;
  doc["screen_rot"] = deviceData.screen_rotation;
  doc["armed_time"] = deviceData.armed_time;
  doc["metric_temp"] = deviceData.metric_temp;
  doc["metric_alt"] = deviceData.metric_alt;
  doc["min_batt_v"] = deviceData.min_batt_v;
  doc["max_batt_v"] = deviceData.max_batt_v;
  doc["sea_pressure"] = deviceData.sea_pressure;
  doc["device_id"] = chipId();

  char output[256];
  serializeJson(doc, output);
  usb_web.println(output);
}
