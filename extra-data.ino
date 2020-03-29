// Copyright 2020 <Zach Whitehead>
// OpenPPG

// ** Logic for Saved Data **
void refreshDeviceData() {
  uint8_t tempBuf[sizeof(deviceData)];

  file.open(FILENAME, FILE_O_READ);
  // file existed
  if ( file ) {
    Serial.println(FILENAME " file exists. reading");

    uint32_t readlen;
    readlen = file.read(tempBuf, sizeof(deviceData));
    file.close();
    tempBuf[readlen] = 0;
    memcpy((uint8_t*)&deviceData, tempBuf, sizeof(deviceData));
    Serial.println("read into memory");
  } else {
    resetDeviceData();
  }

  Serial.println("Done");
}

void resetDeviceData() {
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

  Serial.print("Open " FILENAME " file to write ... ");

  if (file.open(FILENAME, FILE_O_WRITE)) {
    Serial.println("OK");
    file.write((uint8_t*)&deviceData, sizeof(deviceData));
    file.close();
    Serial.println("Written");
  } else {
    Serial.println("Failed!");
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

  if (doc["command"] && doc["command"] == "rbl") {
    enterUf2Dfu();
    return;
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
