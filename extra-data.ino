// Copyright 2020 <Zach Whitehead>
// OpenPPG

// ** Logic for Saved Data **

// Loads the data from a file
void refreshDeviceData() {
  file.open(FILENAME, FILE_O_READ);
  if ( file ) {
    deviceData.crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
    const size_t capacity = JSON_OBJECT_SIZE(11) + 90;
    DynamicJsonDocument doc(capacity);

    file.open(FILENAME, FILE_O_READ);

    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      Serial.println(F("Failed to read file, using default configuration"));
      resetDeviceData();
    }
  } else {
    resetDeviceData();
  }
}

void resetDeviceData() {
  STR_DEVICE_DATA_V2 deviceData = STR_DEVICE_DATA_V2();

  deviceData.version_major = VERSION_MAJOR;
  deviceData.version_minor = VERSION_MINOR;
  deviceData.screen_rotation = 2;
  deviceData.sea_pressure = DEFAULT_SEA_PRESSURE;  // 1013.25 mbar
  deviceData.metric_temp = true;
  deviceData.metric_alt = true;
  deviceData.min_batt_v = BATT_MIN_V;
  deviceData.max_batt_v = BATT_MAX_V;
  Serial.println("reset memory");
  writeDeviceData();
}

void writeDeviceData() {
  deviceData.crc = crc16((uint8_t*)&deviceData, sizeof(deviceData) - 2);
  const size_t capacity = JSON_OBJECT_SIZE(11) + 112;
  DynamicJsonDocument doc(capacity);

  doc["version_major"] = deviceData.version_major;
  doc["version_minor"] = deviceData.version_minor;
  doc["armed_time"] = 2;

  Serial.print("Open " FILENAME " file to write ... ");
  char json_string[512];

  if (file.open(FILENAME, FILE_O_WRITE)) {
    Serial.println("OK");
    serializeJson(doc, file);
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


DynamicJsonDocument deviceDataToJson(STR_DEVICE_DATA_V2 data) {
  const size_t capacity = JSON_OBJECT_SIZE(11) + 112;
  DynamicJsonDocument doc(capacity);

  doc["major_v"] = VERSION_MAJOR;
  doc["minor_v"] = VERSION_MINOR;
  doc["screen_rot"] = data.screen_rotation;
  doc["armed_time"] = data.armed_time;
  doc["metric_temp"] = data.metric_temp;
  doc["metric_alt"] = data.metric_alt;
  doc["min_batt_v"] = data.min_batt_v;
  doc["max_batt_v"] = data.max_batt_v;
  doc["sea_pressure"] = data.sea_pressure;
  doc["device_id"] = chipId();
  doc["crc"] = crc16((uint8_t*)&data, sizeof(data) - 2);
  return doc;
}

STR_DEVICE_DATA_V2 jsonToDeviceData(DynamicJsonDocument doc, bool sanitize = true) {
  STR_DEVICE_DATA_V2 data = STR_DEVICE_DATA_V2();

  data.screen_rotation = doc["screen_rot"];  // "2/0"
  data.sea_pressure = doc["sea_pressure"];  // 1013.25 mbar
  data.metric_temp = doc["metric_temp"];  // true/false
  data.metric_alt = doc["metric_alt"];  // true/false
  data.min_batt_v = doc["min_batt_v"];  // 47.2v
  data.max_batt_v = doc["max_batt_v"];  // 59.2v
  if (!sanitize) {
    data.armed_time = doc["armed_time"];
    data.version_major = doc["major_v"];
    data.version_minor = doc["minor_v"];
  }
  data.crc = crc16((uint8_t*)&data, sizeof(data) - 2);
  return data;
}

void parse_usb_serial() {
  const size_t capacity = JSON_OBJECT_SIZE(11) + 112;
  DynamicJsonDocument doc(capacity);
  deserializeJson(doc, usb_web);

  if (doc["command"] && doc["command"] == "rbl") {
    enterUf2Dfu();
    return;
  }

  deviceData = jsonToDeviceData(doc, true);
  initDisplay();
  writeDeviceData();
  send_usb_serial();
}

void send_usb_serial() {
  const size_t capacity = JSON_OBJECT_SIZE(11) + 112;
  DynamicJsonDocument doc(capacity);

  doc = deviceDataToJson(deviceData);
  serializeJson(doc, usb_web);
  usb_web.println();
}
