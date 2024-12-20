// Copyright 2019 <Zach Whitehead>
// OpenPPG

/**
 * For digital time display - prints leading 0
 *
 * @param digits number to be converted to a string.
 * @return string `12`, or 07 if `digits` is less than 10.
 */
String convertToDigits(byte digits) {
  String digits_string = "";
  if (digits < 10) digits_string.concat("0");
  digits_string.concat(digits);
  return digits_string;
}

// set LED color
void setLEDColor(uint32_t color) {
  if (board_config.enable_neopixel) {
    led_color = color;
    pixels.setPixelColor(0, color);
    pixels.show();
  }
}

/**
 * Sets the state of the LEDs.
 *
 * If board_config.enable_neopixel is true, sets the color of the NeoPixel LED to led_color
 * when the state is HIGH, and clears the NeoPixel LED when the state is LOW.
 * Otherwise, sets the state of the LED_SW pin to the given state.
 *
 * @param state The state to set the LEDs to (HIGH or LOW).
 */
void setLEDs(byte state) {
  if (board_config.enable_neopixel) {
    if (state == HIGH) {
      pixels.setPixelColor(0, led_color);
    } else {
      pixels.clear();
    }
    pixels.show();
  } else {
    digitalWrite(board_config.led_sw, state);
  }
}

// toggle LEDs
byte ledState = LOW;

void blinkLED() {
  ledState = !ledState;
  setLEDs(ledState);
}

/**
 * Plays a melody using a piezo buzzer.
 *
 * @param melody An array of uint16_t values representing the notes of the melody.
 * @param siz The size of the melody array.
 * @return Returns true if the melody was played successfully, false otherwise.
 */
bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZ) return false;

  // Create a static buffer for the melody
  static uint16_t melodyBuffer[32];  // Adjust size as needed

  // Copy melody to static buffer to ensure it persists
  for(int i = 0; i < min(siz, 32); i++) {
    melodyBuffer[i] = melody[i];
  }

  MelodyRequest request = {
    .notes = melodyBuffer,
    .size = (uint8_t)min(siz, 32),
    .duration = 125  // Default duration
  };

  // Send to queue with timeout
  if(xQueueSend(melodyQueue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    USBSerial.println("Failed to queue melody");
    return false;
  }

  return true;
}

/**
 * Plays a melody to indicate arm failure.
 */
void handleArmFail() {
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
}

// for debugging
void printDeviceData() {
  Serial.print(F("version major "));
  Serial.println(deviceData.version_major);
  Serial.print(F("version minor "));
  Serial.println(deviceData.version_minor);
  Serial.print(F("screen rotation "));
  Serial.println(deviceData.screen_rotation);
  Serial.print(F("sea pressure "));
  Serial.println(deviceData.sea_pressure);
  Serial.print(F("metric temp "));
  Serial.println(deviceData.metric_temp);
  Serial.print(F("metric alt "));
  Serial.println(deviceData.metric_alt);
  Serial.print(F("performance mode "));
  Serial.println(deviceData.performance_mode);
  Serial.print(F("theme "));
  Serial.println(deviceData.theme);
  Serial.print(F("batt size "));
  Serial.println(deviceData.batt_size);
  Serial.print(F("armed time "));
  Serial.println(deviceData.armed_time);
  Serial.print(F("revision "));
  Serial.println(deviceData.revision);
  Serial.print(F("crc "));
  Serial.println(deviceData.crc);
}

#ifdef CAN_PIO
String chipId() {
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  return String(chipId);
}
#endif
