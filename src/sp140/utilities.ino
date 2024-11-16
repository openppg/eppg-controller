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
  USBSerial.println("playMelody");

  if (!ENABLE_BUZ) { return false; }
  for (int thisNote = 0; thisNote < siz; thisNote++) {
    // quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 125;
    playNote(melody[thisNote], noteDuration);
  }
  return true;
}

// blocking tone function that delays for notes
void playNote(uint16_t note, uint16_t duration) {
  USBSerial.println("playNote");
  // quarter note = 1000 / 4, eighth note = 1000/8, etc.
  tone(8, note);
  delay(duration);  // to distinguish the notes, delay between them
  noTone(8);
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
