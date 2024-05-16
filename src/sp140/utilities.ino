// Copyright 2019 <Zach Whitehead>
// OpenPPG

#ifdef M0_PIO

#define DBL_TAP_PTR ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
#define DBL_TAP_MAGIC 0xf01669ef  // Randomly selected, adjusted to have first and last bit set
#define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef

#endif

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

bool runVibe(const unsigned int sequence[], int siz) {
  if (!vibePresent) { return false; }

  for (int thisVibe = 0; thisVibe < siz; thisVibe++) {
    vibe.setWaveform(thisVibe, sequence[thisVibe]);
  }
  vibe.go();
  return true;
}

/**
 * Plays a melody using a piezo buzzer.
 *
 * @param melody An array of uint16_t values representing the notes of the melody.
 * @param siz The size of the melody array.
 * @return Returns true if the melody was played successfully, false otherwise.
 */
bool playMelody(uint16_t melody[], int siz) {
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
  // quarter note = 1000 / 4, eighth note = 1000/8, etc.
  tone(board_config.buzzer_pin, note);
  delay(duration);  // to distinguish the notes, delay between them
  noTone(board_config.buzzer_pin);
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

#ifdef M0_PIO
// get chip serial number (for SAMD21)
String chipId() {
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;

  char id_buf[33];
  sprintf(id_buf, "%8x%8x%8x%8x", val1, val2, val3, val4);
  return String(id_buf);
}
#elif RP_PIO
String chipId() {
  int len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
  uint8_t buff[len] = "";
  pico_get_unique_board_id_string((char *)buff, len);
  return String((char *)buff);
}
#endif // M0_PIO/RP_PIO

#ifdef M0_PIO
// reboot/reset controller
void(* resetFunc) (void) = 0;  // declare reset function @ address 0

// sets the magic pointer to trigger a reboot to the bootloader for updating
void rebootBootloader() {
  *DBL_TAP_PTR = DBL_TAP_MAGIC;

  resetFunc();
}

#elif RP_PIO

// reboot/reset controller
void rebootBootloader() {
#ifdef USE_TINYUSB
  TinyUSB_Port_EnterDFU();
#endif
}
#endif
