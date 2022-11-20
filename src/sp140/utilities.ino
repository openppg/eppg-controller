// Copyright 2019 <Zach Whitehead>
// OpenPPG

#define LAST_PAGE 1  // starts at 0

#ifdef M0_PIO

#define DBL_TAP_PTR ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
#define DBL_TAP_MAGIC 0xf01669ef  // Randomly selected, adjusted to have first and last bit set
#define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef

#endif

// Map double values
double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

/**
 * advance to next screen page
 *
 * @return the number of next page
 */
int nextPage() {
  gfx->fillRect(0, 37, 160, 54, DEFAULT_BG_COLOR);

  if (page >= LAST_PAGE) {
    return page = 0;
  }
  return ++page;
}

void addVSpace() {
  gfx->setTextSize(1);
  gfx->println(" ");
}

void setLEDs(byte state) {
  // digitalWrite(LED_2, state);
  // digitalWrite(LED_3, state);
  digitalWrite(LED_SW, state);
}

// toggle LEDs
void blinkLED() {
  byte ledState = !digitalRead(LED_SW);
  setLEDs(ledState);
}

bool runVibe(unsigned int sequence[], int siz) {
  if (!ENABLE_VIB) { return false; }

  for (int thisVibe = 0; thisVibe < siz; thisVibe++) {
    vibe.setWaveform(thisVibe, sequence[thisVibe]);
  }
  vibe.go();
  return true;
}

bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZ) { return false; }
  for (int thisNote = 0; thisNote < siz; thisNote++) {
    // quarter note = 1000 / 4, eigth note = 1000/8, etc.
    int noteDuration = 125;
    playNote(melody[thisNote], noteDuration);
  }
  return true;
}

#ifdef RP_PIO
// non-blocking tone function that uses second core
void playNote(uint16_t note, uint16_t duration) {
    STR_NOTE noteData;
    // fifo uses 32 bit messages so package up the note and duration
    uint32_t note_msg;
    noteData.duration = duration;
    noteData.freq = note;

    memcpy((uint32_t*)&note_msg, &noteData, sizeof(noteData));
    rp2040.fifo.push_nb(note_msg);  // send note to second core via fifo queue
}
#else
// blocking tone function that delays for notes
void playNote(uint16_t note, uint16_t duration) {
  // quarter note = 1000 / 4, eigth note = 1000/8, etc.
  tone(BUZZER_PIN, note);
  delay(duration);  // to distinguish the notes, delay between them
  noTone(BUZZER_PIN);
}
#endif

void handleArmFail() {
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
}

// for debugging
void printDeviceData() {
  Serial.print("version major ");
  Serial.println(deviceData.version_major);
  Serial.print("version minor ");
  Serial.println(deviceData.version_minor);
  Serial.print("armed_time ");
  Serial.println(deviceData.armed_time);
  Serial.print("crc ");
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

void displayMeta() {
  gfx->setTextColor(BLACK);
  gfx->setCursor(25, 30);
  gfx->println("OpenPPG");
  gfx->setFont();
  gfx->setTextSize(2);
  gfx->setCursor(60, 60);
  gfx->print("v" + String(VERSION_MAJOR) + "." + String(VERSION_MINOR));
#ifdef RP_PIO
  gfx->print("R");
#endif
  gfx->setCursor(54, 90);
  displayTime(deviceData.armed_time);
}
