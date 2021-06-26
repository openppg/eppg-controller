// Copyright 2020 <Zach Whitehead>

void handleFlightTime() {
  if (!armed) {
    throttledFlag = true;
    throttled = false;
  }
  if (armed) {
    if (throttlePercent > 30 && throttledFlag) {
      throttledAtMillis = millis();
      throttledFlag = false;
      throttled = true;
    }
    if (throttled) {
      throttleSecs = (millis()-throttledAtMillis) / 1000.0;
    } else {
      throttleSecs = 0;
    }
  }
}

// TODO (bug) rolls over at 99mins
void displayTime(int val, int x, int y, uint16_t bg_color) {
  // displays number of minutes and seconds (since armed and throttled)
  display.setCursor(x, y);
  display.setTextSize(2);
  display.setTextColor(BLACK);
  minutes = val / 60;
  seconds = numberOfSeconds(val);
  if (minutes < 10) {
    display.setCursor(x, y);
    display.print("0");
  }
  dispValue(minutes, prevMinutes, 2, 0, x, y, 2, BLACK, bg_color);
  display.setCursor(x+24, y);
  display.print(":");
  display.setCursor(x+36, y);
  if (seconds < 10) {
    display.print("0");
  }
  dispValue(seconds, prevSeconds, 2, 0, x+36, y, 2, BLACK, bg_color);
}


//**************************************************************************************//
//  Helper function to print values without flashing numbers due to slow screen refresh.
//  This function only re-draws the digit that needs to be updated.
//    BUG:  If textColor is not constant across multiple uses of this function,
//          weird things happen.
//**************************************************************************************//
void dispValue(float value, float &prevVal, int maxDigits, int precision, int x, int y, int textSize, int textColor, int background){
  int numDigits = 0;
  char prevDigit[DIGIT_ARRAY_SIZE] = {};
  char digit[DIGIT_ARRAY_SIZE] = {};
  String prevValTxt = String(prevVal, precision);
  String valTxt = String(value, precision);
  prevValTxt.toCharArray(prevDigit, maxDigits+1);
  valTxt.toCharArray(digit, maxDigits+1);

  // COUNT THE NUMBER OF DIGITS THAT NEED TO BE PRINTED:
  for(int i=0; i<maxDigits; i++){
    if (digit[i]) {
      numDigits++;
    }
  }

  display.setTextSize(textSize);
  display.setCursor(x, y);

  // PRINT LEADING SPACES TO RIGHT-ALIGN:
  display.setTextColor(background);
  for(int i=0; i<(maxDigits-numDigits); i++){
    display.print(char(218));
  }
  display.setTextColor(textColor);

  // ERASE ONLY THE NESSESARY DIGITS:
  for (int i=0; i<numDigits; i++) {
    if (digit[i]!=prevDigit[i]) {
      display.setTextColor(background);
      display.print(char(218));
    } else {
      display.setTextColor(textColor);
      display.print(digit[i]);
    }
  }

  // BACK TO THE BEGINNING:
  display.setCursor(x, y);

  // ADVANCE THE CURSOR TO THE PROPER LOCATION:
  display.setTextColor(background);
  for (int i=0; i<(maxDigits-numDigits); i++) {
    display.print(char(218));
  }
  display.setTextColor(textColor);

  // PRINT THE DIGITS THAT NEED UPDATING
  for(int i=0; i<numDigits; i++){
    display.print(digit[i]);
  }

  prevVal = value;
}

void initBmp() {
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}

void buzzInit(bool enableBuz) {
  pinMode(BUZ_PIN, OUTPUT);
  return;  // TODO deprecated?

  if (enableBuz) {
    tone(BUZ_PIN, 500);
    delay(200);
    tone(BUZ_PIN, 700);
    delay(200);
    tone(BUZ_PIN, 900);
    delay(200);
    noTone(BUZ_PIN);
  }
}

void prepareSerialRead() {  // TODO needed?
  while (Serial5.available() > 0) {
    byte t = Serial5.read();
  }
}

void handleTelemetry() {
  prepareSerialRead();
  Serial5.readBytes(escData, ESC_DATA_SIZE);
  // enforceChecksum();
  if (enforceFletcher16()) {
    parseData();
  }
  // printRawSentence();
}

bool enforceFletcher16() {
  // Check checksum, revert to previous data if bad:
  word checksum = (unsigned short)(word(escData[19], escData[18]));
  unsigned char sum1 = 0;
  unsigned char sum2 = 0;
  unsigned short sum = 0;
  for (int i = 0; i < ESC_DATA_SIZE-2; i++) {
    sum1 = (unsigned char)(sum1 + escData[i]);
    sum2 = (unsigned char)(sum2 + sum1);
  }
  sum = (unsigned char)(sum1 - sum2);
  sum = sum << 8;
  sum |= (unsigned char)(sum2 - 2*sum1);
  // Serial.print(F("     SUM: "));
  // Serial.println(sum);
  // Serial.print(sum1,HEX);
  // Serial.print(" ");
  // Serial.println(sum2,HEX);
  // Serial.print(F("CHECKSUM: "));
  // Serial.println(checksum);
  if (sum != checksum) {
    //Serial.println(F("_____________________CHECKSUM FAILED!"));
    failed++;
    if (failed >= 1000) {  // keep track of how reliable the transmission is
      transmitted = 1;
      failed = 0;
    }
    return false;
  }
  return true;
}

// Not used
void enforceChecksum() {
  //Check checksum, revert to previous data if bad:
  word checksum = word(escData[19], escData[18]);
  int sum = 0;
  for (int i=0; i<ESC_DATA_SIZE-2; i++) {
    sum += escData[i];
  }
  Serial.print(F("     SUM: "));
  Serial.println(sum);
  Serial.print(F("CHECKSUM: "));
  Serial.println(checksum);
  if (sum != checksum) {
    Serial.println(F("__________________________CHECKSUM FAILED!"));
    failed++;
    if (failed >= 1000) {  // keep track of how reliable the transmission is
      transmitted = 1;
      failed = 0;
    }
    for (int i=0; i<ESC_DATA_SIZE; i++) {  // revert to previous data
      escData[i] = prevData[i];
    }
  }
  for (int i=0; i<ESC_DATA_SIZE; i++) {
    prevData[i] = escData[i];
  }
}


void printRawSentence() {
  Serial.print(F("DATA: "));
  for (int i = 0; i < ESC_DATA_SIZE; i++) {
    Serial.print(escData[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}


void parseData() {
  // LSB First
  // TODO is this being called even with no ESC?

  _volts = word(escData[1], escData[0]);
  //_volts = ((unsigned int)escData[1] << 8) + escData[0];
  telemetryData.volts = _volts / 100.0;

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.5;  // calibration
  }

  if (telemetryData.volts > 1) {  // ignore empty data
    voltageBuffer.push(telemetryData.volts);
  }
  // Serial.print(F("Volts: "));
  // Serial.println(telemetryData.volts);

  // batteryPercent = mapf(telemetryData.volts, BATT_MIN_V, BATT_MAX_V, 0.0, 100.0); // flat line

  _temperatureC = word(escData[3], escData[2]);
  telemetryData.temperatureC = _temperatureC/100.0;
  // reading 17.4C = 63.32F in 84F ambient?
  // Serial.print(F("TemperatureC: "));
  // Serial.println(temperatureC);

  _amps = word(escData[5], escData[4]);
  telemetryData.amps = _amps;

  // Serial.print(F("Amps: "));
  // Serial.println(amps);

  watts = telemetryData.amps * telemetryData.volts;

  // 7 and 6 are reserved bytes

  _eRPM = escData[11];     // 0
  _eRPM << 8;
  _eRPM += escData[10];    // 0
  _eRPM << 8;
  _eRPM += escData[9];     // 30
  _eRPM << 8;
  _eRPM += escData[8];     // b4
  telemetryData.eRPM = _eRPM/6.0/2.0;

  // Serial.print(F("eRPM: "));
  // Serial.println(eRPM);

  _inPWM = word(escData[13], escData[12]);
  telemetryData.inPWM = _inPWM/100.0;

  // Serial.print(F("inPWM: "));
  // Serial.println(inPWM);

  _outPWM = word(escData[15], escData[14]);
  telemetryData.outPWM = _outPWM/100.0;

  // Serial.print(F("outPWM: "));
  // Serial.println(outPWM);

  // 17 and 16 are reserved bytes
  // 19 and 18 is checksum
  telemetryData.checksum = word(escData[19], escData[18]);

  // Serial.print(F("CHECKSUM: "));
  // Serial.print(escData[19]);
  // Serial.print(F(" + "));
  // Serial.print(escData[18]);
  // Serial.print(F(" = "));
  // Serial.println(checksum);
}

void vibrateAlert() {
  if (!ENABLE_VIB) { return; }
  int effect = 15;  // 1 through 117 (see example sketch)
  vibe.setWaveform(0, effect);
  vibe.setWaveform(1, 0);
  vibe.go();
}

void vibrateNotify() {
  if (!ENABLE_VIB) { return; }

  vibe.setWaveform(0, 15);  // 1 through 117 (see example sketch)
  vibe.setWaveform(1, 0);
  vibe.go();
}


int limitedThrottle(int current, int last, int threshold) {
  prevPotLvl = current;

  if (current > last) {  // accelerating
    if (current - last >= threshold) {  // acccelerating too fast. limit
      int limitedThrottle = last + threshold;
      prevPotLvl = limitedThrottle;
      return limitedThrottle;
    } else {
      return current;
    }
  } else {  // deccelerating / maintaining
    return current;
  }
}

float getBatteryVoltSmoothed() {
  float avg = 0.0;

  if (voltageBuffer.isEmpty()) { return avg; }

  using index_t = decltype(voltageBuffer)::index_t;
  for (index_t i = 0; i < voltageBuffer.size(); i++) {
    avg += voltageBuffer[i] / voltageBuffer.size();
  }
  return avg;
}

float getBatteryPercent(float voltage) {
  //Serial.print("percent v ");
  //Serial.println(voltage);

  int battPercent = 0;
  if (voltage < BATT_MIN_V) { return battPercent; }  // just stop here if low

  if (telemetryData.volts >= BATT_MID_V) {
    battPercent = mapf(voltage, BATT_MID_V, BATT_MAX_V, 50.0, 100.0);
  } else {
    battPercent = mapf(voltage, BATT_MIN_V, BATT_MID_V, 0.0, 50.0);
  }
  // batteryPercent = battery_sigmoidal(voltage, 58.0, BATT_MAX_V);
  return constrain(battPercent, 0, 100);
}

// inspired by https://github.com/rlogiacco/BatterySense/
// https://www.desmos.com/calculator/7m9lu26vpy
uint8_t battery_sigmoidal(float voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}
