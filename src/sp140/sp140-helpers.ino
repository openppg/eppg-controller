// Copyright 2020 <Zach Whitehead>

// track flight timer
void handleFlightTime() {
  if (!armed) {
    throttledFlag = true;
    throttled = false;
  } else { // armed
    // start the timer when armed and throttle is above the threshold
    if (throttlePWM > 1300 && throttledFlag) {
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

// maps battery percentage to a display color
uint16_t batt2color(int percentage) {
  if (percentage >= 30) {
    return GREEN;
  } else if (percentage >= 15) {
    return YELLOW;
  }
  return RED;
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
    display.print(static_cast<char>(218));
  }
  display.setTextColor(textColor);

  // ERASE ONLY THE NESSESARY DIGITS:
  for (int i=0; i<numDigits; i++) {
    if (digit[i]!=prevDigit[i]) {
      display.setTextColor(background);
      display.print(static_cast<char>(218));
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
    display.print(static_cast<char>(218));
  }
  display.setTextColor(textColor);

  // PRINT THE DIGITS THAT NEED UPDATING
  for(int i=0; i<numDigits; i++){
    display.print(digit[i]);
  }

  prevVal = value;
}

// Start the bmp388 sensor
void initBmp() {
  bmp.begin_I2C();
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
}

void buzzInit(bool enableBuz) {
  pinMode(BUZ_PIN, OUTPUT);
}

void prepareSerialRead() {  // TODO needed?
  while (Serial5.available() > 0) {
    byte t = Serial5.read();
  }
}

void handleTelemetry() {
  prepareSerialRead();
  byte stopbyte = 255;
  byte res = Serial5.readBytes(escData, ESC_DATA_SIZE);
  //printRawSentence();

  if (escData[20] != stopbyte || escData[21] != stopbyte) {
    Serial.println("not valid stops");
    Serial5.flush();
    return;
  }

  uint16_t check_calc = word(escData[19], escData[18]);

  if (validFletcher16(escData, ESC_DATA_SIZE-4, check_calc)) {
    parseData();
  } else {
    Serial.println("checksum failed");
  }
  Serial5.flush();
}

bool validFletcher16(uint8_t *data, int limit, uint16_t parsed_sum) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (int i = 0; i < limit; i++) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  uint16_t calculated = (sum2 << 8) | sum1;
  return calculated == parsed_sum;
}

// for debugging
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

  //Serial.print(F("Volts: "));
  //Serial.println(_volts);

  telemetryData.volts += VOLT_OFFSET;  // calibration

  if (telemetryData.volts > 10) {  // ignore empty data
    voltageBuffer.push(telemetryData.volts);
  }

  // batteryPercent = mapf(telemetryData.volts, BATT_MIN_V, BATT_MAX_V, 0.0, 100.0); // flat line

  _temperatureC = word(escData[3], escData[2]);
  telemetryData.temperatureC = _temperatureC/100.0;
  // reading 17.4C = 63.32F in 84F ambient?
  // Serial.print(F("TemperatureC: "));
  // Serial.println(temperatureC);

  _amps = word(escData[5], escData[4]);
  telemetryData.amps = constrain(_amps / 12.5, 0, 500);

  //Serial.print(F("Amps: "));
  //Serial.println(_amps);

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

void vibrateNotify() {
  if (!ENABLE_VIB) { return; }

  vibe.setWaveform(0, 15);  // 1 through 117 (see example sketch)
  vibe.setWaveform(1, 0);
  vibe.go();
}

// throttle easing function based on threshold/performance mode
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast. limit
    int limitedThrottle = last + threshold;
    // TODO: cleanup global var use
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  } else if (last - current >= threshold * 2) {  // decelerating too fast. limit
    int limitedThrottle = last - threshold * 2;  // double the decel vs accel
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  }
  prevPotLvl = current;
  return current;
}

// ring buffer for voltage readings
float getBatteryVoltSmoothed() {
  float avg = 0.0;

  if (voltageBuffer.isEmpty()) { return avg; }

  using index_t = decltype(voltageBuffer)::index_t;
  for (index_t i = 0; i < voltageBuffer.size(); i++) {
    avg += voltageBuffer[i] / voltageBuffer.size();
  }
  return avg;
}
