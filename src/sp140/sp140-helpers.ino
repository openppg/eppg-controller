// Copyright 2020 <Zach Whitehead>

// track flight timer
void handleFlightTime() {
  if (!armed) {
    throttledFlag = true;
    throttled = false;
  } else { // armed
    // start the timer when armed and throttle is above the threshold
    if (throttlePWM > 1250 && throttledFlag) {
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
bool initBmp() {
  if (!bmp.begin_I2C()) { return false; }

  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);

  return true;
}

// initialize the buzzer
void initBuzz() {
  pinMode(BUZZER_PIN, OUTPUT);
}

// initialize the vibration motor
bool initVibe() {
  if (!ENABLE_VIB) { return false; }
  if (!vibe.begin()) { return false; }

  vibe.selectLibrary(1);
  vibe.setMode(DRV2605_MODE_INTTRIG);
  vibrateNotify();  // initial boot vibration

  return true;
}

// on boot check for button to switch mode
void modeSwitch(bool update_display) {
  // 0=CHILL 1=SPORT 2=LUDICROUS?!
  if (deviceData.performance_mode == 0) {
    deviceData.performance_mode = 1;
  } else {
    deviceData.performance_mode = 0;
  }
  writeDeviceData();
  if (update_display) { // clear out old text
    clearModeDisplay();
  }
  uint16_t notify_melody[] = { 900, 1976 };
  playMelody(notify_melody, 2);
}

void clearModeDisplay() {
  String prevMode = (deviceData.performance_mode == 0) ? "SPORT" : "CHILL";
  display.setCursor(30, 60);
  display.setTextSize(1);
  display.setTextColor(DEFAULT_BG_COLOR);
  display.print(prevMode);
}

void prepareSerialRead() {  // TODO needed?
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
}

void handleTelemetry() {
  prepareSerialRead();
  SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE);
  //printRawSentence();
  handleSerialData(escDataV2);
}

// new for v2 ESC telemetry
void handleSerialData(byte buffer[]) {
  // if(sizeof(buffer) != 22) {
  //     Serial.print("wrong size ");
  //     Serial.println(sizeof(buffer));
  //     return; //Ignore malformed packets
  // }

  if (buffer[20] != 255 || buffer[21] != 255) {
    //Serial.println("no stop byte");

    return; //Stop byte of 65535 not recieved
  }

  //Check the fletcher checksum
  int checkFletch = CheckFlectcher16(buffer);

  // checksum
  raw_telemdata.CSUM_HI = buffer[19];
  raw_telemdata.CSUM_LO = buffer[18];

  //TODO alert if no new data in 3 seconds
  int checkCalc = (int)(((raw_telemdata.CSUM_HI << 8) + raw_telemdata.CSUM_LO));

  // Checksums do not match
  if (checkFletch != checkCalc) {
    return;
  }
  // Voltage
  raw_telemdata.V_HI = buffer[1];
  raw_telemdata.V_LO = buffer[0];

  float voltage = (raw_telemdata.V_HI << 8 | raw_telemdata.V_LO) / 100.0;
  telemetryData.volts = voltage; //Voltage

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.0; // calibration
  }

  voltageBuffer.push(telemetryData.volts);

  // Temperature
  raw_telemdata.T_HI = buffer[3];
  raw_telemdata.T_LO = buffer[2];

  float rawVal = (float)((raw_telemdata.T_HI << 8) + raw_telemdata.T_LO);

  static int SERIESRESISTOR = 10000;
  static int NOMINAL_RESISTANCE = 10000;
  static int NOMINAL_TEMPERATURE = 25;
  static int BCOEFFICIENT = 3455;

  //convert value to resistance
  float Rntc = (4096 / (float) rawVal) - 1;
  Rntc = SERIESRESISTOR / Rntc;

  // Get the temperature
  float temperature = Rntc / (float) NOMINAL_RESISTANCE; // (R/Ro)
  temperature = (float) log(temperature); // ln(R/Ro)
  temperature /= BCOEFFICIENT; // 1/B * ln(R/Ro)

  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  temperature = 1.0 / temperature; // Invert
  temperature -= 273.15; // convert to Celcius

  // filter bad values
  if (temperature < 0 || temperature > 200) {
    temperature = 0;
  }

  temperature = (float) trunc(temperature * 100) / 100; // 2 decimal places
  telemetryData.temperatureC = temperature;

  // Current
  _amps = word(buffer[5], buffer[4]);
  telemetryData.amps = _amps / 12.5;

  // Serial.print("amps ");
  // Serial.print(currentAmpsInput);
  // Serial.print(" - ");

  watts = telemetryData.amps * telemetryData.volts;

  // Reserved
  raw_telemdata.R0_HI = buffer[7];
  raw_telemdata.R0_LO = buffer[6];

  // eRPM
  raw_telemdata.RPM0 = buffer[11];
  raw_telemdata.RPM1 = buffer[10];
  raw_telemdata.RPM2 = buffer[9];
  raw_telemdata.RPM3 = buffer[8];

  int poleCount = 62;
  int currentERPM = (int)((raw_telemdata.RPM0 << 24) + (raw_telemdata.RPM1 << 16) + (raw_telemdata.RPM2 << 8) + (raw_telemdata.RPM3 << 0)); //ERPM output
  int currentRPM = currentERPM / poleCount;  // Real RPM output
  telemetryData.eRPM = currentRPM;

  // Serial.print("RPM ");
  // Serial.print(currentRPM);
  // Serial.print(" - ");

  // Input Duty
  raw_telemdata.DUTYIN_HI = buffer[13];
  raw_telemdata.DUTYIN_LO = buffer[12];

  int throttleDuty = (int)(((raw_telemdata.DUTYIN_HI << 8) + raw_telemdata.DUTYIN_LO) / 10);
  telemetryData.inPWM = (throttleDuty / 10); //Input throttle

  // Serial.print("throttle ");
  // Serial.print(telemetryData.inPWM);
  // Serial.print(" - ");

  // Motor Duty
  raw_telemdata.MOTORDUTY_HI = buffer[15];
  raw_telemdata.MOTORDUTY_LO = buffer[14];

  int motorDuty = (int)(((raw_telemdata.MOTORDUTY_HI << 8) + raw_telemdata.MOTORDUTY_LO) / 10);
  int currentMotorDuty = (motorDuty / 10); //Motor duty cycle

  // Reserved
  // raw_telemdata.R1 = buffer[17];

  /* Status Flags
  # Bit position in byte indicates flag set, 1 is set, 0 is default
  # Bit 0: Motor Started, set when motor is running as expected
  # Bit 1: Motor Saturation Event, set when saturation detected and power is reduced for desync protection
  # Bit 2: ESC Over temperature event occuring, shut down method as per configuration
  # Bit 3: ESC Overvoltage event occuring, shut down method as per configuration
  # Bit 4: ESC Undervoltage event occuring, shut down method as per configuration
  # Bit 5: Startup error detected, motor stall detected upon trying to start*/
  raw_telemdata.statusFlag = buffer[16];
  telemetryData.statusFlag = raw_telemdata.statusFlag;
  // Serial.print("status ");
  // Serial.print(raw_telemdata.statusFlag, BIN);
  // Serial.print(" - ");
  // Serial.println(" ");
}

// new v2
int CheckFlectcher16(byte byteBuffer[]) {
    int fCCRC16;
    int i;
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (i = 0; i < 18; i++) //Check only first 18 bytes, skip crc bytes
    {
        c0 = (int)(c0 + ((int)byteBuffer[i])) % 255;
        c1 = (int)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = ( c1 << 8 ) | c0;
    return (int)fCCRC16;
}

// for debugging
void printRawSentence() {
  Serial.print(F("DATA: "));
  for (int i = 0; i < ESC_DATA_V2_SIZE; i++) {
    Serial.print(escDataV2[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}

// OLD
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

  // batteryPercent = mapd(telemetryData.volts, BATT_MIN_V, BATT_MAX_V, 0.0, 100.0); // flat line

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
