// Copyright 2020 <Zach Whitehead>

// initialize the buzzer
void initBuzz() {
  pinMode(board_config.buzzer_pin, OUTPUT);
}

// on boot check for button to switch mode
void modeSwitch(bool update_display) {
  // 0=CHILL 1=SPORT 2=LUDICROUS?!
  if (deviceData.performance_mode == 0) {
    deviceData.performance_mode = 1;
  } else {
    deviceData.performance_mode = 0;
  }

  if (!armed) {
    writeDeviceData();
  }

  uint16_t notify_melody[] = { 900, 1976 };
  playMelody(notify_melody, 2);
}

void handleTelemetry() {
  #ifdef CAN_PIO
    // TODO
  #else
  while (SerialESC.available() > 0) { SerialESC.read(); }
  SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE);
  //printRawSentence();
  handleSerialData(escDataV2);
  #endif
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

    return; //Stop byte of 65535 not received
  }

  //Check the fletcher checksum
  int checkFletch = CheckFlectcher16(buffer);

  // checksum
  raw_telemdata.CSUM_HI = buffer[19];
  raw_telemdata.CSUM_LO = buffer[18];

  //TODO: alert if no new data in 3 seconds
  int checkCalc = (int)(((raw_telemdata.CSUM_HI << 8) + raw_telemdata.CSUM_LO));

  // Checksums do not match
  if (checkFletch != checkCalc) {
    return;
  }
  // Voltage
  raw_telemdata.V_HI = buffer[1];
  raw_telemdata.V_LO = buffer[0];

  float voltage = (raw_telemdata.V_HI << 8 | raw_telemdata.V_LO) / 100.0;
  telemetryData.volts = voltage;  // Voltage

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.0;  // calibration
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
  float temperature = Rntc / (float) NOMINAL_RESISTANCE;  // (R/Ro)
  temperature = (float) log(temperature); // ln(R/Ro)
  temperature /= BCOEFFICIENT; // 1/B * ln(R/Ro)

  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15);  // + (1/To)
  temperature = 1.0 / temperature; // Invert
  temperature -= 273.15; // convert to Celsius

  // filter bad values
  if (temperature < 0 || temperature > 200) {
    telemetryData.temperatureC = __FLT_MIN__;
  } else {
    temperature = (float) trunc(temperature * 100) / 100;  // 2 decimal places
    telemetryData.temperatureC = temperature;
  }

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
  telemetryData.inPWM = (throttleDuty / 10);  // Input throttle

  // Serial.print("throttle ");
  // Serial.print(telemetryData.inPWM);
  // Serial.print(" - ");

  // Motor Duty
  // raw_telemdata.MOTORDUTY_HI = buffer[15];
  // raw_telemdata.MOTORDUTY_LO = buffer[14];

  // int motorDuty = (int)(((raw_telemdata.MOTORDUTY_HI << 8) + raw_telemdata.MOTORDUTY_LO) / 10);
  // int currentMotorDuty = (motorDuty / 10);  // Motor duty cycle

  // Reserved
  // raw_telemdata.R1 = buffer[17];

  /* Status Flags
  # Bit position in byte indicates flag set, 1 is set, 0 is default
  # Bit 0: Motor Started, set when motor is running as expected
  # Bit 1: Motor Saturation Event, set when saturation detected and power is reduced for desync protection
  # Bit 2: ESC Over temperature event occurring, shut down method as per configuration
  # Bit 3: ESC Overvoltage event occurring, shut down method as per configuration
  # Bit 4: ESC Undervoltage event occurring, shut down method as per configuration
  # Bit 5: Startup error detected, motor stall detected upon trying to start*/
  raw_telemdata.statusFlag = buffer[16];
  telemetryData.statusFlag = raw_telemdata.statusFlag;
  // Serial.print("status ");
  // Serial.print(raw_telemdata.statusFlag, BIN);
  // Serial.print(" - ");
  // Serial.println(" ");
}

// new V2
int CheckFlectcher16(byte byteBuffer[]) {
  int fCCRC16;
  int i;
  int c0 = 0;
  int c1 = 0;

  // Calculate checksum intermediate bytesUInt16
  for (i = 0; i < 18; i++) { //Check only first 18 bytes, skip crc bytes
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

// throttle easing function based on threshold/performance mode
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast. limit
    int limitedThrottle = last + threshold;
    // TODO:  cleanup global var use
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
