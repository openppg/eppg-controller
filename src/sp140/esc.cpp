#include "sp140/esc.h"
#include <Servo.h>
#include "sp140/globals.h"
#include <CircularBuffer.hpp>

Servo esc;
extern CircularBuffer<float, 50> voltageBuffer;

STR_ESC_TELEMETRY_140 escTelemetryData;
static telem_esc_t raw_esc_telemdata;

void initESC(int escPin) {
  esc.attach(escPin);
  esc.writeMicroseconds(ESC_DISARMED_PWM);
  setupESCSerial();
}

void setupESCSerial() {
  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
}

void setESCThrottle(int throttlePWM) {
  esc.writeMicroseconds(throttlePWM);
}

void readESCTelemetry() {
  prepareESCSerialRead();
  byte escDataV2[ESC_DATA_V2_SIZE];
  SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE);
  handleESCSerialData(escDataV2);
}

void prepareESCSerialRead() {
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
}

void handleESCSerialData(byte buffer[]) {
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
  int checkFletch = checkFletcher16(buffer);

  // checksum
  raw_esc_telemdata.CSUM_HI = buffer[19];
  raw_esc_telemdata.CSUM_LO = buffer[18];

  //TODO alert if no new data in 3 seconds
  int checkCalc = (int)(((raw_esc_telemdata.CSUM_HI << 8) + raw_esc_telemdata.CSUM_LO));

  // Checksums do not match
  if (checkFletch != checkCalc) {
    return;
  }
  // Voltage
  raw_esc_telemdata.V_HI = buffer[1];
  raw_esc_telemdata.V_LO = buffer[0];

  float voltage = (raw_esc_telemdata.V_HI << 8 | raw_esc_telemdata.V_LO) / 100.0;
  escTelemetryData.volts = voltage;  // Voltage

  if (escTelemetryData.volts > BATT_MIN_V) {
    escTelemetryData.volts += 1.0;  // calibration
  }

  voltageBuffer.push(escTelemetryData.volts);

  // Temperature
  raw_esc_telemdata.T_HI = buffer[3];
  raw_esc_telemdata.T_LO = buffer[2];

  float rawVal = (float)((raw_esc_telemdata.T_HI << 8) + raw_esc_telemdata.T_LO);

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
    escTelemetryData.temperatureC = __FLT_MIN__;
  } else {
    temperature = (float) trunc(temperature * 100) / 100;  // 2 decimal places
    escTelemetryData.temperatureC = temperature;
  }

  // Current
  _amps = word(buffer[5], buffer[4]);
  escTelemetryData.amps = _amps / 12.5;

  // Serial.print("amps ");
  // Serial.print(currentAmpsInput);
  // Serial.print(" - ");

  watts = escTelemetryData.amps * escTelemetryData.volts;

  // Reserved
  raw_esc_telemdata.R0_HI = buffer[7];
  raw_esc_telemdata.R0_LO = buffer[6];

  // eRPM
  raw_esc_telemdata.RPM0 = buffer[11];
  raw_esc_telemdata.RPM1 = buffer[10];
  raw_esc_telemdata.RPM2 = buffer[9];
  raw_esc_telemdata.RPM3 = buffer[8];

  int poleCount = 62;
  int currentERPM = (int)((raw_esc_telemdata.RPM0 << 24) + (raw_esc_telemdata.RPM1 << 16) + (raw_esc_telemdata.RPM2 << 8) + (raw_esc_telemdata.RPM3 << 0)); //ERPM output
  int currentRPM = currentERPM / poleCount;  // Real RPM output
  escTelemetryData.eRPM = currentRPM;

  // Serial.print("RPM ");
  // Serial.print(currentRPM);
  // Serial.print(" - ");

  // Input Duty
  raw_esc_telemdata.DUTYIN_HI = buffer[13];
  raw_esc_telemdata.DUTYIN_LO = buffer[12];

  int throttleDuty = (int)(((raw_esc_telemdata.DUTYIN_HI << 8) + raw_esc_telemdata.DUTYIN_LO) / 10);
  escTelemetryData.inPWM = (throttleDuty / 10);  // Input throttle

  // Serial.print("throttle ");
  // Serial.print(escTelemetryData.inPWM);
  // Serial.print(" - ");

  // Motor Duty
  // raw_esc_telemdata.MOTORDUTY_HI = buffer[15];
  // raw_esc_telemdata.MOTORDUTY_LO = buffer[14];

  // int motorDuty = (int)(((raw_esc_telemdata.MOTORDUTY_HI << 8) + raw_esc_telemdata.MOTORDUTY_LO) / 10);
  // int currentMotorDuty = (motorDuty / 10);  // Motor duty cycle

  // Reserved
  // raw_esc_telemdata.R1 = buffer[17];

  /* Status Flags
  # Bit position in byte indicates flag set, 1 is set, 0 is default
  # Bit 0: Motor Started, set when motor is running as expected
  # Bit 1: Motor Saturation Event, set when saturation detected and power is reduced for desync protection
  # Bit 2: ESC Over temperature event occurring, shut down method as per configuration
  # Bit 3: ESC Overvoltage event occurring, shut down method as per configuration
  # Bit 4: ESC Undervoltage event occurring, shut down method as per configuration
  # Bit 5: Startup error detected, motor stall detected upon trying to start*/
  raw_esc_telemdata.statusFlag = buffer[16];
  escTelemetryData.statusFlag = raw_esc_telemdata.statusFlag;
  // Serial.print("status ");
  // Serial.print(raw_esc_telemdata.statusFlag, BIN);
  // Serial.print(" - ");
  // Serial.println(" ");
}

// new V2 
int checkFletcher16(byte byteBuffer[]) {
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
