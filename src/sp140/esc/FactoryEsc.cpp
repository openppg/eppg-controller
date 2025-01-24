#ifndef FACTORY_ESC
#define FACTORY_ESC

#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"
#else
  #include "../../inc/sp140/rp2040-config.h"
#endif

#include <Arduino.h>
// #include <CircularBuffer.hpp>
#include "../../../inc/sp140/battery/BatteryData.h"
#include "../../../inc/sp140/esc/FactoryEsc.h"
#include "../../../inc/sp140/esc/EscBase.h"
#include "sp140/structs.h"

FactoryEsc::FactoryEsc(CircularBuffer<float, 50>* pVoltageBuffer) {
    this->pVoltageBuffer = pVoltageBuffer;
}

void FactoryEsc::initESC(int escPin) {
    esc.attach(escPin);
    esc.writeMicroseconds(ESC_DISARMED_PWM);
    setupESCSerial();
}

void FactoryEsc::setupESCSerial() {
    SerialESC.begin(ESC_BAUD_RATE);
    SerialESC.setTimeout(ESC_TIMEOUT);
}

void FactoryEsc::setESCThrottle(int throttlePWM) {
    esc.writeMicroseconds(throttlePWM);
}

void FactoryEsc::readESCTelemetry(BatteryData& batteryData, EscData& escData) {
    prepareESCSerialRead();
    static byte escDataV2[ESC_DATA_V2_SIZE];
    SerialESC.readBytes(escDataV2, ESC_DATA_V2_SIZE);
    handleESCSerialData(batteryData, escData, escDataV2);
}

void FactoryEsc::prepareESCSerialRead() {
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
}

void FactoryEsc::handleESCSerialData(BatteryData& batteryData, EscData& escData, byte buffer[]) {
    // if(sizeof(buffer) != 22) {
  //     Serial.print("wrong size ");
  //     Serial.println(sizeof(buffer));
  //     return; //Ignore malformed packets
  // }

  if (buffer[20] != 255 || buffer[21] != 255) {
    // Serial.println("no stop byte");

    return;  // Stop byte of 65535 not received
  }

  // Check the fletcher checksum
  int checkFletch = checkFletcher16(buffer);
  telem_esc_t& raw_telemdata = escData.raw_esc_telemdata;
  STR_ESC_TELEMETRY_140& telemeteryData = escData.escTelemetryData;
  // checksum
  raw_telemdata.CSUM_HI = buffer[19];
  raw_telemdata.CSUM_LO = buffer[18];

  // TODO: alert if no new data in 3 seconds
  int checkCalc = (int)(((raw_telemdata.CSUM_HI << 8) + raw_telemdata.CSUM_LO));

  // Checksums do not match
  if (checkFletch != checkCalc) {
    return;
  }
  // Voltage
  raw_telemdata.V_HI = buffer[1];
  raw_telemdata.V_LO = buffer[0];

  float voltage = (raw_telemdata.V_HI << 8 | raw_telemdata.V_LO) / 100.0;
  telemeteryData.volts = voltage;  // Voltage

  if (telemeteryData.volts > BATT_MIN_V) {
    telemeteryData.volts += 1.0;  // calibration
  }

  pVoltageBuffer->push(telemeteryData.volts);

  // Temperature
  raw_telemdata.T_HI = buffer[3];
  raw_telemdata.T_LO = buffer[2];

  float rawVal = (float)((raw_telemdata.T_HI << 8) + raw_telemdata.T_LO);

  static int SERIESRESISTOR = 10000;
  static int NOMINAL_RESISTANCE = 10000;
  static int NOMINAL_TEMPERATURE = 25;
  static int BCOEFFICIENT = 3455;

  // convert value to resistance
  float Rntc = (4096 / (float) rawVal) - 1;
  Rntc = SERIESRESISTOR / Rntc;

  // Get the temperature
  float temperature = Rntc / (float) NOMINAL_RESISTANCE;  // (R/Ro)
  temperature = (float) log(temperature);  // ln(R/Ro)
  temperature /= BCOEFFICIENT;  // 1/B * ln(R/Ro)

  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15);  // + (1/To)
  temperature = 1.0 / temperature;  // Invert
  temperature -= 273.15;  // convert to Celsius

  // filter bad values
  if (temperature < 0 || temperature > 200) {
    telemeteryData.temperatureC = __FLT_MIN__;
  } else {
    temperature = (float) trunc(temperature * 100) / 100;  // 2 decimal places
    telemeteryData.temperatureC = temperature;
  }

  // Current
  int16_t _amps = 0;
  _amps = word(buffer[5], buffer[4]);
  telemeteryData.amps = _amps / 12.5;

  // Serial.print("amps ");
  // Serial.print(currentAmpsInput);
  // Serial.print(" - ");

  batteryData.watts = telemeteryData.amps * telemeteryData.volts;

  // Reserved
  raw_telemdata.R0_HI = buffer[7];
  raw_telemdata.R0_LO = buffer[6];

  // eRPM
  raw_telemdata.RPM0 = buffer[11];
  raw_telemdata.RPM1 = buffer[10];
  raw_telemdata.RPM2 = buffer[9];
  raw_telemdata.RPM3 = buffer[8];

  int poleCount = 62;
  uint32_t rawERPM = (raw_telemdata.RPM0 << 24) |
                     (raw_telemdata.RPM1 << 16) |
                     (raw_telemdata.RPM2 << 8) |
                     raw_telemdata.RPM3;
  int currentERPM = static_cast<int>(rawERPM);  // ERPM output
  int currentRPM = currentERPM / poleCount;  // Real RPM output
  telemeteryData.eRPM = currentRPM;

  // Serial.print("RPM ");
  // Serial.print(currentRPM);
  // Serial.print(" - ");

  // Input Duty
  raw_telemdata.DUTYIN_HI = buffer[13];
  raw_telemdata.DUTYIN_LO = buffer[12];

  int throttleDuty = (int)(((raw_telemdata.DUTYIN_HI << 8) + raw_telemdata.DUTYIN_LO) / 10);
  telemeteryData.inPWM = (throttleDuty / 10);  // Input throttle

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
  raw_telemdata.statusFlag = buffer[16];
  telemeteryData.statusFlag = raw_telemdata.statusFlag;
  // Serial.print("status ");
  // Serial.print(raw_esc_telemdata.statusFlag, BIN);
  // Serial.print(" - ");
  // Serial.println(" ");
}

// new V2 ESC checking
int FactoryEsc::checkFletcher16(byte byteBuffer[]) {
    int fCCRC16;
    int i;
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (i = 0; i < 18; i++) {  // Check only first 18 bytes, skip crc bytes
    c0 = (int)(c0 + ((int)byteBuffer[i])) % 255;
    c1 = (int)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = (c1 << 8) | c0;
    return (int)fCCRC16;
}

// for debugging
void FactoryEsc::printRawSentence(byte buffer[]) {
  Serial.print(F("DATA: "));
  for (int i = 0; i < ESC_DATA_V2_SIZE; i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}

// FactoryEsc::~FactoryEsc() {
// }

#endif