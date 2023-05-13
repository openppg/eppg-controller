// Copyright 2021 <Zach Whitehead>
// OpenPPG

#include "../../inc/sp140/voltage-curves.h"
#include "../../inc/sp140/globals.h"

// estimate remaining battery percent using cell manufacturer's voltage curves
float getBatteryPercent(float voltage) {
  float current = telemetryData.amps;
  const float temperature = ambientTempC;
  // calculate cell voltage and current (assume evenly distributed)
  voltage = voltage / cellsInSeries;
  current = current / cellsInParallel;

  // voltage curves measured at 23C - transpose curve by adding cold temperature correction term
  // voltage has greater drop rate when <0C than between 0C and 10C
  if (temperature < 0) {
    // 8.4% voltage drop from 23C to -20C
    voltage = voltage * mapd(temperature, 23, -20, 1, 1.084);
  }
  else if (temperature < 10) {
    // 3.5% voltage drop from 23C to 0C
    voltage = voltage * mapd(temperature, 23, 0, 1, 1.035);
  }

  // find the two voltage curves with the closest current
  const int numCurves = sizeof(VOLTAGE_CURVES) / sizeof(*VOLTAGE_CURVES);
  const VoltageCurve *lower = VOLTAGE_CURVES;
  const VoltageCurve *higher = VOLTAGE_CURVES + 1;
  const VoltageCurve *end = VOLTAGE_CURVES + numCurves - 1; // ensure both pointers are valid
  while (higher != end && current > higher->current) {
    lower++;
    higher++;
  }

  // interpolate between the two curves
  float energyLowerCurrent = getEnergy(voltage, lower->voltageCurve, lower->energyLookup, lower->numPoints);
  float energyHigherCurrent = getEnergy(voltage, higher->voltageCurve, higher->energyLookup, higher->numPoints);
  float remainingEnergy = mapd(current, lower->current, higher->current, energyLowerCurrent, energyHigherCurrent);
  float totalEnergy = mapd(current, lower->current, higher->current, lower->totalEnergy, higher->totalEnergy);
  return constrain(remainingEnergy / totalEnergy * 100, 0, 100);
}


// given a voltage, voltage curve, energy lookup, and length of curve, return the energy (Wh)
float getEnergy(float voltage, const float *voltageCurve, const float *energyLookup, const int len) {
  // find the closest voltage in the curve (voltages are in descending order)
  const float *higherVoltage = voltageCurve;
  const float *lowerVoltage = voltageCurve + 1;
  const float *higherEnergy = energyLookup;
  const float *lowerEnergy = energyLookup + 1;
  const float *end = voltageCurve + len - 1; // ensure both pointers are valid
  while (lowerVoltage != end && voltage < *lowerVoltage) {
    higherVoltage++;
    lowerVoltage++;
    higherEnergy++;
    lowerEnergy++;
  }
  // remove remaining energy in between both data points
  return mapd(voltage, *higherVoltage, *lowerVoltage, *higherEnergy, *lowerEnergy);
}


// inspired by https://github.com/rlogiacco/BatterySense/
// https://www.desmos.com/calculator/7m9lu26vpy
uint8_t battery_sigmoidal(float voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}
