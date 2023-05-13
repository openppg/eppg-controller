// Copyright 2021 <Zach Whitehead>
// OpenPPG

void updateBatteryPercent() {
  if (millis() < BATTERY_ANALYSIS_END) {
    batteryPercent = analyzeInitialBatteryPercent();
  }
  else {
    // set initial battery energy only once
    if (analysisFlag) {
      initialBatteryEnergy = batteryPercent / 100 * exactCapacityWh;
      analysisFlag = false;
    }
    float newEnergy = initialBatteryEnergy - wattsHoursUsed;
    batteryPercent = newEnergy / exactCapacityWh * 100;
  }
  batteryPercent = constrain(batteryPercent, 0, 100);
}

// get initial battery percent and update moving average
float analyzeInitialBatteryPercent() {
  // use voltage curve to estimate SOC
  float avgVoltage = getBatteryVoltSmoothed();
  float batteryPercent = getBatteryPercent(avgVoltage);

  // wait a little before calculating moving average (due to potential voltage drop from starting the motor)
  if (millis() > BATTERY_ANALYSIS_START) {
    initialSOCMovingAvg = (initialSOCMovingAvg * numInitialSOCReadings + batteryPercent) / (numInitialSOCReadings + 1);
    numInitialSOCReadings++;
    return initialSOCMovingAvg;
  }
  return batteryPercent;
}

// simple set of data points from load testing
// maps voltage to battery percentage
float getBatteryPercent(float voltage) {
  float battPercent = 0;

  if (voltage > 94.8) {
    battPercent = mapd(voltage, 94.8, 99.6, 90, 100);
  } else if (voltage > 93.36) {
    battPercent = mapd(voltage, 93.36, 94.8, 80, 90);
  } else if (voltage > 91.68) {
    battPercent = mapd(voltage, 91.68, 93.36, 70, 80);
  } else if (voltage > 89.76) {
    battPercent = mapd(voltage, 89.76, 91.68, 60, 70);
  } else if (voltage > 87.6) {
    battPercent = mapd(voltage, 87.6, 89.76, 50, 60);
  } else if (voltage > 85.2) {
    battPercent = mapd(voltage, 85.2, 87.6, 40, 50);
  } else if (voltage > 82.32) {
    battPercent = mapd(voltage, 82.32, 85.2, 30, 40);
  } else if (voltage > 80.16) {
    battPercent = mapd(voltage, 80.16, 82.32, 20, 30);
  } else if (voltage > 78) {
    battPercent = mapd(voltage, 78, 80.16, 10, 20);
  } else if (voltage > 60.96) {
    battPercent = mapd(voltage, 60.96, 78, 0, 10);
  }
  return constrain(battPercent, 0, 100);
}


// inspired by https://github.com/rlogiacco/BatterySense/
// https://www.desmos.com/calculator/7m9lu26vpy
uint8_t battery_sigmoidal(float voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}
