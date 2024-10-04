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

  if (currentState == DISARMED) {
    writeDeviceData();
  }

  uint16_t notify_melody[] = { 900, 1976 };
  playMelody(notify_melody, 2);
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
