#include "sp140/esc.h"
#include "sp140/globals.h"
#include <CircularBuffer.hpp>

#ifndef CAN_PIO
  #include <Servo.h>  // For generating PWM for ESC

  Servo esc;  // Creating a servo class with name of esc
#else
  #pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  #include "driver/twai.h"

  #define ESC_RX_PIN 3  // CAN RX pin to transceiver
  #define ESC_TX_PIN 2  // CAN TX pin to transceiver
  #define LOCAL_NODE_ID 0x01  // The ID on the network of this device

  static CanardAdapter adapter;
  static uint8_t memory_pool[1024] __attribute__((aligned(8)));
  static SineEsc esc(adapter);
  static bool twaiDriverInstalled = false;
#endif

extern CircularBuffer<float, 50> voltageBuffer;

STR_ESC_TELEMETRY_140 escTelemetryData;

bool initESC(int escPin) {
  setupTWAI();
  adapter.begin(memory_pool, sizeof(memory_pool));
  adapter.setLocalNodeId(LOCAL_NODE_ID);
  esc.begin(0x20);  // Default ID for the ESC

  // Find ESC
  // TODO better handling of this
  int attempts = 0;
  const int maxAttempts = 10;
  while (!esc.getModel().hasGetHardwareInfoResponse && attempts < maxAttempts) {
    esc.getHardwareInfo();
    adapter.processTxRxOnce();
    USBSerial.println("Waiting for ESC");
    delay(200);
    attempts++;
  }

  // Set idle throttle
  const uint16_t IdleThrottle_us = 10000; // 1000us (0.1us resolution)
  esc.setThrottleSettings2(IdleThrottle_us);
  delay(500); // Wait for ESC to process the command
  adapter.processTxRxOnce();

  return true;
}

void setESCThrottle(int throttlePWM) {
  // Input validation
  if (throttlePWM < 1000 || throttlePWM > 2000) {
    return;  // Ignore invalid throttle values
  }

  // Direct calculation: multiply by 10 to convert μs to 0.1μs resolution
  uint16_t scaledThrottle = throttlePWM * 10;
  esc.setThrottleSettings2(scaledThrottle);
}

void readESCTelemetry() {
  //USBSerial.println("readESCTelemetry");

  // TODO: Skip if the esc is not initialized?

  const SineEscModel &model = esc.getModel();

  if (model.hasSetThrottleSettings2Response) {
    const sine_esc_SetThrottleSettings2Response *res = &model.setThrottleSettings2Response;
    //dumpThrottleResponse(res);
    // Voltage
    escTelemetryData.volts = res->voltage / 10.0f;
    voltageBuffer.push(escTelemetryData.volts);

    // Current
    escTelemetryData.amps = res->bus_current / 10.0f;

    // Temperature (using MOS temperature as an example)
    escTelemetryData.mos_temp = res->mos_temp / 10.0f;
    // cap_temp
    escTelemetryData.cap_temp = res->cap_temp / 10.0f;
    // mcu_temp
    escTelemetryData.mcu_temp = res->mcu_temp / 10.0f;

    // eRPM (assuming 'speed' is in eRPM)
    escTelemetryData.eRPM = res->speed;

    // Input PWM (assuming 'recv_pwm' is equivalent)
    escTelemetryData.inPWM = res->recv_pwm / 10.0f;

    // Status flags (combining running_error and selfcheck_error)
    //escTelemetryData.statusFlag = (res->running_error & 0xFF) | ((res->selfcheck_error & 0xFF) << 8);

    // Calculate watts
    watts = escTelemetryData.amps * escTelemetryData.volts;

    // Update timestamp
    escTelemetryData.lastUpdateMs = millis();

    // debug the esc recpwm
    //USBSerial.print(", ");
    //USBSerial.println(res->comm_pwm);
    // Send to queue for BLE updates
    if (escTelemetryQueue != NULL) {
      xQueueOverwrite(escTelemetryQueue, &escTelemetryData);  // Always use latest data
    }
  }

  adapter.processTxRxOnce();  // Process CAN messages
}

// CAN specific setup
bool setupTWAI() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
                                      (gpio_num_t)ESC_TX_PIN,
                                      (gpio_num_t)ESC_RX_PIN,
                                      TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      USBSerial.println("Driver installed");
  } else {
      USBSerial.println("Failed to install driver");
      return false;
  }

  if (twai_start() == ESP_OK) {
      USBSerial.println("Driver started");
  } else {
      USBSerial.println("Failed to start driver");
      return false;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA
                              | TWAI_ALERT_ERR_PASS
                              | TWAI_ALERT_BUS_ERROR
                              | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
      USBSerial.println("CAN Alerts reconfigured");
  } else {
      USBSerial.println("Failed to reconfigure alerts");
      return false;
  }

  return true;
}

void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res) {
  USBSerial.println("Got SetThrottleSettings2 response");

  USBSerial.print("\trecv_pwm: ");
  USBSerial.println(res->recv_pwm);

  USBSerial.print("\tcomm_pwm: ");
  USBSerial.println(res->comm_pwm);

  USBSerial.print("\tspeed: ");
  USBSerial.println(res->speed);

  USBSerial.print("\tcurrent: ");
  USBSerial.println(res->current);

  USBSerial.print("\tbus_current: ");
  USBSerial.println(res->bus_current);

  USBSerial.print("\tvoltage: ");
  USBSerial.println(res->voltage);

  USBSerial.print("\tv_modulation: ");
  USBSerial.println(res->v_modulation);

  USBSerial.print("\tmos_temp: ");
  USBSerial.println(res->mos_temp);

  USBSerial.print("\tcap_temp: ");
  USBSerial.println(res->cap_temp);

  USBSerial.print("\tmcu_temp: ");
  USBSerial.println(res->mcu_temp);

  USBSerial.print("\trunning_error: ");
  USBSerial.println(res->running_error);

  USBSerial.print("\tselfcheck_error: ");
  USBSerial.println(res->selfcheck_error);

  USBSerial.print("\tmotor_temp: ");
  USBSerial.println(res->motor_temp);

  USBSerial.print("\ttime_10ms: ");
  USBSerial.println(res->time_10ms);
}

void dumpESCMessages(void) {
  const SineEscModel &model = esc.getModel();

  if (model.hasGetHardwareInfoResponse) {
    USBSerial.println("Got HwInfo response");

    const sine_esc_GetHwInfoResponse *b = &model.getHardwareInfoResponse;
    USBSerial.print("\thardware_id: ");
    USBSerial.println(b->hardware_id, HEX);
    USBSerial.print("\tbootloader_version: ");
    USBSerial.println(b->bootloader_version, HEX);
    USBSerial.print("\tapp_version: ");
    USBSerial.println(b->app_version, HEX);
  }

  if (model.hasSetThrottleSettings2Response) {
    dumpThrottleResponse(&model.setThrottleSettings2Response);
  }

  if (model.hasSetRotationSpeedSettingsResponse) {
    USBSerial.println("Got SetRotationSpeedSettings response");
  }
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * This function takes a voltage level as input and returns the corresponding
 * battery percentage. It uses a lookup table (`batteryLevels`) to map voltage
 * levels to percentages. Thats based  on a simple set of data points from load testing.
 * If the input voltage is greater than a threshold
 * voltage, linear interpolation is used to calculate the percentage between
 * two consecutive voltage-percentage mappings.
 *
 * @param voltage The input voltage level.
 * @return The calculated battery percentage (0-100).
 */
float getBatteryPercent(float voltage) {
  // Calculate the number of voltage-percentage mappings
  int numLevels = sizeof(batteryLevels) / sizeof(BatteryVoltagePoint);

  // Handle edge cases where the voltage is outside the defined range
  if (voltage >= batteryLevels[0].voltage) {
    return batteryLevels[0].percent;
  } else if (voltage <= batteryLevels[numLevels - 1].voltage) {
    return batteryLevels[numLevels - 1].percent;
  }

  // Iterate through the voltage-percentage mappings
  for (int i = 0; i < numLevels - 1; i++) {
    // Check if the input voltage is between the current and next mapping
    if (voltage <= batteryLevels[i].voltage && voltage > batteryLevels[i + 1].voltage) {
      // Interpolate the percentage between the current and next mapping
      return mapDouble(voltage, batteryLevels[i + 1].voltage, batteryLevels[i].voltage,
                        batteryLevels[i + 1].percent, batteryLevels[i].percent);
    }
  }

  return 0;  // Fallback, should never reach here
}
