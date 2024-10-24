#include "sp140/esc.h"
#include "sp140/globals.h"
#include <CircularBuffer.hpp>

#ifndef CAN_PIO
  #include <Servo.h>  // For generating PWM for ESC

  Servo esc;  // Creating a servo class with name of esc
#else
  #pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  #include "driver/twai.h"

  #define RX_PIN 10  // CAN RX pin to transceiver
  #define TX_PIN 9  // CAN TX pin to transceiver
  #define LOCAL_NODE_ID 0x01  // The ID on the network of this device

  static CanardAdapter adapter;
  static uint8_t memory_pool[1024] __attribute__((aligned(8)));
  static SineEsc esc(adapter);
  static bool twaiDriverInstalled = false;
#endif

extern CircularBuffer<float, 50> voltageBuffer;

STR_ESC_TELEMETRY_140 escTelemetryData;
static telem_esc_t raw_esc_telemdata;

bool initESC(int escPin) {
  setupTWAI();
  adapter.begin(memory_pool, sizeof(memory_pool));
  adapter.setLocalNodeId(LOCAL_NODE_ID);
  esc.begin(0x20);  // Default ID for the ESC

  // Find ESC
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
  delay(1000); // Wait for ESC to process the command
  adapter.processTxRxOnce();

  return true;
}

void setupESCSerial() {
#ifndef CAN_PIO
  USBSerialESC.begin(ESC_BAUD_RATE);
  USBSerialESC.setTimeout(ESC_TIMEOUT);
#endif
}

void setESCThrottle(int throttlePWM) {
  // TODO: Calibrate and use constants
  uint16_t scaledThrottle = map(throttlePWM, 1000, 2000, 10000, 14000);
  //USBSerial.print("Setting throttle to: ");
  //USBSerial.println(scaledThrottle);
  esc.setThrottleSettings2(scaledThrottle);
  // Remove the immediate call to processTxRxOnce here
  adapter.processTxRxOnce();  // Process CAN messages
}

void readESCTelemetry() {
  //USBSerial.println("readESCTelemetry");

  // TODO: Skip if the esc is not initialized?

  const SineEscModel &model = esc.getModel();

  if (model.hasSetThrottleSettings2Response) {
    const sine_esc_SetThrottleSettings2Response *res = &model.setThrottleSettings2Response;
    dumpThrottleResponse(res);
    // Voltage
    escTelemetryData.volts = res->voltage / 10.0f;
    voltageBuffer.push(escTelemetryData.volts);

    // Current
    escTelemetryData.amps = res->current / 10.0f;

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
  }

  adapter.processTxRxOnce();  // Process CAN messages
}

// new V2 ESC checking
int checkFletcher16(byte byteBuffer[]) {
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

// CAN specific setup
bool setupTWAI() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
                                      (gpio_num_t)TX_PIN,
                                      (gpio_num_t)RX_PIN,
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
