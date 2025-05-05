#include "sp140/esc.h"
#include "sp140/globals.h"
#include <CircularBuffer.hpp>

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "driver/twai.h"

#define ESC_RX_PIN 3  // CAN RX pin to transceiver
#define ESC_TX_PIN 2  // CAN TX pin to transceiver
#define LOCAL_NODE_ID 0x01  // The ID on the network of this device

#define TELEMETRY_TIMEOUT_MS 50  // Threshold to determine stale ESC telemetry in ms

static CanardAdapter adapter;
static uint8_t memory_pool[1024] __attribute__((aligned(8)));
static SineEsc esc(adapter);
static unsigned long lastSuccessfulCommTimeMs = 0;  // Store millis() time of last successful ESC comm


STR_ESC_TELEMETRY_140 escTelemetryData = {
  .escState = TelemetryState::NOT_CONNECTED
};

void initESC() {
  escTwaiInitialized = setupTWAI();
  if (!escTwaiInitialized) {
    USBSerial.println("ESC TWAI Initialization failed. ESC will not function.");
    return;  // Can't proceed without TWAI driver
  }

  adapter.begin(memory_pool, sizeof(memory_pool));
  adapter.setLocalNodeId(LOCAL_NODE_ID);
  esc.begin(0x20);  // Default ID for the ESC

  // Set idle throttle only if ESC is found
  const uint16_t IdleThrottle_us = 10000;  // 1000us (0.1us resolution)
  esc.setThrottleSettings2(IdleThrottle_us);
  adapter.processTxRxOnce();
  delay(20);  // Wait for ESC to process the command
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
  // Only proceed if TWAI is initialized
  if (!escTwaiInitialized) { return; }  // NOLINT(whitespace/newline)

  // Store the last known ESC timestamp before checking for new data
  unsigned long previousEscReportedTimeMs = escTelemetryData.lastUpdateMs;

  const SineEscModel &model = esc.getModel();

  if (model.hasSetThrottleSettings2Response) {
    const sine_esc_SetThrottleSettings2Response *res = &model.setThrottleSettings2Response;

    unsigned long newEscReportedTimeMs = res->time_10ms * 10;

    // Check if the timestamp from the ESC has actually changed
    if (newEscReportedTimeMs != previousEscReportedTimeMs) {
      // Timestamp is new, process the telemetry data
      escTelemetryData.lastUpdateMs = newEscReportedTimeMs;

      // Update telemetry data
      escTelemetryData.volts = res->voltage / 10.0f;
      escTelemetryData.amps = res->bus_current / 10.0f;
      escTelemetryData.mos_temp = res->mos_temp / 10.0f;
      escTelemetryData.cap_temp = res->cap_temp / 10.0f;
      escTelemetryData.mcu_temp = res->mcu_temp / 10.0f;
      escTelemetryData.motor_temp = res->motor_temp / 10.0f;
      escTelemetryData.eRPM = res->speed;
      escTelemetryData.inPWM = res->recv_pwm / 10.0f;
      watts = escTelemetryData.amps * escTelemetryData.volts;

      // Temperature states
      escTelemetryData.mos_state = checkTempState(escTelemetryData.mos_temp, COMP_ESC_MOS);
      escTelemetryData.mcu_state = checkTempState(escTelemetryData.mcu_temp, COMP_ESC_MCU);
      escTelemetryData.cap_state = checkTempState(escTelemetryData.cap_temp, COMP_ESC_CAP);
      escTelemetryData.motor_state = checkTempState(escTelemetryData.motor_temp, COMP_MOTOR);
      escTelemetryData.temp_sensor_error =
        (escTelemetryData.mos_state == TEMP_INVALID) ||
        (escTelemetryData.mcu_state == TEMP_INVALID) ||
        (escTelemetryData.cap_state == TEMP_INVALID) ||
        (escTelemetryData.motor_state == TEMP_INVALID);

      // Record the time of this successful communication using the local clock
      lastSuccessfulCommTimeMs = millis();
    }  // else: Timestamp hasn't changed, treat as stale data, don't update local timer or telemetry

  } else {
      // If we are connected but don't have a response, perhaps mark as stale or log?
      // For now, do nothing
  }

  // Update connection state based on time since last successful communication
  unsigned long currentTimeMs = millis();
  if (lastSuccessfulCommTimeMs == 0 || (currentTimeMs - lastSuccessfulCommTimeMs) > TELEMETRY_TIMEOUT_MS) {
    if (escTelemetryData.escState != TelemetryState::NOT_CONNECTED) {
      // Log state change only if it actually changed
      USBSerial.printf("ESC State: %d -> NOT_CONNECTED (Timeout)\n", escTelemetryData.escState);
      escTelemetryData.escState = TelemetryState::NOT_CONNECTED;
      // Optional: Consider resetting telemetry values here if needed when disconnected
    }
  } else {
    if (escTelemetryData.escState != TelemetryState::CONNECTED) {
      // Log state change only if it actually changed
      USBSerial.printf("ESC State: %d -> CONNECTED\n", escTelemetryData.escState);
      escTelemetryData.escState = TelemetryState::CONNECTED;
    }
  }

  adapter.processTxRxOnce();  // Process CAN messages
}

// CAN specific setup
bool setupTWAI() {
  // Check if already installed by checking status
  twai_status_info_t status_info;
  esp_err_t status_result = twai_get_status_info(&status_info);

  if (status_result == ESP_OK) {
    // Driver is installed. We can check its state if needed.
    USBSerial.printf("TWAI driver already installed. State: %d\n", status_info.state);
    // If it's stopped, maybe we need to start it? For now, assume it's okay.
    // if (status_info.state == TWAI_STATE_STOPPED) { twai_start(); }
    return true;  // Already initialized
  } else if (status_result != ESP_ERR_INVALID_STATE) {
    // An error other than "not installed" occurred
    USBSerial.printf("Error checking TWAI status: %s\n", esp_err_to_name(status_result));
    return false;  // Don't proceed if status check failed unexpectedly
  }
  // If status_result was ESP_ERR_INVALID_STATE, proceed with installation

  USBSerial.println("TWAI driver not installed. Proceeding with installation...");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
                                      (gpio_num_t)ESC_TX_PIN,
                                      (gpio_num_t)ESC_RX_PIN,
                                      TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t install_err = twai_driver_install(&g_config, &t_config, &f_config);
  if (install_err == ESP_OK) {
      USBSerial.println("TWAI Driver installed");
  } else {
      USBSerial.printf("Failed to install TWAI driver: %s\n", esp_err_to_name(install_err));
      return false;
  }

  esp_err_t start_err = twai_start();
  if (start_err == ESP_OK) {
      USBSerial.println("TWAI Driver started");
  } else {
      USBSerial.printf("Failed to start TWAI driver: %s\n", esp_err_to_name(start_err));
      // Attempt to uninstall if start failed
      twai_driver_uninstall();
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
      USBSerial.println("Failed to reconfigure CAN alerts");
      // Consider this non-fatal for now? Or return false?
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

float getHighestTemp(const STR_ESC_TELEMETRY_140& telemetry) {
  return max(telemetry.motor_temp, max(telemetry.mos_temp, telemetry.cap_temp));
}

TempState checkTempState(float temp, TempComponent component) {
  // Check for invalid temperature readings
  if (temp < -50 || temp > 200) {
      return TEMP_INVALID;
  }

  switch (component) {
    case COMP_ESC_MOS:
      return temp >= ESC_MOS_CRIT ? TEMP_CRITICAL :
              temp >= ESC_MOS_WARN ? TEMP_WARNING : TEMP_NORMAL;

    case COMP_ESC_MCU:
      return temp >= ESC_MCU_CRIT ? TEMP_CRITICAL :
              temp >= ESC_MCU_WARN ? TEMP_WARNING : TEMP_NORMAL;

    case COMP_ESC_CAP:
      return temp >= ESC_CAP_CRIT ? TEMP_CRITICAL :
              temp >= ESC_CAP_WARN ? TEMP_WARNING : TEMP_NORMAL;

    case COMP_MOTOR:
      return temp >= MOTOR_CRIT ? TEMP_CRITICAL :
              temp >= MOTOR_WARN ? TEMP_WARNING : TEMP_NORMAL;

    default:
      return TEMP_INVALID;
  }
}
