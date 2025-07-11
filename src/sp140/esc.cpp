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
  .escState = TelemetryState::NOT_CONNECTED,
  .running_error = 0,
  .selfcheck_error = 0
};

/**
 * Initialize the ESC communication system
 * Sets up the CAN bus (TWAI) interface and configures the ESC with default settings
 * Must be called before any other ESC functions
 */
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

/**
 * Set the ESC throttle value
 * @param throttlePWM Throttle value in microseconds (1000-2000)
 *                    1000 = minimum throttle, 2000 = maximum throttle
 *
 * Important: The ESC requires messages at least every 300ms or it will reset
 */
void setESCThrottle(int throttlePWM) {
  // Input validation
  if (throttlePWM < 1000 || throttlePWM > 2000) {
    return;  // Ignore invalid throttle values
  }

  // Direct calculation: multiply by 10 to convert μs to 0.1μs resolution
  uint16_t scaledThrottle = throttlePWM * 10;
  esc.setThrottleSettings2(scaledThrottle);
}

/**
 * Read telemetry data from the ESC
 * Updates the global escTelemetryData structure with current values
 * Should be called regularly (every 20-50ms) to maintain connection
 * Also monitors connection state and sets NOT_CONNECTED if timeout occurs
 */
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

      // Store error bitmasks
      escTelemetryData.running_error = res->running_error;
      escTelemetryData.selfcheck_error = res->selfcheck_error;

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

/**
 * Setup the ESP32 TWAI (Two-Wire Automotive Interface) for CAN communication
 * Configures the CAN bus at 1Mbps for communication with the ESC
 * @return true if setup was successful, false otherwise
 */
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

/**
 * Debug function to dump ESC throttle response data to serial
 * @param res Pointer to the throttle response structure from ESC
 */
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
  USBSerial.print(res->running_error);
  USBSerial.print(" (");
  USBSerial.print(decodeRunningError(res->running_error));
  USBSerial.println(")");

  USBSerial.print("\tselfcheck_error: ");
  USBSerial.print(res->selfcheck_error);
  USBSerial.print(" (");
  USBSerial.print(decodeSelfCheckError(res->selfcheck_error));
  USBSerial.println(")");

  USBSerial.print("\tmotor_temp: ");
  USBSerial.println(res->motor_temp);

  USBSerial.print("\ttime_10ms: ");
  USBSerial.println(res->time_10ms);
}

/**
 * Debug function to dump all available ESC messages to serial
 * Displays hardware info, throttle response, and rotation speed settings if available
 */
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

/**
 * Map a double value from one range to another
 * @param x Input value to map
 * @param in_min Minimum of input range
 * @param in_max Maximum of input range
 * @param out_min Minimum of output range
 * @param out_max Maximum of output range
 * @return Mapped value in output range
 */
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Get the highest temperature reading from all ESC sensors
 * @param telemetry ESC telemetry data structure
 * @return The highest temperature value among motor, MOSFET, and capacitor temps
 */
float getHighestTemp(const STR_ESC_TELEMETRY_140& telemetry) {
  return max(telemetry.motor_temp, max(telemetry.mos_temp, telemetry.cap_temp));
}

/**
 * Check temperature state for a specific component
 * @param temp Temperature value to check
 * @param component Component type (ESC_MOS, ESC_MCU, ESC_CAP, or MOTOR)
 * @return Temperature state (NORMAL, WARNING, CRITICAL, or INVALID)
 */
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

/**
 * Decode running error bitmask into human-readable string
 * @param errorCode 16-bit running error code from ESC
 * @return String containing decoded error messages
 */
String decodeRunningError(uint16_t errorCode) {
  if (errorCode == 0) {
    return "no_errors";
  }

  String result = "";
  bool firstError = true;

  // Define error messages for each bit
  const char* errorMessages[] = {
    "over_current_protect",           // Bit 0: Over current/short circuit protection occurs
    "locked_rotor_protect",           // Bit 1: Locked-rotor protection occurs
    "over_temp_protect",              // Bit 2: Over-temperature protection
    "pwm_throttle_lost",              // Bit 3: PWM throttle lost pulse
    "no_load",                        // Bit 4: No load
    "throttle_saturation",            // Bit 5: Throttle saturation
    "over_volt_protect",              // Bit 6: Over voltage protection
    "voltage_drop",                   // Bit 7: Voltage drop
    "comm_throttle_loss",             // Bit 8: Communication throttle loss
    "undef_9",                        // Bit 9: Undefined
    "undef_10",                       // Bit 10: Undefined
    "undef_11",                       // Bit 11: Undefined
    "undef_12",                       // Bit 12: Undefined
    "undef_13",                       // Bit 13: Undefined
    "undef_14",                       // Bit 14: Undefined
    "undef_15"                        // Bit 15: Undefined
  };

  for (int i = 0; i < 16; i++) {
    if (errorCode & (1 << i)) {
      if (!firstError) {
        result += ", ";
      }
      result += errorMessages[i];
      firstError = false;
    }
  }

  return result;
}

/**
 * Decode self-check error bitmask into human-readable string
 * @param errorCode 16-bit self-check error code from ESC
 * @return String containing decoded error messages
 */
String decodeSelfCheckError(uint16_t errorCode) {
  if (errorCode == 0) {
    return "no_errors";
  }

  String result = "";
  bool firstError = true;

  // Define error messages for each bit
  const char* errorMessages[] = {
    "motor_i_out_bad",                     // Bit 0: Motor line current output abnormal
    "total_i_out_bad",                     // Bit 1: Total current output abnormal
    "motor_v_out_bad",                     // Bit 2: Motor line voltage abnormal
    "cap_ntc_bad",                         // Bit 3: Electrolytic capacitor NTC output abnormal
    "mos_ntc_bad",                         // Bit 4: MOS Tube NTC output abnormal
    "bus_v_range",                         // Bit 5: Bus voltage over/under voltage
    "bus_v_sample_bad",                    // Bit 6: Bus voltage sampling abnormal
    "motor_z_too_low",                     // Bit 7: Motor wire loop impedance too low
    "motor_z_too_high",                    // Bit 8: Motor wire loop impedance too large
    "motor_v_det1_bad",                    // Bit 9: Motor line voltage detection circuit abnormal 1
    "motor_v_det2_bad",                    // Bit 10: Motor line voltage detection circuit abnormal 2
    "motor_i_det2_bad",                    // Bit 11: Motor line current detection circuit abnormal 02
    "undef_12",                            // Bit 12: undefined
    "sw_hw_incompatible",                  // Bit 13: Software and hardware versions incompatible
    "bootloader_unsupported",              // Bit 14: Boot loader unsupported
    "undef_15"                             // Bit 15: Undefined
  };

  for (int i = 0; i < 16; i++) {
    if (errorCode & (1 << i)) {
      if (!firstError) {
        result += ", ";
      }
      result += errorMessages[i];
      firstError = false;
    }
  }

  return result;
}

/**
 * Check if there are any running errors
 * @param errorCode 16-bit running error code from ESC
 * @return true if any error bits are set
 */
bool hasRunningError(uint16_t errorCode) {
  return errorCode != 0;
}

/**
 * Check if there are any self-check errors
 * @param errorCode 16-bit self-check error code from ESC
 * @return true if any error bits are set
 */
bool hasSelfCheckError(uint16_t errorCode) {
  return errorCode != 0;
}

/**
 * Check if there are any critical running errors
 * Critical errors are high-priority faults that require immediate attention
 * @param errorCode 16-bit running error code from ESC
 * @return true if any critical error bits are set
 */
bool hasCriticalRunningError(uint16_t errorCode) {
  // Define critical error bits (High level errors from documentation)
  // Bits 0,1,2,3,6,7 are High priority
  const uint16_t criticalBits = 0x00CF;  // Bits 0,1,2,3,6,7 (High priority)
  // Bit 4 (no_load) and Bit 8 (comm_throttle_loss) are Low priority
  // Bit 5 (throttle_saturation) is Middle priority - treat as non-critical for now

  return (errorCode & criticalBits) != 0;
}

/**
 * Check if there are any critical self-check errors
 * All self-check errors are considered critical as they indicate hardware issues
 * @param errorCode 16-bit self-check error code from ESC
 * @return true if any error bits are set (all self-check errors are critical)
 */
bool hasCriticalSelfCheckError(uint16_t errorCode) {
  return errorCode != 0;  // All self-check errors are critical
}
