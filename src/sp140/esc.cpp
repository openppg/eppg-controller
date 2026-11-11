#include "sp140/esc.h"
#include "sp140/globals.h"
#include "sp140/esc_config_relay.h"
#include "sp140/esc_flasher_relay.h"
#include "sp140/time_utils.h"
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

// Expose the single shared adapter to the ESC config relay module.
CanardAdapter& escAdapter() { return adapter; }
static unsigned long lastSuccessfulCommTimeMs = 0;  // Store timeMillis() of last successful ESC comm
// Flag set by requestEscHardwareInfo() (may be called from BLE task),
// consumed safely inside readESCTelemetry() on the throttle task.
static volatile bool s_hwInfoRequested = false;

enum class PendingEscTone : uint8_t {
  NONE = 0,
  ARM,
  DISARM,
};

static volatile EscStatusLightMode sRequestedStatusLightMode =
    EscStatusLightMode::OFF;
static EscStatusLightMode sLastSentStatusLightMode = EscStatusLightMode::OFF;
static unsigned long sLastStatusLightSendMs = 0;
static bool sHaveSentStatusLight = false;
static volatile PendingEscTone sPendingEscTone = PendingEscTone::NONE;

// ESC runtime accumulation — unwraps the uint16 time_10ms counter (~10.9 min period)
// Unsigned subtraction naturally handles wrap: (uint16_t)(current - last) is correct even across rollover.
static uint16_t sEscLastTime10ms = 0;
static uint32_t sEscAccumulatedRuntimeMs = 0;
static bool sEscFirstUpdate = true;

namespace {

constexpr uint8_t kEscToneLow = 2;
constexpr uint8_t kEscToneHigh = 8;
constexpr uint8_t kEscToneVolumePct = 80;
constexpr uint8_t kEscToneDuration10ms = 20;

// Caller must pass ARM or DISARM (never NONE).
void buildEscMotorTone(uint8_t* out, PendingEscTone tone) {
  if (tone == PendingEscTone::ARM) {
    SineEsc::makeBeepEntry(&out[0], kEscToneLow, kEscToneDuration10ms, kEscToneVolumePct);
    SineEsc::makeBeepEntry(&out[3], kEscToneHigh, kEscToneDuration10ms, kEscToneVolumePct);
  } else {
    SineEsc::makeBeepEntry(&out[0], kEscToneHigh, kEscToneDuration10ms, kEscToneVolumePct);
    SineEsc::makeBeepEntry(&out[3], kEscToneLow, kEscToneDuration10ms, kEscToneVolumePct);
  }
}

unsigned long escStatusLightRefreshMs(EscStatusLightMode mode) {
  switch (mode) {
  case EscStatusLightMode::FLIGHT_NOMINAL:
  case EscStatusLightMode::FLIGHT_WARNING:
  case EscStatusLightMode::FLIGHT_CRITICAL:
    return 0;
  case EscStatusLightMode::DISARMED_NOMINAL:
  case EscStatusLightMode::DISARMED_WARNING:
  case EscStatusLightMode::DISARMED_CRITICAL:
    return 0;
  case EscStatusLightMode::OFF:
  default:
    return 0;
  }
}

void sendEscStatusLight(EscStatusLightMode mode) {
  switch (mode) {
  case EscStatusLightMode::DISARMED_NOMINAL: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_GREEN_BREATH, 60),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 20),
    };
    esc.setLedControl(pattern, 2);
    break;
  }
  case EscStatusLightMode::DISARMED_WARNING: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_YELLOW_BREATH, 60),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 20),
    };
    esc.setLedControl(pattern, 2);
    break;
  }
  case EscStatusLightMode::DISARMED_CRITICAL: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_RED_BREATH, 60),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 20),
    };
    esc.setLedControl(pattern, 2);
    break;
  }
  case EscStatusLightMode::FLIGHT_NOMINAL: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_GREEN, 1),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 2),
      SineEsc::makeLedControlEntry(SineEsc::LED_GREEN, 1),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 30),
    };
    esc.setLedControl(pattern, 4);
    break;
  }
  case EscStatusLightMode::FLIGHT_WARNING: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_YELLOW, 1),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 2),
      SineEsc::makeLedControlEntry(SineEsc::LED_YELLOW, 1),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 30),
    };
    esc.setLedControl(pattern, 4);
    break;
  }
  case EscStatusLightMode::FLIGHT_CRITICAL: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_RED, 1),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 2),
      SineEsc::makeLedControlEntry(SineEsc::LED_RED, 1),
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 30),
    };
    esc.setLedControl(pattern, 4);
    break;
  }
  case EscStatusLightMode::OFF:
  default: {
    const uint16_t pattern[] = {
      SineEsc::makeLedControlEntry(SineEsc::LED_OFF, 20),
    };
    esc.setLedControl(pattern, 1);
    break;
  }
  }
}

void syncEscOutputs() {
  const bool escConnected =
      escTwaiInitialized &&
      escTelemetryData.escState == TelemetryState::CONNECTED;

  if (!escConnected) {
    sPendingEscTone = PendingEscTone::NONE;
    sHaveSentStatusLight = false;
    sLastSentStatusLightMode = EscStatusLightMode::OFF;
    sLastStatusLightSendMs = 0;
    return;
  }

  const PendingEscTone pendingTone = sPendingEscTone;
  if (pendingTone != PendingEscTone::NONE) {
    uint8_t beepData[6];
    buildEscMotorTone(beepData, pendingTone);
    esc.setMotorSound(beepData, 2);
    sPendingEscTone = PendingEscTone::NONE;
  }

  const EscStatusLightMode requestedMode = sRequestedStatusLightMode;
  const unsigned long now = timeMillis();
  const bool needsRefresh =
      sHaveSentStatusLight &&
      escStatusLightRefreshMs(requestedMode) > 0 &&
      (now - sLastStatusLightSendMs) >= escStatusLightRefreshMs(requestedMode);

  if (!sHaveSentStatusLight || requestedMode != sLastSentStatusLightMode ||
      needsRefresh) {
    sendEscStatusLight(requestedMode);
    sLastSentStatusLightMode = requestedMode;
    sLastStatusLightSendMs = now;
    sHaveSentStatusLight = true;
  }
}

}  // namespace


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
  vTaskDelay(pdMS_TO_TICKS(20));  // Wait for ESC to process the command

  // Register the ESC config-relay + firmware-relay nodes on the shared adapter.
  escConfigRelayInit();
  escFlasherRelayInit();
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

  const SineEscModel &model = esc.getModel();

  if (model.hasSetThrottleSettings2Response) {
    const sine_esc_SetThrottleSettings2Response *res = &model.setThrottleSettings2Response;

    uint16_t rawTime10ms = res->time_10ms;

    // Check if the timestamp from the ESC has actually changed (stale-data guard)
    if (rawTime10ms != sEscLastTime10ms) {
      // Unwrap the uint16 counter using unsigned subtraction — naturally correct across rollover.
      // e.g. last=65000, current=100: (uint16_t)(100-65000) = 636 ticks = 6360ms elapsed.
      if (!sEscFirstUpdate) {
        uint16_t deltaTicks = rawTime10ms - sEscLastTime10ms;
        sEscAccumulatedRuntimeMs += (uint32_t)deltaTicks * 10UL;
      }
      sEscFirstUpdate = false;
      sEscLastTime10ms = rawTime10ms;

      // Timestamp is new, process the telemetry data
      escTelemetryData.lastUpdateMs = sEscAccumulatedRuntimeMs;
      escTelemetryData.esc_runtime_ms = sEscAccumulatedRuntimeMs;

      // Update telemetry data
      escTelemetryData.volts = res->voltage / 10.0f;
      escTelemetryData.amps = res->bus_current / 10.0f;
      escTelemetryData.mos_temp = res->mos_temp / 10.0f;
      escTelemetryData.cap_temp = res->cap_temp / 10.0f;
      escTelemetryData.mcu_temp = res->mcu_temp / 10.0f;
      // Filter motor temp - only update if sensor is connected (valid range: -20°C to 140°C)
      // Disconnected sensor reads ~149°C (thermistor pulled high)
      float rawMotorTemp = res->motor_temp / 10.0f;
      if (isMotorTempValidC(rawMotorTemp)) {
        escTelemetryData.motor_temp = rawMotorTemp;
      } else {
        // Store invalid motor temp as NaN. Downstream consumers can skip on isnan().
        escTelemetryData.motor_temp = NAN;
      }
      escTelemetryData.phase_current = res->current / 10.0f;
      escTelemetryData.eRPM = res->speed;
      escTelemetryData.inPWM = res->recv_pwm / 10.0f;
      escTelemetryData.comm_pwm = res->comm_pwm;
      escTelemetryData.v_modulation = res->v_modulation;
      watts = escTelemetryData.amps * escTelemetryData.volts;

      // Store error bitmasks
      escTelemetryData.running_error = res->running_error;
      escTelemetryData.selfcheck_error = res->selfcheck_error;

      // Record the time of this successful communication using the local clock
      lastSuccessfulCommTimeMs = timeMillis();
    }  // else: Timestamp hasn't changed, treat as stale data, don't update local timer or telemetry

  } else {
      // If we are connected but don't have a response, perhaps mark as stale or log?
      // For now, do nothing
  }

  // Update connection state based on time since last successful communication
  unsigned long currentTimeMs = timeMillis();
  if (lastSuccessfulCommTimeMs == 0 || (currentTimeMs - lastSuccessfulCommTimeMs) > TELEMETRY_TIMEOUT_MS) {
    if (escTelemetryData.escState != TelemetryState::NOT_CONNECTED) {
      // Log state change only if it actually changed
      USBSerial.printf("ESC State: %d -> NOT_CONNECTED (Timeout)\n", static_cast<int>(escTelemetryData.escState));
      escTelemetryData.escState = TelemetryState::NOT_CONNECTED;
      // Reset runtime accumulator so it restarts from zero on reconnect.
      // Keep sEscLastTime10ms at its last real value so the stale-data guard
      // continues to reject the unchanged res->time_10ms from the SINE library.
      sEscAccumulatedRuntimeMs = 0;
      sEscFirstUpdate = true;
    }
  } else {
    if (escTelemetryData.escState != TelemetryState::CONNECTED) {
      // Log state change only if it actually changed
      USBSerial.printf("ESC State: %d -> CONNECTED\n", static_cast<int>(escTelemetryData.escState));
      escTelemetryData.escState = TelemetryState::CONNECTED;
      // Auto-request hardware info on first connection if not already populated
      if (escTelemetryData.hardware_id == 0) {
        USBSerial.println("ESC: Auto-requesting hardware info");
        esc.getHardwareInfo();
      }
    }
  }

  // Process any pending hardware info request (set from BLE command or retry)
  if (s_hwInfoRequested) {
    s_hwInfoRequested = false;
    USBSerial.println("ESC: Sending GetHwInfo (app-requested)");
    esc.getHardwareInfo();
  }

  // Populate static hardware info whenever available (comes in once after boot)
  if (model.hasGetHardwareInfoResponse) {
    const sine_esc_GetHwInfoResponse *hw = &model.getHardwareInfoResponse;
    escTelemetryData.hardware_id = hw->hardware_id;
    escTelemetryData.fw_version = hw->app_version;
    escTelemetryData.bootloader_version = hw->bootloader_version;
    memcpy(escTelemetryData.sn_code, hw->sn_code, sizeof(escTelemetryData.sn_code));
  }

  syncEscOutputs();
  adapter.processTxRxOnce();  // Process CAN messages

  // Drive any in-flight ESC config-relay / firmware-relay session. Both run on
  // the throttle task so all CAN traffic stays single-owner. Non-blocking; only
  // one can be active at a time (mutually gated) and only when DISARMED.
  escConfigRelayServiceTick();
  escFlasherRelayServiceTick();
}

/**
 * Request ESC hardware info (HW ID, FW version, bootloader version, serial number).
 * Thread-safe: sets a flag that is consumed by readESCTelemetry() on its next
 * tick, keeping all CAN traffic on the throttle task.
 * Called by the BLE command characteristic handler (0x01 from the app).
 */
void requestEscHardwareInfo() {
  s_hwInfoRequested = true;
}

void requestEscStatusLightMode(EscStatusLightMode mode) {
  sRequestedStatusLightMode = mode;
}

void queueEscMotorBeepArm() {
  sPendingEscTone = PendingEscTone::ARM;
}

void queueEscMotorBeepDisarm() {
  sPendingEscTone = PendingEscTone::DISARM;
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
