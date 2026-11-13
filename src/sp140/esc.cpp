#include "platform/runtime.h"
#include "sp140/esc.h"
#include "sp140/globals.h"
#include "sp140/esc_config_relay.h"
#include "sp140/esc_flasher_relay.h"
#include "platform/ring_buffer.h"

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "driver/twai.h"

#define ESC_RX_PIN 3  // CAN RX pin to transceiver
#define ESC_TX_PIN 2  // CAN TX pin to transceiver
#define LOCAL_NODE_ID 0x01  // The ID on the network of this device

#define TELEMETRY_TIMEOUT_MS 50  // Threshold to determine stale ESC telemetry in ms
#define TWAI_HEALTH_POLL_MS 250  // How often to sample driver status/alerts

static CanardAdapter adapter;
static uint8_t memory_pool[1024] __attribute__((aligned(8)));
static SineEsc esc(adapter);

// Expose the single shared adapter to the ESC config relay module.
CanardAdapter& escAdapter() { return adapter; }
static unsigned long lastSuccessfulCommTimeMs = 0;  // Store platform::millis32() time of last successful ESC comm
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
  const unsigned long now = platform::millis32();
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

static TwaiBusHealth sTwaiHealth = {};
// True from the moment we see BUS_OFF until the post-recovery restart succeeds.
// Gates the STOPPED->start transition so we only auto-start a driver that WE
// took down for recovery, never one something else stopped on purpose.
static bool sTwaiRecoveryPending = false;

const char* twaiBusStateName(uint8_t state) {
  switch (static_cast<twai_state_t>(state)) {
    case TWAI_STATE_STOPPED: return "STOPPED";
    case TWAI_STATE_RUNNING: return "RUNNING";
    case TWAI_STATE_BUS_OFF: return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default: return "UNKNOWN";
  }
}

TwaiBusHealth getTwaiBusHealth() {
  return sTwaiHealth;
}

/**
 * Poll TWAI driver health and auto-recover from bus-off.
 *
 * A severed or shorted CAN pair while we're transmitting drives the TX error
 * counter into error-passive and potentially bus-off. In bus-off the driver
 * stops participating entirely — without recovery, fixing the cable would NOT
 * restore comms until a reboot. Recovery sequence: initiate while BUS_OFF,
 * the driver lands in STOPPED once it sees the required idle bus, then we
 * restart it.
 */
static void serviceTwaiBusHealth() {
  const unsigned long now = platform::millis32();
  static unsigned long lastPollMs = 0;
  if (now - lastPollMs < TWAI_HEALTH_POLL_MS) { return; }  // NOLINT(whitespace/newline)
  lastPollMs = now;

  // Drain latched alerts (non-blocking) so error-state edges hit the log
  // even if the state recovers between status polls.
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts != 0) {
    if (alerts & TWAI_ALERT_ERR_PASS) {
      platform::console.line("[TWAI] Entered error-passive (CAN wiring fault or unacked TX)");
    }
    if (alerts & TWAI_ALERT_BUS_OFF) {
      sTwaiHealth.bus_off_count++;
      platform::console.line("[TWAI] BUS-OFF");
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
      platform::console.line("[TWAI] Bus recovered");
    }
  }

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) { return; }  // NOLINT(whitespace/newline)

  const uint8_t prevState = sTwaiHealth.state;
  sTwaiHealth.state = static_cast<uint8_t>(status.state);
  sTwaiHealth.bus_error_count = status.bus_error_count;
  sTwaiHealth.tx_failed_count = status.tx_failed_count;
  sTwaiHealth.rx_missed_count = status.rx_missed_count;
  sTwaiHealth.arb_lost_count = status.arb_lost_count;
  sTwaiHealth.tx_error_counter = status.tx_error_counter;
  sTwaiHealth.rx_error_counter = status.rx_error_counter;

  if (prevState != sTwaiHealth.state) {
    platform::console.format("[TWAI] State %s -> %s (tec=%lu rec=%lu busErr=%lu)\n",
                     twaiBusStateName(prevState), twaiBusStateName(sTwaiHealth.state),
                     (unsigned long)status.tx_error_counter,
                     (unsigned long)status.rx_error_counter,
                     (unsigned long)status.bus_error_count);
  }

  switch (status.state) {
    case TWAI_STATE_BUS_OFF:
      if (!sTwaiRecoveryPending) {
        sTwaiRecoveryPending = true;
        platform::console.line("[TWAI] Initiating bus-off recovery");
        twai_initiate_recovery();
      }
      break;
    case TWAI_STATE_RECOVERING:
      break;  // Recovery in progress — wait for STOPPED
    case TWAI_STATE_STOPPED:
      if (sTwaiRecoveryPending && twai_start() == ESP_OK) {
        sTwaiRecoveryPending = false;
        sTwaiHealth.recovery_count++;
        platform::console.line("[TWAI] Driver restarted after bus-off recovery");
      }
      break;
    case TWAI_STATE_RUNNING:
    default:
      sTwaiRecoveryPending = false;
      break;
  }
}

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
    platform::console.line("ESC TWAI Initialization failed. ESC will not function.");
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
  // libcanard has no instance until esc.begin succeeds.
  if (!escTwaiInitialized) return;
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
        // Store invalid motor temp as NaN. Downstream consumers can skip on std::isnan().
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
      lastSuccessfulCommTimeMs = platform::millis32();
    }  // else: Timestamp hasn't changed, treat as stale data, don't update local timer or telemetry

  } else {
      // If we are connected but don't have a response, perhaps mark as stale or log?
      // For now, do nothing
  }

  // Update connection state based on time since last successful communication
  unsigned long currentTimeMs = platform::millis32();
  if (lastSuccessfulCommTimeMs == 0 || (currentTimeMs - lastSuccessfulCommTimeMs) > TELEMETRY_TIMEOUT_MS) {
    if (escTelemetryData.escState != TelemetryState::NOT_CONNECTED) {
      // Log state change only if it actually changed
      platform::console.format("ESC State: %d -> NOT_CONNECTED (Timeout)\n", static_cast<int>(escTelemetryData.escState));
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
      platform::console.format("ESC State: %d -> CONNECTED\n", static_cast<int>(escTelemetryData.escState));
      escTelemetryData.escState = TelemetryState::CONNECTED;
      // Auto-request hardware info on first connection if not already populated
      if (escTelemetryData.hardware_id == 0) {
        platform::console.line("ESC: Auto-requesting hardware info");
        esc.getHardwareInfo();
      }
    }
  }

  // Process any pending hardware info request (set from BLE command or retry)
  if (s_hwInfoRequested) {
    s_hwInfoRequested = false;
    platform::console.line("ESC: Sending GetHwInfo (app-requested)");
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
  serviceTwaiBusHealth();     // Track bus errors; auto-recover from bus-off

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
    platform::console.format("TWAI driver already installed. State: %d\n", status_info.state);
    // If it's stopped, maybe we need to start it? For now, assume it's okay.
    // if (status_info.state == TWAI_STATE_STOPPED) { twai_start(); }
    return true;  // Already initialized
  } else if (status_result != ESP_ERR_INVALID_STATE &&
             status_result != ESP_ERR_INVALID_ARG) {
    // An error other than "not installed" occurred
    platform::console.format("Error checking TWAI status: %s\n", esp_err_to_name(status_result));
    return false;  // Don't proceed if status check failed unexpectedly
  }
  // IDF 5.5 reports INVALID_ARG for the null legacy driver handle before install.
  // Both not-installed results must proceed with installation.

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
                                      (gpio_num_t)ESC_TX_PIN,
                                      (gpio_num_t)ESC_RX_PIN,
                                      TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t install_err = twai_driver_install(&g_config, &t_config, &f_config);
  if (install_err == ESP_OK) {
      platform::console.line("TWAI Driver installed");
  } else {
      platform::console.format("Failed to install TWAI driver: %s\n", esp_err_to_name(install_err));
      return false;
  }

  esp_err_t start_err = twai_start();
  if (start_err == ESP_OK) {
      platform::console.line("TWAI Driver started");
  } else {
      platform::console.format("Failed to start TWAI driver: %s\n", esp_err_to_name(start_err));
      // Attempt to uninstall if start failed
      twai_driver_uninstall();
      return false;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA
                              | TWAI_ALERT_ERR_PASS
                              | TWAI_ALERT_BUS_ERROR
                              | TWAI_ALERT_RX_QUEUE_FULL
                              | TWAI_ALERT_BUS_OFF
                              | TWAI_ALERT_BUS_RECOVERED;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
      platform::console.line("CAN Alerts reconfigured");
  } else {
      platform::console.line("Failed to reconfigure CAN alerts");
      // Consider this non-fatal for now? Or return false?
  }

  return true;
}

/**
 * Debug function to dump ESC throttle response data to serial
 * @param res Pointer to the throttle response structure from ESC
 */
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res) {
  platform::console.line("Got SetThrottleSettings2 response");

  platform::console.text("\trecv_pwm: ");
  platform::console.line(res->recv_pwm);

  platform::console.text("\tcomm_pwm: ");
  platform::console.line(res->comm_pwm);

  platform::console.text("\tspeed: ");
  platform::console.line(res->speed);

  platform::console.text("\tcurrent: ");
  platform::console.line(res->current);

  platform::console.text("\tbus_current: ");
  platform::console.line(res->bus_current);

  platform::console.text("\tvoltage: ");
  platform::console.line(res->voltage);

  platform::console.text("\tv_modulation: ");
  platform::console.line(res->v_modulation);

  platform::console.text("\tmos_temp: ");
  platform::console.line(res->mos_temp);

  platform::console.text("\tcap_temp: ");
  platform::console.line(res->cap_temp);

  platform::console.text("\tmcu_temp: ");
  platform::console.line(res->mcu_temp);

  platform::console.text("\trunning_error: ");
  platform::console.text(res->running_error);
  platform::console.text(" (");
  platform::console.text(decodeRunningError(res->running_error));
  platform::console.line(")");

  platform::console.text("\tselfcheck_error: ");
  platform::console.text(res->selfcheck_error);
  platform::console.text(" (");
  platform::console.text(decodeSelfCheckError(res->selfcheck_error));
  platform::console.line(")");

  platform::console.text("\tmotor_temp: ");
  platform::console.line(res->motor_temp);

  platform::console.text("\ttime_10ms: ");
  platform::console.line(res->time_10ms);
}

/**
 * Debug function to dump all available ESC messages to serial
 * Displays hardware info, throttle response, and rotation speed settings if available
 */
void dumpESCMessages(void) {
  const SineEscModel &model = esc.getModel();

  if (model.hasGetHardwareInfoResponse) {
    platform::console.line("Got HwInfo response");

    const sine_esc_GetHwInfoResponse *b = &model.getHardwareInfoResponse;
    platform::console.text("\thardware_id: ");
    platform::console.format("%lX\r\n", static_cast<unsigned long>(b->hardware_id));
    platform::console.text("\tbootloader_version: ");
    platform::console.format("%lX\r\n", static_cast<unsigned long>(b->bootloader_version));
    platform::console.text("\tapp_version: ");
    platform::console.format("%lX\r\n", static_cast<unsigned long>(b->app_version));
  }

  if (model.hasSetThrottleSettings2Response) {
    dumpThrottleResponse(&model.setThrottleSettings2Response);
  }

  if (model.hasSetRotationSpeedSettingsResponse) {
    platform::console.line("Got SetRotationSpeedSettings response");
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
 * Decode running error bitmask into human-readable string
 * @param errorCode 16-bit running error code from ESC
 * @return std::string containing decoded error messages
 */
std::string decodeRunningError(uint16_t errorCode) {
  if (errorCode == 0) {
    return "no_errors";
  }

  std::string result = "";
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
 * @return std::string containing decoded error messages
 */
std::string decodeSelfCheckError(uint16_t errorCode) {
  if (errorCode == 0) {
    return "no_errors";
  }

  std::string result = "";
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
  // Bits 0,1,2,6,7 are High priority and relevant for CAN communication
  const uint16_t criticalBits = 0x00C7;  // Bits 0,1,2,6,7 (High priority)
  // Excluded bits:
  // - Bit 3 (pwm_throttle_lost): Not relevant for CAN communication
  // - Bit 4 (no_load): Low priority
  // - Bit 5 (throttle_saturation): Middle priority
  // - Bit 8 (comm_throttle_loss): Low priority, expected with CAN

  return (errorCode & criticalBits) != 0;
}

/**
 * Check if there are any warning-level running errors
 * Warning errors are middle-priority faults that should be monitored but aren't critical
 * @param errorCode 16-bit running error code from ESC
 * @return true if any warning error bits are set
 */
bool hasWarningRunningError(uint16_t errorCode) {
  // Bit 5 (throttle_saturation) is Middle priority - treat as warning
  const uint16_t warningBits = 0x0020;  // Bit 5 only

  return (errorCode & warningBits) != 0;
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

// Individual error bit checkers for specific monitoring
bool hasOverCurrentError(uint16_t errorCode) {
  return (errorCode & 0x0001) != 0;  // Bit 0
}

bool hasLockedRotorError(uint16_t errorCode) {
  return (errorCode & 0x0002) != 0;  // Bit 1
}

bool hasOverTempError(uint16_t errorCode) {
  return (errorCode & 0x0004) != 0;  // Bit 2
}

bool hasOverVoltError(uint16_t errorCode) {
  return (errorCode & 0x0040) != 0;  // Bit 6
}

bool hasVoltagDropError(uint16_t errorCode) {
  return (errorCode & 0x0080) != 0;  // Bit 7
}

bool hasThrottleSatWarning(uint16_t errorCode) {
  return (errorCode & 0x0020) != 0;  // Bit 5
}

// Individual self-check error bit checkers
bool hasMotorCurrentOutError(uint16_t errorCode) {
  return (errorCode & 0x0001) != 0;  // Bit 0
}

bool hasTotalCurrentOutError(uint16_t errorCode) {
  return (errorCode & 0x0002) != 0;  // Bit 1
}

bool hasMotorVoltageOutError(uint16_t errorCode) {
  return (errorCode & 0x0004) != 0;  // Bit 2
}

bool hasCapNTCError(uint16_t errorCode) {
  return (errorCode & 0x0008) != 0;  // Bit 3
}

bool hasMosNTCError(uint16_t errorCode) {
  return (errorCode & 0x0010) != 0;  // Bit 4
}

bool hasBusVoltRangeError(uint16_t errorCode) {
  return (errorCode & 0x0020) != 0;  // Bit 5
}

bool hasBusVoltSampleError(uint16_t errorCode) {
  return (errorCode & 0x0040) != 0;  // Bit 6
}

bool hasMotorZLowError(uint16_t errorCode) {
  return (errorCode & 0x0080) != 0;  // Bit 7
}

bool hasMotorZHighError(uint16_t errorCode) {
  return (errorCode & 0x0100) != 0;  // Bit 8
}

bool hasMotorVDet1Error(uint16_t errorCode) {
  return (errorCode & 0x0200) != 0;  // Bit 9
}

bool hasMotorVDet2Error(uint16_t errorCode) {
  return (errorCode & 0x0400) != 0;  // Bit 10
}

bool hasMotorIDet2Error(uint16_t errorCode) {
  return (errorCode & 0x0800) != 0;  // Bit 11
}

bool hasSwHwIncompatError(uint16_t errorCode) {
  return (errorCode & 0x2000) != 0;  // Bit 13
}

bool hasBootloaderBadError(uint16_t errorCode) {
  return (errorCode & 0x4000) != 0;  // Bit 14
}
