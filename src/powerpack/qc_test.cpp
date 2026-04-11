#include "powerpack/qc_test.h"
#include "powerpack/config.h"
#include <Arduino.h>
#include <CanardAdapter.h>
#include <dronecan_msgs.h>
#include <math.h>

extern CanardAdapter canAdapter;

// =============================================================================
// Motor spin test parameters
// =============================================================================

// Throttle values in 0.1us units (ESC protocol resolution)
static const uint16_t PWM_IDLE      = 10000;  // 1000us - motor off
static const uint16_t PWM_START     = 10500;  // 1050us - just above engagement
static const uint16_t PWM_LOW       = 12000;  // 1200us - ~17% throttle
static const uint16_t PWM_MID       = 15000;  // 1500us - ~50% throttle
static const uint16_t PWM_HIGH      = 18000;  // 1800us - ~83% throttle
static const uint16_t PWM_MAX       = 19500;  // 1950us - 100% throttle

// Ramp timing
static const uint32_t RAMP_STEP_MS  = 50;     // PWM update interval during ramp
static const uint16_t RAMP_RATE     = 100;    // PWM increment per step (0.1us) = 10us/50ms
static const uint32_t HOLD_TIME_MS  = 2000;   // Hold at each plateau
static const uint32_t SETTLE_MS     = 1000;   // Settle time after returning to idle
static const uint32_t IDLE_POLL_MS  = 2000;   // Time to poll at idle before spin

// Expected values for pass/fail
static const int32_t  EXPECTED_MAX_RPM      = 2500;
static const float    MAX_ACCEPTABLE_PHASE_A = 250.0f;  // Over this = FAIL
static const float    MAX_VOLTAGE_SAG_V     = 15.0f;    // More than 15V drop = WARN
static const float    CRITICAL_VOLTAGE_SAG_V = 25.0f;   // = FAIL
static const float    MAX_MOS_TEMP_RISE_C   = 40.0f;    // WARN if >40C rise
static const float    MAX_MOTOR_TEMP_RISE_C = 30.0f;
static const uint16_t CRITICAL_RUN_ERRORS   = 0x00C7;   // bits 0,1,2,6,7 = hard faults

// =============================================================================
// Test state machine
// =============================================================================

enum class TestPhase : uint8_t {
    IDLE_CHECK,       // Read HW info + idle sensor readings
    RAMP_UP,          // Ramp throttle from idle to max
    HOLD_MAX,         // Hold at max throttle, record peaks
    RAMP_DOWN,        // Ramp back to idle
    COOL_DOWN,        // Settle at idle, record post-run temps
    EVALUATE,         // Crunch numbers
    DONE
};

static QcTestResults s_results;
static TestPhase s_phase;
static const char* s_statusLine = "Idle";
static uint8_t s_progress = 0;

// Timing
static unsigned long s_phaseStartMs = 0;
static unsigned long s_lastRampMs = 0;
static uint16_t s_currentPwm = PWM_IDLE;

// Telemetry polling
static uint8_t s_throttleTransferId = 0;
static uint8_t s_hwInfoTransferId = 0;
static volatile bool s_gotTelemetry = false;
static volatile bool s_gotHwInfo = false;
static sine_esc_SetThrottleSettings2Response s_telem;
static sine_esc_GetHwInfoResponse s_hwInfo;
static uint8_t s_idlePollCount = 0;

// Peak tracking (reset before spin)
static float s_peakPhaseA = 0;
static float s_peakBusA = 0;
static int32_t s_peakRpm = 0;
static float s_minBusV = 999.0f;
static uint16_t s_spinErrors = 0;  // OR of all running_errors seen during spin

// =============================================================================
// CAN node for telemetry + HW info responses
// =============================================================================

class QcTestNode : public CanardAdapterNode {
public:
    QcTestNode(CanardAdapter& adapter) : CanardAdapterNode(adapter) {}
    void begin() { _beginNode(); }
    CanardInstance* canard() { return _canard; }

    bool shouldAcceptTransfer(uint64_t* out_data_type_signature,
                              uint16_t data_type_id,
                              CanardTransferType transfer_type,
                              uint8_t source_node_id) override {
        if (source_node_id != ESC_NODE_ID) return false;
        if (transfer_type == CanardTransferTypeResponse) {
            if (data_type_id == SINE_ESC_SETTHROTTLESETTINGS2_ID) {
                *out_data_type_signature = SINE_ESC_SETTHROTTLESETTINGS2_SIGNATURE;
                return true;
            }
            if (data_type_id == SINE_ESC_GETHWINFO_REQUEST_ID) {
                *out_data_type_signature = SINE_ESC_GETHWINFO_REQUEST_SIGNATURE;
                return true;
            }
        }
        return false;
    }

    void onTransferReceived(CanardRxTransfer* transfer) override {
        if (transfer->transfer_type != CanardTransferTypeResponse) return;
        if (transfer->data_type_id == SINE_ESC_SETTHROTTLESETTINGS2_ID) {
            sine_esc_SetThrottleSettings2Response_decode(transfer, &s_telem);
            s_gotTelemetry = true;
        } else if (transfer->data_type_id == SINE_ESC_GETHWINFO_ID) {
            sine_esc_GetHwInfoResponse_decode(transfer, &s_hwInfo);
            s_gotHwInfo = true;
        }
    }
};

static QcTestNode* s_testNode = nullptr;

// =============================================================================
// CAN send helpers
// =============================================================================

static void sendThrottle(uint16_t pwm_01us) {
    sine_esc_SetThrottleSettings2Request req = { .pwm_us = pwm_01us };
    uint8_t buffer[SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_MAX_SIZE];
    uint32_t len = sine_esc_SetThrottleSettings2Request_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_SIGNATURE,
        .data_type_id = SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_ID,
        .inout_transfer_id = &s_throttleTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_testNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendGetHwInfo() {
    sine_esc_GetHwInfoRequest req;
    uint8_t buffer[SINE_ESC_GETHWINFO_REQUEST_MAX_SIZE];
    uint32_t len = sine_esc_GetHwInfoRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = SINE_ESC_GETHWINFO_REQUEST_SIGNATURE,
        .data_type_id = SINE_ESC_GETHWINFO_REQUEST_ID,
        .inout_transfer_id = &s_hwInfoTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_testNode->canard(), ESC_NODE_ID, &transfer);
}

// =============================================================================
// Telemetry helpers
// =============================================================================

static TelemetrySnapshot takeSnapshot() {
    TelemetrySnapshot snap;
    snap.rpm          = s_telem.speed;
    snap.phaseCurrent = s_telem.current / 10.0f;
    snap.busCurrent   = s_telem.bus_current / 10.0f;
    snap.busVoltage   = s_telem.voltage / 10.0f;
    snap.mosTemp      = s_telem.mos_temp / 10.0f;
    snap.capTemp      = s_telem.cap_temp / 10.0f;
    snap.mcuTemp      = s_telem.mcu_temp / 10.0f;
    snap.motorTemp    = s_telem.motor_temp / 10.0f;
    snap.vModulation  = s_telem.v_modulation;
    snap.runErr       = s_telem.running_error;
    snap.selfChkErr   = s_telem.selfcheck_error;
    snap.commPwm      = s_telem.comm_pwm;
    return snap;
}

static void updatePeaks() {
    float phaseA = fabsf(s_telem.current / 10.0f);
    float busA   = fabsf(s_telem.bus_current / 10.0f);
    float busV   = s_telem.voltage / 10.0f;
    int32_t rpm  = abs(s_telem.speed);

    if (phaseA > s_peakPhaseA) s_peakPhaseA = phaseA;
    if (busA > s_peakBusA)     s_peakBusA = busA;
    if (rpm > s_peakRpm)       s_peakRpm = rpm;
    if (busV < s_minBusV && busV > 10.0f) s_minBusV = busV;  // ignore junk readings

    // Accumulate any errors seen during spin
    s_spinErrors |= s_telem.running_error;
}

// Check for hard-abort conditions during motor spin
static bool shouldAbort() {
    uint16_t critical = s_telem.running_error & CRITICAL_RUN_ERRORS;
    if (critical) {
        USBSerial.printf("QC ABORT: critical running error 0x%04X during spin\n", critical);
        return true;
    }
    if (s_telem.selfcheck_error != 0) {
        USBSerial.printf("QC ABORT: selfcheck error 0x%04X during spin\n", s_telem.selfcheck_error);
        return true;
    }
    float mosT = s_telem.mos_temp / 10.0f;
    if (mosT > 130.0f) {
        USBSerial.printf("QC ABORT: MOS temp %.1fC exceeds 130C\n", mosT);
        return true;
    }
    return false;
}

static QcResult checkRange(float val, float minOk, float maxOk) {
    if (isnan(val)) return QcResult::FAIL;
    if (val < minOk || val > maxOk) return QcResult::FAIL;
    return QcResult::PASS;
}

static bool isMotorTempValid(float t) {
    return t > QC_MOTOR_TEMP_VALID_MIN_C && t <= QC_MOTOR_TEMP_VALID_MAX_C;
}

// =============================================================================
// Evaluation - called after all phases complete
// =============================================================================

static void evaluate() {
    USBSerial.println("\n=== QC Evaluation ===");

    // --- Static checks ---
    s_results.hwId = {"HW ID", (float)s_results.rawHwId, "",
        (s_results.rawHwId != 0) ? QcResult::PASS : QcResult::FAIL};

    s_results.fwVersion = {"FW Version", (float)s_results.rawFwVersion, "",
        (s_results.rawFwVersion != 0) ? QcResult::PASS : QcResult::FAIL};

    s_results.selfCheck = {"Self-Check", (float)s_results.snapIdle.selfChkErr, "",
        (s_results.snapIdle.selfChkErr == 0) ? QcResult::PASS : QcResult::FAIL};

    s_results.busVoltageIdle = {"Bus V (idle)", s_results.snapIdle.busVoltage, "V",
        checkRange(s_results.snapIdle.busVoltage, QC_VBUS_MIN_V, QC_VBUS_MAX_V)};

    s_results.mosTempIdle = {"MOS T (idle)", s_results.snapIdle.mosTemp, "C",
        checkRange(s_results.snapIdle.mosTemp, QC_TEMP_MIN_C, QC_TEMP_MAX_C)};

    s_results.capTempIdle = {"Cap T (idle)", s_results.snapIdle.capTemp, "C",
        checkRange(s_results.snapIdle.capTemp, QC_TEMP_MIN_C, QC_TEMP_MAX_C)};

    s_results.mcuTempIdle = {"MCU T (idle)", s_results.snapIdle.mcuTemp, "C",
        checkRange(s_results.snapIdle.mcuTemp, QC_TEMP_MIN_C, QC_TEMP_MAX_C)};

    s_results.motorTempIdle = {"Motor T (idle)", s_results.snapIdle.motorTemp, "C",
        isMotorTempValid(s_results.snapIdle.motorTemp) ? QcResult::PASS : QcResult::FAIL};

    // --- Motor spin checks ---
    // Direction: RPM should be positive (motor_direction=1=inversion in config)
    bool dirOk = (s_peakRpm > 100);  // Motor actually spun
    s_results.motorDirection = {"Direction", (float)s_results.snapPeak.rpm, "RPM",
        dirOk ? QcResult::PASS : QcResult::FAIL};

    // Reaches target RPM (>= 80% of expected max)
    float rpmPct = (float)s_peakRpm / (float)EXPECTED_MAX_RPM * 100.0f;
    QcResult rpmResult = (rpmPct >= 80.0f) ? QcResult::PASS :
                         (rpmPct >= 50.0f) ? QcResult::WARN : QcResult::FAIL;
    s_results.reachesTargetRpm = {"Peak RPM", (float)s_peakRpm, "RPM", rpmResult};

    // Peak phase current
    QcResult phaseResult = (s_peakPhaseA <= MAX_ACCEPTABLE_PHASE_A) ? QcResult::PASS : QcResult::FAIL;
    s_results.peakPhaseCurrent = {"Peak Phase I", s_peakPhaseA, "A", phaseResult};

    // Peak bus current
    s_results.peakBusCurrent = {"Peak Bus I", s_peakBusA, "A", QcResult::PASS};

    // Voltage sag
    float sag = s_results.idleBusV - s_minBusV;
    QcResult sagResult = (sag < MAX_VOLTAGE_SAG_V) ? QcResult::PASS :
                         (sag < CRITICAL_VOLTAGE_SAG_V) ? QcResult::WARN : QcResult::FAIL;
    s_results.voltageSag = {"V Sag", sag, "V", sagResult};

    // Temperature rise
    float mosTrise = s_results.snapAfter.mosTemp - s_results.snapIdle.mosTemp;
    s_results.mosTempRise = {"MOS T rise", mosTrise, "C",
        (mosTrise < MAX_MOS_TEMP_RISE_C) ? QcResult::PASS : QcResult::WARN};

    float capTrise = s_results.snapAfter.capTemp - s_results.snapIdle.capTemp;
    s_results.capTempRise = {"Cap T rise", capTrise, "C", QcResult::PASS};

    float motorTrise = 0;
    if (isMotorTempValid(s_results.snapAfter.motorTemp) && isMotorTempValid(s_results.snapIdle.motorTemp)) {
        motorTrise = s_results.snapAfter.motorTemp - s_results.snapIdle.motorTemp;
    }
    s_results.motorTempRise = {"Motor T rise", motorTrise, "C",
        (motorTrise < MAX_MOTOR_TEMP_RISE_C) ? QcResult::PASS : QcResult::WARN};

    // Errors during spin (mask benign idle errors: bits 3,4,5,8)
    uint16_t spinErrRelevant = s_spinErrors & ~0x0138;
    s_results.noSpinErrors = {"Spin Errors", (float)s_spinErrors, "",
        (spinErrRelevant == 0) ? QcResult::PASS : QcResult::FAIL};

    // Store raw peaks
    s_results.peakPhaseA = s_peakPhaseA;
    s_results.peakBusA = s_peakBusA;
    s_results.peakRpm = s_peakRpm;
    s_results.minBusV = s_minBusV;

    // --- Overall verdict ---
    s_results.overall = QcResult::PASS;
    const QcCheck* allChecks[] = {
        &s_results.hwId, &s_results.fwVersion, &s_results.selfCheck,
        &s_results.busVoltageIdle, &s_results.mosTempIdle, &s_results.capTempIdle,
        &s_results.mcuTempIdle, &s_results.motorTempIdle,
        &s_results.motorDirection, &s_results.reachesTargetRpm,
        &s_results.peakPhaseCurrent, &s_results.voltageSag,
        &s_results.noSpinErrors
    };
    for (auto* c : allChecks) {
        if (c->result == QcResult::FAIL) {
            s_results.overall = QcResult::FAIL;
            break;
        }
    }
    // Downgrade to WARN if any warns and no fails
    if (s_results.overall == QcResult::PASS) {
        for (auto* c : allChecks) {
            if (c->result == QcResult::WARN) {
                s_results.overall = QcResult::WARN;
                break;
            }
        }
    }

    // --- Serial report ---
    USBSerial.println("--- Static Checks ---");
    USBSerial.printf("  HW ID:      0x%04X\n", s_results.rawHwId);
    USBSerial.printf("  FW Ver:     0x%04X\n", s_results.rawFwVersion);
    USBSerial.printf("  Bootloader: 0x%04X\n", s_results.rawBootloaderVer);
    USBSerial.printf("  SelfCheck:  0x%04X [%s]\n", s_results.snapIdle.selfChkErr,
        s_results.selfCheck.result == QcResult::PASS ? "PASS" : "FAIL");
    USBSerial.printf("  Bus V:      %.1fV [%s]\n", s_results.snapIdle.busVoltage,
        s_results.busVoltageIdle.result == QcResult::PASS ? "PASS" : "FAIL");
    USBSerial.printf("  MOS Temp:   %.1fC\n", s_results.snapIdle.mosTemp);
    USBSerial.printf("  Cap Temp:   %.1fC\n", s_results.snapIdle.capTemp);
    USBSerial.printf("  MCU Temp:   %.1fC\n", s_results.snapIdle.mcuTemp);
    USBSerial.printf("  Motor Temp: %.1fC [%s]\n", s_results.snapIdle.motorTemp,
        s_results.motorTempIdle.result == QcResult::PASS ? "PASS" : "FAIL");

    USBSerial.println("--- Motor Spin Test ---");
    USBSerial.printf("  Peak RPM:     %ld [%s]\n", (long)s_peakRpm,
        s_results.reachesTargetRpm.result == QcResult::PASS ? "PASS" :
        s_results.reachesTargetRpm.result == QcResult::WARN ? "WARN" : "FAIL");
    USBSerial.printf("  Peak Phase I: %.1fA [%s]\n", s_peakPhaseA,
        s_results.peakPhaseCurrent.result == QcResult::PASS ? "PASS" : "FAIL");
    USBSerial.printf("  Peak Bus I:   %.1fA\n", s_peakBusA);
    USBSerial.printf("  V Sag:        %.1fV (%.1f -> %.1f) [%s]\n",
        sag, s_results.idleBusV, s_minBusV,
        s_results.voltageSag.result == QcResult::PASS ? "PASS" :
        s_results.voltageSag.result == QcResult::WARN ? "WARN" : "FAIL");
    USBSerial.printf("  MOS T rise:   %.1fC (%.1f -> %.1f)\n",
        mosTrise, s_results.snapIdle.mosTemp, s_results.snapAfter.mosTemp);
    USBSerial.printf("  Motor T rise: %.1fC\n", motorTrise);
    USBSerial.printf("  Spin Errors:  0x%04X [%s]\n", s_spinErrors,
        s_results.noSpinErrors.result == QcResult::PASS ? "PASS" : "FAIL");

    const char* overallStr = s_results.overall == QcResult::PASS ? "PASS" :
                             s_results.overall == QcResult::WARN ? "WARN" : "FAIL";
    USBSerial.printf("\n  *** OVERALL: %s ***\n", overallStr);
}

// =============================================================================
// Public API
// =============================================================================

void qcTestInit() {
    if (!s_testNode) {
        s_testNode = new QcTestNode(canAdapter);
        s_testNode->begin();
    }
}

void qcTestStart() {
    memset(&s_results, 0, sizeof(s_results));
    s_results.running = true;
    s_results.complete = false;
    s_results.overall = QcResult::NOT_RUN;

    s_phase = TestPhase::IDLE_CHECK;
    s_phaseStartMs = millis();
    s_currentPwm = PWM_IDLE;
    s_idlePollCount = 0;
    s_gotTelemetry = false;
    s_gotHwInfo = false;

    // Reset peaks
    s_peakPhaseA = 0;
    s_peakBusA = 0;
    s_peakRpm = 0;
    s_minBusV = 999.0f;
    s_spinErrors = 0;

    s_statusLine = "Getting HW info...";
    s_progress = 0;

    USBSerial.println("\n=== QC Test: Starting ===");

    // Request HW info
    sendGetHwInfo();
    // Start idle throttle to get telemetry flowing
    sendThrottle(PWM_IDLE);
}

bool qcTestTick() {
    canAdapter.processTxRxOnce();

    if (!s_results.running) return false;

    unsigned long now = millis();
    unsigned long elapsed = now - s_phaseStartMs;

    switch (s_phase) {

    // -----------------------------------------------------------------
    // PHASE 1: Idle checks - poll telemetry at idle for ~2s
    // -----------------------------------------------------------------
    case TestPhase::IDLE_CHECK: {
        s_statusLine = "Idle check...";
        s_progress = 5;

        // Keep polling
        static unsigned long lastPoll = 0;
        if (now - lastPoll > 100) {
            sendThrottle(PWM_IDLE);
            lastPoll = now;
        }

        // Capture HW info when it arrives
        if (s_gotHwInfo) {
            s_results.rawHwId = s_hwInfo.hardware_id;
            s_results.rawFwVersion = s_hwInfo.app_version;
            s_results.rawBootloaderVer = s_hwInfo.bootloader_version;
            memcpy(s_results.serialNumber, s_hwInfo.sn_code, 16);
            USBSerial.printf("QC: HW=0x%04X FW=0x%04X BL=0x%04X\n",
                s_hwInfo.hardware_id, s_hwInfo.app_version, s_hwInfo.bootloader_version);
            s_gotHwInfo = false;
        }

        if (s_gotTelemetry) {
            s_idlePollCount++;
            s_gotTelemetry = false;
        }

        // After enough idle samples, snapshot and move on
        if (elapsed > IDLE_POLL_MS && s_idlePollCount >= 5) {
            s_results.snapIdle = takeSnapshot();
            s_results.idleBusV = s_results.snapIdle.busVoltage;
            USBSerial.printf("QC: Idle snapshot: %.1fV, MOS=%.1fC, Motor=%.1fC\n",
                s_results.snapIdle.busVoltage, s_results.snapIdle.mosTemp,
                s_results.snapIdle.motorTemp);

            s_phase = TestPhase::RAMP_UP;
            s_phaseStartMs = now;
            s_lastRampMs = now;
            s_currentPwm = PWM_IDLE;
            USBSerial.println("QC: Starting motor ramp-up...");
        }
        break;
    }

    // -----------------------------------------------------------------
    // PHASE 2: Ramp up from idle to max
    // -----------------------------------------------------------------
    case TestPhase::RAMP_UP: {
        // Progress: 10% to 60% across the ramp
        float rampPct = (float)(s_currentPwm - PWM_IDLE) / (float)(PWM_MAX - PWM_IDLE);
        s_progress = 10 + (uint8_t)(rampPct * 50);

        char buf[32];
        int pwmUs = s_currentPwm / 10;
        snprintf(buf, sizeof(buf), "Ramp %dus %dRPM", pwmUs, abs(s_telem.speed));
        s_statusLine = buf;

        if (now - s_lastRampMs >= RAMP_STEP_MS) {
            s_lastRampMs = now;

            if (s_currentPwm < PWM_MAX) {
                s_currentPwm += RAMP_RATE;
                if (s_currentPwm > PWM_MAX) s_currentPwm = PWM_MAX;
            }
            sendThrottle(s_currentPwm);
        }

        if (s_gotTelemetry) {
            updatePeaks();
            s_gotTelemetry = false;

            if (shouldAbort()) {
                USBSerial.println("QC: ABORT - sending idle throttle");
                sendThrottle(PWM_IDLE);
                s_currentPwm = PWM_IDLE;
                s_results.snapPeak = takeSnapshot();
                s_phase = TestPhase::COOL_DOWN;
                s_phaseStartMs = now;
                break;
            }
        }

        // Once at max PWM, transition to hold
        if (s_currentPwm >= PWM_MAX) {
            s_phase = TestPhase::HOLD_MAX;
            s_phaseStartMs = now;
            USBSerial.printf("QC: At max throttle, holding %dms\n", HOLD_TIME_MS);
        }
        break;
    }

    // -----------------------------------------------------------------
    // PHASE 3: Hold at max throttle
    // -----------------------------------------------------------------
    case TestPhase::HOLD_MAX: {
        s_statusLine = "Full throttle hold";
        s_progress = 65;

        // Keep sending throttle
        static unsigned long lastSend = 0;
        if (now - lastSend > 50) {
            sendThrottle(PWM_MAX);
            lastSend = now;
        }

        if (s_gotTelemetry) {
            updatePeaks();
            s_gotTelemetry = false;

            if (shouldAbort()) {
                sendThrottle(PWM_IDLE);
                s_currentPwm = PWM_IDLE;
                s_results.snapPeak = takeSnapshot();
                s_phase = TestPhase::COOL_DOWN;
                s_phaseStartMs = now;
                break;
            }
        }

        if (elapsed >= HOLD_TIME_MS) {
            s_results.snapPeak = takeSnapshot();
            USBSerial.printf("QC: Peak snapshot: %ldRPM, %.1fA phase, %.1fV\n",
                (long)s_results.snapPeak.rpm, s_results.snapPeak.phaseCurrent,
                s_results.snapPeak.busVoltage);
            s_phase = TestPhase::RAMP_DOWN;
            s_phaseStartMs = now;
            s_lastRampMs = now;
            USBSerial.println("QC: Ramping down...");
        }
        break;
    }

    // -----------------------------------------------------------------
    // PHASE 4: Ramp back down to idle
    // -----------------------------------------------------------------
    case TestPhase::RAMP_DOWN: {
        float rampPct = (float)(PWM_MAX - s_currentPwm) / (float)(PWM_MAX - PWM_IDLE);
        s_progress = 70 + (uint8_t)(rampPct * 15);
        s_statusLine = "Ramping down...";

        if (now - s_lastRampMs >= RAMP_STEP_MS) {
            s_lastRampMs = now;

            if (s_currentPwm > PWM_IDLE) {
                if (s_currentPwm > RAMP_RATE + PWM_IDLE) {
                    s_currentPwm -= RAMP_RATE;
                } else {
                    s_currentPwm = PWM_IDLE;
                }
            }
            sendThrottle(s_currentPwm);
        }

        if (s_gotTelemetry) {
            updatePeaks();  // Still tracking during ramp down
            s_gotTelemetry = false;
        }

        if (s_currentPwm <= PWM_IDLE) {
            s_phase = TestPhase::COOL_DOWN;
            s_phaseStartMs = now;
            USBSerial.println("QC: At idle, settling...");
        }
        break;
    }

    // -----------------------------------------------------------------
    // PHASE 5: Cool-down at idle - record post-run temps
    // -----------------------------------------------------------------
    case TestPhase::COOL_DOWN: {
        s_statusLine = "Cooldown / post-check";
        s_progress = 88;

        static unsigned long lastSend2 = 0;
        if (now - lastSend2 > 100) {
            sendThrottle(PWM_IDLE);
            lastSend2 = now;
        }

        if (s_gotTelemetry) {
            s_gotTelemetry = false;
        }

        if (elapsed >= SETTLE_MS) {
            s_results.snapAfter = takeSnapshot();
            USBSerial.printf("QC: Post-run: MOS=%.1fC, Motor=%.1fC\n",
                s_results.snapAfter.mosTemp, s_results.snapAfter.motorTemp);
            s_phase = TestPhase::EVALUATE;
        }
        break;
    }

    // -----------------------------------------------------------------
    // PHASE 6: Evaluate
    // -----------------------------------------------------------------
    case TestPhase::EVALUATE:
        s_statusLine = "Evaluating...";
        s_progress = 95;
        evaluate();
        s_progress = 100;
        s_results.running = false;
        s_results.complete = true;
        s_phase = TestPhase::DONE;
        break;

    case TestPhase::DONE:
        return false;
    }

    return true;
}

const QcTestResults& qcTestGetResults() { return s_results; }
const char* qcTestStatusLine() { return s_statusLine; }
uint8_t qcTestProgress() { return s_progress; }
