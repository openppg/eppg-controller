#ifndef POWERPACK_QC_TEST_H_
#define POWERPACK_QC_TEST_H_

#include <stdint.h>

enum class QcResult : uint8_t {
    NOT_RUN,
    PASS,
    FAIL,
    WARN
};

struct QcCheck {
    const char* name;
    float value;
    const char* unit;
    QcResult result;
};

// Snapshot of telemetry at a single moment
struct TelemetrySnapshot {
    int32_t rpm;
    float phaseCurrent;   // A (from 0.1A raw)
    float busCurrent;     // A
    float busVoltage;     // V
    float mosTemp;        // C
    float capTemp;        // C
    float mcuTemp;        // C
    float motorTemp;      // C
    uint16_t vModulation; // modulation ratio
    uint16_t runErr;
    uint16_t selfChkErr;
    uint16_t commPwm;     // 0.1us
};

// Comprehensive test results
struct QcTestResults {
    bool running;
    bool complete;
    QcResult overall;

    // --- Phase 1: Static / idle checks ---
    QcCheck hwId;
    QcCheck fwVersion;
    QcCheck selfCheck;
    QcCheck busVoltageIdle;
    QcCheck mosTempIdle;
    QcCheck capTempIdle;
    QcCheck mcuTempIdle;
    QcCheck motorTempIdle;

    // --- Phase 2: Motor spin-up test ---
    QcCheck motorDirection;     // RPM positive = correct direction
    QcCheck reachesTargetRpm;   // Hit >= 90% of max RPM
    QcCheck peakPhaseCurrent;   // Max A recorded during ramp
    QcCheck peakBusCurrent;     // Max bus A during ramp
    QcCheck voltageSag;         // Idle V - loaded V
    QcCheck mosTempRise;        // Temp delta from idle to peak
    QcCheck capTempRise;        // Temp delta
    QcCheck motorTempRise;      // Temp delta
    QcCheck noSpinErrors;       // No critical errors during motor run

    // Raw peak tracking
    float peakPhaseA;
    float peakBusA;
    int32_t peakRpm;
    float minBusV;             // Lowest voltage under load
    float idleBusV;            // Voltage at idle before spin

    // Snapshots at key points
    TelemetrySnapshot snapIdle;
    TelemetrySnapshot snapPeak;
    TelemetrySnapshot snapAfter;

    // HW info
    uint16_t rawHwId;
    uint16_t rawFwVersion;
    uint16_t rawBootloaderVer;
    uint8_t  serialNumber[16];
};

// Initialize QC test module
void qcTestInit();

// Start the full QC test (idle checks + motor spin + evaluation)
void qcTestStart();

// Drive the test. Returns true while running.
bool qcTestTick();

// Get results
const QcTestResults& qcTestGetResults();

// Get a one-line status string for the display during the test
const char* qcTestStatusLine();

// Get progress 0-100 during motor test
uint8_t qcTestProgress();

#endif // POWERPACK_QC_TEST_H_
