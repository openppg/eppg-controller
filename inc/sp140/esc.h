#ifndef INC_SP140_ESC_H_
#define INC_SP140_ESC_H_

#include "platform/runtime.h"

// Motor temp validity range (disconnected/invalid readings are represented as NaN)
constexpr float MOTOR_TEMP_VALID_MIN_C = -20.0f;
constexpr float MOTOR_TEMP_VALID_MAX_C = 140.0f;

inline bool isMotorTempValidC(float tempC) {
  return tempC > MOTOR_TEMP_VALID_MIN_C && tempC <= MOTOR_TEMP_VALID_MAX_C;
}
#include "sp140/structs.h"
#include "../../inc/sp140/esp32s3-config.h"
#include <SineEsc.h>
#include <CanardAdapter.h>

enum class EscStatusLightMode : uint8_t {
  OFF = 0,
  DISARMED_NOMINAL,
  DISARMED_WARNING,
  DISARMED_CRITICAL,
  FLIGHT_NOMINAL,
  FLIGHT_WARNING,
  FLIGHT_CRITICAL,
};

void initESC();
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
bool setupTWAI();

// TWAI (CAN) bus health — updated by the ESC telemetry loop.
// Error counters distinguish a wiring/bus fault (TX errors climbing into
// error-passive/bus-off while transmitting into a severed or shorted pair)
// from an ESC that is simply unpowered (bus idle, counters flat).
struct TwaiBusHealth {
  uint32_t bus_error_count;   // cumulative bus errors (driver status)
  uint32_t tx_failed_count;   // frames that exhausted retries
  uint32_t rx_missed_count;   // frames lost to full RX queue
  uint32_t arb_lost_count;    // arbitration losses
  uint32_t tx_error_counter;  // current TEC (>=128 = error-passive)
  uint32_t rx_error_counter;  // current REC
  uint32_t bus_off_count;     // times the controller entered bus-off
  uint32_t recovery_count;    // successful automatic bus-off recoveries
  uint8_t  state;             // last observed twai_state_t
};
TwaiBusHealth getTwaiBusHealth();
const char* twaiBusStateName(uint8_t state);

// Accessor to the single shared CanardAdapter (the controller's sole CAN owner).
// Used by the ESC config relay so its requests go out on the same adapter the
// throttle task drives — keeping all CAN traffic on one task, no mutex needed.
CanardAdapter& escAdapter();

// Request ESC hardware info (HW ID, FW version, bootloader, serial number).
// Thread-safe: sets a flag consumed by readESCTelemetry() on its next tick.
// Also called automatically the first time the ESC connects.
void requestEscHardwareInfo();

// ESC Error Decoding Functions
std::string decodeRunningError(uint16_t errorCode);
std::string decodeSelfCheckError(uint16_t errorCode);
bool hasRunningError(uint16_t errorCode);
bool hasSelfCheckError(uint16_t errorCode);
bool hasCriticalRunningError(uint16_t errorCode);
bool hasWarningRunningError(uint16_t errorCode);
bool hasCriticalSelfCheckError(uint16_t errorCode);

// Individual running error bit checkers
bool hasOverCurrentError(uint16_t errorCode);
bool hasLockedRotorError(uint16_t errorCode);
bool hasOverTempError(uint16_t errorCode);
bool hasOverVoltError(uint16_t errorCode);
bool hasVoltagDropError(uint16_t errorCode);
bool hasThrottleSatWarning(uint16_t errorCode);

// Individual self-check error bit checkers
bool hasMotorCurrentOutError(uint16_t errorCode);
bool hasTotalCurrentOutError(uint16_t errorCode);
bool hasMotorVoltageOutError(uint16_t errorCode);
bool hasCapNTCError(uint16_t errorCode);
bool hasMosNTCError(uint16_t errorCode);
bool hasBusVoltRangeError(uint16_t errorCode);
bool hasBusVoltSampleError(uint16_t errorCode);
bool hasMotorZLowError(uint16_t errorCode);
bool hasMotorZHighError(uint16_t errorCode);
bool hasMotorVDet1Error(uint16_t errorCode);
bool hasMotorVDet2Error(uint16_t errorCode);
bool hasMotorIDet2Error(uint16_t errorCode);
bool hasSwHwIncompatError(uint16_t errorCode);
bool hasBootloaderBadError(uint16_t errorCode);

// ESC LED control
void requestEscStatusLightMode(EscStatusLightMode mode);

// ESC motor beep
void queueEscMotorBeepArm();
void queueEscMotorBeepDisarm();

// for debugging
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res);
void dumpESCMessages();  // dumps all messages to USBSerial

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
