#ifndef INC_SP140_ESC_ERRORS_H_
#define INC_SP140_ESC_ERRORS_H_

#include <Arduino.h>

// Decoding of the ESC running and self-check error bitmasks. Kept free of CAN
// and hardware dependencies so it can be unit tested natively.

// ESC Error Decoding Functions
String decodeRunningError(uint16_t errorCode);
String decodeSelfCheckError(uint16_t errorCode);
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

#endif  // INC_SP140_ESC_ERRORS_H_
