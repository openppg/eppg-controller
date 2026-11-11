#include "sp140/esc_errors.h"

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
