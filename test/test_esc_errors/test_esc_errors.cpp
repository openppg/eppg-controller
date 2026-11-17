#include <gtest/gtest.h>
#include "../native_stubs/Arduino.h"

// Include the real implementation under test
#include "../../inc/sp140/esc_errors.h"
#include "../../src/sp140/esc_errors.cpp"

namespace {

// Bit position -> name, as reported by the ESC running error word
const char* const kRunningNames[16] = {
    "over_current_protect", "locked_rotor_protect", "over_temp_protect",
    "pwm_throttle_lost", "no_load", "throttle_saturation",
    "over_volt_protect", "voltage_drop", "comm_throttle_loss",
    "undef_9", "undef_10", "undef_11", "undef_12", "undef_13", "undef_14",
    "undef_15"};

// Bit position -> name, as reported by the ESC self-check error word
const char* const kSelfCheckNames[16] = {
    "motor_i_out_bad", "total_i_out_bad", "motor_v_out_bad", "cap_ntc_bad",
    "mos_ntc_bad", "bus_v_range", "bus_v_sample_bad", "motor_z_too_low",
    "motor_z_too_high", "motor_v_det1_bad", "motor_v_det2_bad",
    "motor_i_det2_bad", "undef_12", "sw_hw_incompatible",
    "bootloader_unsupported", "undef_15"};

struct BitChecker {
    const char* name;
    bool (*check)(uint16_t);
    int bit;
};

// Each checker must respond to its own bit and to no other
void expectSingleBitCheckers(const BitChecker* checkers, size_t count) {
    for (size_t c = 0; c < count; c++) {
        EXPECT_FALSE(checkers[c].check(0)) << checkers[c].name;
        for (int bit = 0; bit < 16; bit++) {
            EXPECT_EQ(checkers[c].check((uint16_t)(1u << bit)), bit == checkers[c].bit)
                << checkers[c].name << " with bit " << bit;
        }
    }
}

}  // namespace

TEST(ESC_Errors, NoErrorsDecodesToNoErrors) {
    EXPECT_STREQ(decodeRunningError(0).c_str(), "no_errors");
    EXPECT_STREQ(decodeSelfCheckError(0).c_str(), "no_errors");
}

TEST(ESC_Errors, RunningErrorBitNames) {
    for (int bit = 0; bit < 16; bit++) {
        EXPECT_STREQ(decodeRunningError((uint16_t)(1u << bit)).c_str(), kRunningNames[bit])
            << "bit " << bit;
    }
}

TEST(ESC_Errors, SelfCheckErrorBitNames) {
    for (int bit = 0; bit < 16; bit++) {
        EXPECT_STREQ(decodeSelfCheckError((uint16_t)(1u << bit)).c_str(), kSelfCheckNames[bit])
            << "bit " << bit;
    }
}

TEST(ESC_Errors, MultipleErrorsJoinedInBitOrder) {
    // Bits 0, 2 and 6
    EXPECT_STREQ(decodeRunningError(0x0045).c_str(),
                 "over_current_protect, over_temp_protect, over_volt_protect");
    // Bits 3 and 14
    EXPECT_STREQ(decodeSelfCheckError(0x4008).c_str(),
                 "cap_ntc_bad, bootloader_unsupported");
}

TEST(ESC_Errors, AnyBitCountsAsAnError) {
    EXPECT_FALSE(hasRunningError(0));
    EXPECT_FALSE(hasSelfCheckError(0));
    for (int bit = 0; bit < 16; bit++) {
        EXPECT_TRUE(hasRunningError((uint16_t)(1u << bit))) << "bit " << bit;
        EXPECT_TRUE(hasSelfCheckError((uint16_t)(1u << bit))) << "bit " << bit;
    }
}

TEST(ESC_Errors, CriticalRunningErrorBits) {
    // Over current, locked rotor, over temp, over voltage and voltage drop
    const uint16_t criticalBits = 0x00C7;

    EXPECT_FALSE(hasCriticalRunningError(0));
    for (int bit = 0; bit < 16; bit++) {
        const uint16_t code = (uint16_t)(1u << bit);
        EXPECT_EQ(hasCriticalRunningError(code), (code & criticalBits) != 0) << "bit " << bit;
    }

    // Everything that is not critical, all at once, must stay non-critical
    EXPECT_FALSE(hasCriticalRunningError((uint16_t)~criticalBits));
    // A critical bit is still seen alongside non-critical ones
    const uint16_t nonCritical = 0x0128;  // comm loss + saturation + pwm lost
    EXPECT_FALSE(hasCriticalRunningError(nonCritical));
    EXPECT_TRUE(hasCriticalRunningError(nonCritical | 0x0004));  // + over temp
}

TEST(ESC_Errors, WarningRunningErrorIsThrottleSaturationOnly) {
    EXPECT_FALSE(hasWarningRunningError(0));
    for (int bit = 0; bit < 16; bit++) {
        EXPECT_EQ(hasWarningRunningError((uint16_t)(1u << bit)), bit == 5) << "bit " << bit;
    }
    // A warning must not be reported as critical, and vice versa
    EXPECT_FALSE(hasCriticalRunningError(0x0020));
    EXPECT_FALSE(hasWarningRunningError(0x00C7));
}

TEST(ESC_Errors, EverySelfCheckErrorIsCritical) {
    EXPECT_FALSE(hasCriticalSelfCheckError(0));
    for (int bit = 0; bit < 16; bit++) {
        EXPECT_TRUE(hasCriticalSelfCheckError((uint16_t)(1u << bit))) << "bit " << bit;
    }
}

TEST(ESC_Errors, RunningErrorBitCheckers) {
    const BitChecker checkers[] = {
        {"hasOverCurrentError", hasOverCurrentError, 0},
        {"hasLockedRotorError", hasLockedRotorError, 1},
        {"hasOverTempError", hasOverTempError, 2},
        {"hasThrottleSatWarning", hasThrottleSatWarning, 5},
        {"hasOverVoltError", hasOverVoltError, 6},
        {"hasVoltagDropError", hasVoltagDropError, 7},
    };
    expectSingleBitCheckers(checkers, sizeof(checkers) / sizeof(checkers[0]));
}

TEST(ESC_Errors, SelfCheckErrorBitCheckers) {
    const BitChecker checkers[] = {
        {"hasMotorCurrentOutError", hasMotorCurrentOutError, 0},
        {"hasTotalCurrentOutError", hasTotalCurrentOutError, 1},
        {"hasMotorVoltageOutError", hasMotorVoltageOutError, 2},
        {"hasCapNTCError", hasCapNTCError, 3},
        {"hasMosNTCError", hasMosNTCError, 4},
        {"hasBusVoltRangeError", hasBusVoltRangeError, 5},
        {"hasBusVoltSampleError", hasBusVoltSampleError, 6},
        {"hasMotorZLowError", hasMotorZLowError, 7},
        {"hasMotorZHighError", hasMotorZHighError, 8},
        {"hasMotorVDet1Error", hasMotorVDet1Error, 9},
        {"hasMotorVDet2Error", hasMotorVDet2Error, 10},
        {"hasMotorIDet2Error", hasMotorIDet2Error, 11},
        {"hasSwHwIncompatError", hasSwHwIncompatError, 13},
        {"hasBootloaderBadError", hasBootloaderBadError, 14},
    };
    expectSingleBitCheckers(checkers, sizeof(checkers) / sizeof(checkers[0]));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
