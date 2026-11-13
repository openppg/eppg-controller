#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_MAX_SIZE 33
#define SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_SIGNATURE (0x81B48C174CDA4C7AULL)
#define SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_ID 213

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetThrottleSettings2_cxx_iface;
#endif

struct sine_esc_SetThrottleSettings2Response {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetThrottleSettings2_cxx_iface;
#endif
    uint16_t recv_pwm;
    uint16_t comm_pwm;
    int32_t speed;
    int16_t current;
    int16_t bus_current;
    uint16_t voltage;
    uint16_t v_modulation;
    int16_t mos_temp;
    int16_t cap_temp;
    int16_t mcu_temp;
    uint16_t running_error;
    uint16_t selfcheck_error;
    int16_t motor_temp;
    uint16_t time_10ms;
    uint8_t resv[3];
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetThrottleSettings2Response_encode(struct sine_esc_SetThrottleSettings2Response* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetThrottleSettings2Response_decode(const CanardRxTransfer* transfer, struct sine_esc_SetThrottleSettings2Response* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetThrottleSettings2Response_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettings2Response* msg, bool tao);
static inline bool _sine_esc_SetThrottleSettings2Response_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettings2Response* msg, bool tao);
void _sine_esc_SetThrottleSettings2Response_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettings2Response* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->recv_pwm);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->comm_pwm);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->speed);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->current);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->bus_current);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->voltage);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->v_modulation);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->mos_temp);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->cap_temp);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->mcu_temp);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->running_error);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->selfcheck_error);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->motor_temp);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->time_10ms);
    *bit_ofs += 16;
    for (size_t i=0; i < 3; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->resv[i]);
        *bit_ofs += 8;
    }
}

/*
 decode sine_esc_SetThrottleSettings2Response, return true on failure, false on success
*/
bool _sine_esc_SetThrottleSettings2Response_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettings2Response* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->recv_pwm);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->comm_pwm);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->speed);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->current);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->bus_current);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->voltage);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->v_modulation);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->mos_temp);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->cap_temp);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->mcu_temp);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->running_error);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->selfcheck_error);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->motor_temp);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->time_10ms);
    *bit_ofs += 16;

    for (size_t i=0; i < 3; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->resv[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetThrottleSettings2Response sample_sine_esc_SetThrottleSettings2Response_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
