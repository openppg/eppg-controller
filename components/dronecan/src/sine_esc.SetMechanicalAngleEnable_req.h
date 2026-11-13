#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETMECHANICALANGLEENABLE_REQUEST_MAX_SIZE 1
#define SINE_ESC_SETMECHANICALANGLEENABLE_REQUEST_SIGNATURE (0x8776979B269EFCFFULL)
#define SINE_ESC_SETMECHANICALANGLEENABLE_REQUEST_ID 215

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetMechanicalAngleEnable_cxx_iface;
#endif

struct sine_esc_SetMechanicalAngleEnableRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetMechanicalAngleEnable_cxx_iface;
#endif
    uint8_t enable_mech_nagle;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetMechanicalAngleEnableRequest_encode(struct sine_esc_SetMechanicalAngleEnableRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetMechanicalAngleEnableRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SetMechanicalAngleEnableRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetMechanicalAngleEnableRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetMechanicalAngleEnableRequest* msg, bool tao);
static inline bool _sine_esc_SetMechanicalAngleEnableRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetMechanicalAngleEnableRequest* msg, bool tao);
void _sine_esc_SetMechanicalAngleEnableRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetMechanicalAngleEnableRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->enable_mech_nagle);
    *bit_ofs += 8;
}

/*
 decode sine_esc_SetMechanicalAngleEnableRequest, return true on failure, false on success
*/
bool _sine_esc_SetMechanicalAngleEnableRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetMechanicalAngleEnableRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->enable_mech_nagle);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetMechanicalAngleEnableRequest sample_sine_esc_SetMechanicalAngleEnableRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
