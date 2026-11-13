#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_BROADCASTMECHANICALANGLE_MAX_SIZE 2
#define SINE_ESC_BROADCASTMECHANICALANGLE_SIGNATURE (0xA922986F6F2D9C0EULL)
#define SINE_ESC_BROADCASTMECHANICALANGLE_ID 20021

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_BroadcastMechanicalAngle_cxx_iface;
#endif

struct sine_esc_BroadcastMechanicalAngle {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_BroadcastMechanicalAngle_cxx_iface;
#endif
    int16_t mech_angle;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_BroadcastMechanicalAngle_encode(struct sine_esc_BroadcastMechanicalAngle* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_BroadcastMechanicalAngle_decode(const CanardRxTransfer* transfer, struct sine_esc_BroadcastMechanicalAngle* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_BroadcastMechanicalAngle_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_BroadcastMechanicalAngle* msg, bool tao);
static inline bool _sine_esc_BroadcastMechanicalAngle_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_BroadcastMechanicalAngle* msg, bool tao);
void _sine_esc_BroadcastMechanicalAngle_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_BroadcastMechanicalAngle* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->mech_angle);
    *bit_ofs += 16;
}

/*
 decode sine_esc_BroadcastMechanicalAngle, return true on failure, false on success
*/
bool _sine_esc_BroadcastMechanicalAngle_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_BroadcastMechanicalAngle* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->mech_angle);
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_BroadcastMechanicalAngle sample_sine_esc_BroadcastMechanicalAngle_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(sine_esc_BroadcastMechanicalAngle, SINE_ESC_BROADCASTMECHANICALANGLE_ID, SINE_ESC_BROADCASTMECHANICALANGLE_SIGNATURE, SINE_ESC_BROADCASTMECHANICALANGLE_MAX_SIZE);
#endif
#endif
