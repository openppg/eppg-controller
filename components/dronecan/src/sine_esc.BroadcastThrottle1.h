#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_BROADCASTTHROTTLE1_MAX_SIZE 3
#define SINE_ESC_BROADCASTTHROTTLE1_SIGNATURE (0x5F5282808C0EAB76ULL)
#define SINE_ESC_BROADCASTTHROTTLE1_ID 20010

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_BroadcastThrottle1_cxx_iface;
#endif

struct sine_esc_BroadcastThrottle1 {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_BroadcastThrottle1_cxx_iface;
#endif
    uint8_t esc_id;
    uint16_t pwm_us;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_BroadcastThrottle1_encode(struct sine_esc_BroadcastThrottle1* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_BroadcastThrottle1_decode(const CanardRxTransfer* transfer, struct sine_esc_BroadcastThrottle1* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_BroadcastThrottle1_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_BroadcastThrottle1* msg, bool tao);
static inline bool _sine_esc_BroadcastThrottle1_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_BroadcastThrottle1* msg, bool tao);
void _sine_esc_BroadcastThrottle1_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_BroadcastThrottle1* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->esc_id);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->pwm_us);
    *bit_ofs += 16;
}

/*
 decode sine_esc_BroadcastThrottle1, return true on failure, false on success
*/
bool _sine_esc_BroadcastThrottle1_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_BroadcastThrottle1* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->esc_id);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->pwm_us);
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_BroadcastThrottle1 sample_sine_esc_BroadcastThrottle1_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(sine_esc_BroadcastThrottle1, SINE_ESC_BROADCASTTHROTTLE1_ID, SINE_ESC_BROADCASTTHROTTLE1_SIGNATURE, SINE_ESC_BROADCASTTHROTTLE1_MAX_SIZE);
#endif
#endif
