#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_GETBROADCASTADDRESSINGSTATUS_REQUEST_MAX_SIZE 0
#define SINE_ESC_GETBROADCASTADDRESSINGSTATUS_REQUEST_SIGNATURE (0xDEDEF673E51F542EULL)
#define SINE_ESC_GETBROADCASTADDRESSINGSTATUS_REQUEST_ID 207

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_GetBroadcastAddressingStatus_cxx_iface;
#endif

struct sine_esc_GetBroadcastAddressingStatusRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_GetBroadcastAddressingStatus_cxx_iface;
#endif
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_GetBroadcastAddressingStatusRequest_encode(struct sine_esc_GetBroadcastAddressingStatusRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_GetBroadcastAddressingStatusRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_GetBroadcastAddressingStatusRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_GetBroadcastAddressingStatusRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetBroadcastAddressingStatusRequest* msg, bool tao);
static inline bool _sine_esc_GetBroadcastAddressingStatusRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetBroadcastAddressingStatusRequest* msg, bool tao);
void _sine_esc_GetBroadcastAddressingStatusRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetBroadcastAddressingStatusRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode sine_esc_GetBroadcastAddressingStatusRequest, return true on failure, false on success
*/
bool _sine_esc_GetBroadcastAddressingStatusRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetBroadcastAddressingStatusRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_GetBroadcastAddressingStatusRequest sample_sine_esc_GetBroadcastAddressingStatusRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
