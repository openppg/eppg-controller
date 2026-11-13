#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_GETBOOTSTATUS_REQUEST_MAX_SIZE 0
#define SINE_ESC_GETBOOTSTATUS_REQUEST_SIGNATURE (0x8BA32EBA566F5713ULL)
#define SINE_ESC_GETBOOTSTATUS_REQUEST_ID 200

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_GetBootStatus_cxx_iface;
#endif

struct sine_esc_GetBootStatusRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_GetBootStatus_cxx_iface;
#endif
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_GetBootStatusRequest_encode(struct sine_esc_GetBootStatusRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_GetBootStatusRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_GetBootStatusRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_GetBootStatusRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetBootStatusRequest* msg, bool tao);
static inline bool _sine_esc_GetBootStatusRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetBootStatusRequest* msg, bool tao);
void _sine_esc_GetBootStatusRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetBootStatusRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode sine_esc_GetBootStatusRequest, return true on failure, false on success
*/
bool _sine_esc_GetBootStatusRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetBootStatusRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_GetBootStatusRequest sample_sine_esc_GetBootStatusRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
