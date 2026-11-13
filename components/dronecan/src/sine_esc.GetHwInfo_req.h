#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_GETHWINFO_REQUEST_MAX_SIZE 0
#define SINE_ESC_GETHWINFO_REQUEST_SIGNATURE (0x65DC98CD51449263ULL)
#define SINE_ESC_GETHWINFO_REQUEST_ID 209

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_GetHwInfo_cxx_iface;
#endif

struct sine_esc_GetHwInfoRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_GetHwInfo_cxx_iface;
#endif
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_GetHwInfoRequest_encode(struct sine_esc_GetHwInfoRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_GetHwInfoRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_GetHwInfoRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_GetHwInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoRequest* msg, bool tao);
static inline bool _sine_esc_GetHwInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoRequest* msg, bool tao);
void _sine_esc_GetHwInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode sine_esc_GetHwInfoRequest, return true on failure, false on success
*/
bool _sine_esc_GetHwInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_GetHwInfoRequest sample_sine_esc_GetHwInfoRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
