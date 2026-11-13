#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETESCADDRESSING_RESPONSE_MAX_SIZE 2
#define SINE_ESC_SETESCADDRESSING_RESPONSE_SIGNATURE (0x1BDDF796A3CF4723ULL)
#define SINE_ESC_SETESCADDRESSING_RESPONSE_ID 208

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetEscAddressing_cxx_iface;
#endif

struct sine_esc_SetEscAddressingResponse {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetEscAddressing_cxx_iface;
#endif
    uint8_t esc_id;
    uint8_t valid_flag;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetEscAddressingResponse_encode(struct sine_esc_SetEscAddressingResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetEscAddressingResponse_decode(const CanardRxTransfer* transfer, struct sine_esc_SetEscAddressingResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetEscAddressingResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetEscAddressingResponse* msg, bool tao);
static inline bool _sine_esc_SetEscAddressingResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetEscAddressingResponse* msg, bool tao);
void _sine_esc_SetEscAddressingResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetEscAddressingResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->esc_id);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->valid_flag);
    *bit_ofs += 8;
}

/*
 decode sine_esc_SetEscAddressingResponse, return true on failure, false on success
*/
bool _sine_esc_SetEscAddressingResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetEscAddressingResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->esc_id);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->valid_flag);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetEscAddressingResponse sample_sine_esc_SetEscAddressingResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
