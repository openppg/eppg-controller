#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SENDFWDATA_REQUEST_MAX_SIZE 258
#define SINE_ESC_SENDFWDATA_REQUEST_SIGNATURE (0xF8D50AFD350A8FBBULL)
#define SINE_ESC_SENDFWDATA_REQUEST_ID 202

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SendFwData_cxx_iface;
#endif

struct sine_esc_SendFwDataRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SendFwData_cxx_iface;
#endif
    uint16_t index;
    uint8_t data[256];
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SendFwDataRequest_encode(struct sine_esc_SendFwDataRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SendFwDataRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SendFwDataRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SendFwDataRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SendFwDataRequest* msg, bool tao);
static inline bool _sine_esc_SendFwDataRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SendFwDataRequest* msg, bool tao);
void _sine_esc_SendFwDataRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SendFwDataRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->index);
    *bit_ofs += 16;
    for (size_t i=0; i < 256; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode sine_esc_SendFwDataRequest, return true on failure, false on success
*/
bool _sine_esc_SendFwDataRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SendFwDataRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->index);
    *bit_ofs += 16;

    for (size_t i=0; i < 256; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SendFwDataRequest sample_sine_esc_SendFwDataRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
