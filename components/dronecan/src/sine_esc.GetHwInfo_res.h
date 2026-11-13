#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_GETHWINFO_RESPONSE_MAX_SIZE 26
#define SINE_ESC_GETHWINFO_RESPONSE_SIGNATURE (0x65DC98CD51449263ULL)
#define SINE_ESC_GETHWINFO_RESPONSE_ID 209

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_GetHwInfo_cxx_iface;
#endif

struct sine_esc_GetHwInfoResponse {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_GetHwInfo_cxx_iface;
#endif
    uint16_t hardware_id;
    uint16_t bootloader_version;
    uint16_t app_version;
    uint16_t resv1;
    uint16_t resv2;
    uint8_t sn_code[16];
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_GetHwInfoResponse_encode(struct sine_esc_GetHwInfoResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_GetHwInfoResponse_decode(const CanardRxTransfer* transfer, struct sine_esc_GetHwInfoResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_GetHwInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoResponse* msg, bool tao);
static inline bool _sine_esc_GetHwInfoResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoResponse* msg, bool tao);
void _sine_esc_GetHwInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->hardware_id);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->bootloader_version);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->app_version);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->resv1);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->resv2);
    *bit_ofs += 16;
    for (size_t i=0; i < 16; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sn_code[i]);
        *bit_ofs += 8;
    }
}

/*
 decode sine_esc_GetHwInfoResponse, return true on failure, false on success
*/
bool _sine_esc_GetHwInfoResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetHwInfoResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->hardware_id);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->bootloader_version);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->app_version);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->resv1);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->resv2);
    *bit_ofs += 16;

    for (size_t i=0; i < 16; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sn_code[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_GetHwInfoResponse sample_sine_esc_GetHwInfoResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
