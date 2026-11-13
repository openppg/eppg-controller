#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_STARTFWUPGRADE_REQUEST_MAX_SIZE 6
#define SINE_ESC_STARTFWUPGRADE_REQUEST_SIGNATURE (0x9A5F16E0CDD390F7ULL)
#define SINE_ESC_STARTFWUPGRADE_REQUEST_ID 201

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_StartFwUpgrade_cxx_iface;
#endif

struct sine_esc_StartFwUpgradeRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_StartFwUpgrade_cxx_iface;
#endif
    uint16_t hardware_id;
    uint16_t filse_size_kb;
    uint16_t file_crc;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_StartFwUpgradeRequest_encode(struct sine_esc_StartFwUpgradeRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_StartFwUpgradeRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_StartFwUpgradeRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_StartFwUpgradeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_StartFwUpgradeRequest* msg, bool tao);
static inline bool _sine_esc_StartFwUpgradeRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_StartFwUpgradeRequest* msg, bool tao);
void _sine_esc_StartFwUpgradeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_StartFwUpgradeRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->hardware_id);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->filse_size_kb);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->file_crc);
    *bit_ofs += 16;
}

/*
 decode sine_esc_StartFwUpgradeRequest, return true on failure, false on success
*/
bool _sine_esc_StartFwUpgradeRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_StartFwUpgradeRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->hardware_id);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->filse_size_kb);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->file_crc);
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_StartFwUpgradeRequest sample_sine_esc_StartFwUpgradeRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
