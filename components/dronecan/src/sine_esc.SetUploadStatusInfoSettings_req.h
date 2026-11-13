#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETUPLOADSTATUSINFOSETTINGS_REQUEST_MAX_SIZE 2
#define SINE_ESC_SETUPLOADSTATUSINFOSETTINGS_REQUEST_SIGNATURE (0x1A1EAE8EF6FFA44ULL)
#define SINE_ESC_SETUPLOADSTATUSINFOSETTINGS_REQUEST_ID 230

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetUploadStatusInfoSettings_cxx_iface;
#endif

struct sine_esc_SetUploadStatusInfoSettingsRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetUploadStatusInfoSettings_cxx_iface;
#endif
    uint16_t upload_hz;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetUploadStatusInfoSettingsRequest_encode(struct sine_esc_SetUploadStatusInfoSettingsRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetUploadStatusInfoSettingsRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SetUploadStatusInfoSettingsRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetUploadStatusInfoSettingsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetUploadStatusInfoSettingsRequest* msg, bool tao);
static inline bool _sine_esc_SetUploadStatusInfoSettingsRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetUploadStatusInfoSettingsRequest* msg, bool tao);
void _sine_esc_SetUploadStatusInfoSettingsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetUploadStatusInfoSettingsRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->upload_hz);
    *bit_ofs += 16;
}

/*
 decode sine_esc_SetUploadStatusInfoSettingsRequest, return true on failure, false on success
*/
bool _sine_esc_SetUploadStatusInfoSettingsRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetUploadStatusInfoSettingsRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->upload_hz);
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetUploadStatusInfoSettingsRequest sample_sine_esc_SetUploadStatusInfoSettingsRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
