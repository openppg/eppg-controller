#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETROTATIONSPEEDSETTINGS_REQUEST_MAX_SIZE 4
#define SINE_ESC_SETROTATIONSPEEDSETTINGS_REQUEST_SIGNATURE (0x4461BB2B55B3F410ULL)
#define SINE_ESC_SETROTATIONSPEEDSETTINGS_REQUEST_ID 212

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetRotationSpeedSettings_cxx_iface;
#endif

struct sine_esc_SetRotationSpeedSettingsRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetRotationSpeedSettings_cxx_iface;
#endif
    int32_t speed;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetRotationSpeedSettingsRequest_encode(struct sine_esc_SetRotationSpeedSettingsRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetRotationSpeedSettingsRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SetRotationSpeedSettingsRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetRotationSpeedSettingsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetRotationSpeedSettingsRequest* msg, bool tao);
static inline bool _sine_esc_SetRotationSpeedSettingsRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetRotationSpeedSettingsRequest* msg, bool tao);
void _sine_esc_SetRotationSpeedSettingsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetRotationSpeedSettingsRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->speed);
    *bit_ofs += 32;
}

/*
 decode sine_esc_SetRotationSpeedSettingsRequest, return true on failure, false on success
*/
bool _sine_esc_SetRotationSpeedSettingsRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetRotationSpeedSettingsRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->speed);
    *bit_ofs += 32;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetRotationSpeedSettingsRequest sample_sine_esc_SetRotationSpeedSettingsRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
