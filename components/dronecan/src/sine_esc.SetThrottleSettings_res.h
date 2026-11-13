#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETTHROTTLESETTINGS_RESPONSE_MAX_SIZE 1
#define SINE_ESC_SETTHROTTLESETTINGS_RESPONSE_SIGNATURE (0x1653C41DA4E45824ULL)
#define SINE_ESC_SETTHROTTLESETTINGS_RESPONSE_ID 210

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetThrottleSettings_cxx_iface;
#endif

struct sine_esc_SetThrottleSettingsResponse {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetThrottleSettings_cxx_iface;
#endif
    uint8_t valid_flag;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetThrottleSettingsResponse_encode(struct sine_esc_SetThrottleSettingsResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetThrottleSettingsResponse_decode(const CanardRxTransfer* transfer, struct sine_esc_SetThrottleSettingsResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetThrottleSettingsResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettingsResponse* msg, bool tao);
static inline bool _sine_esc_SetThrottleSettingsResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettingsResponse* msg, bool tao);
void _sine_esc_SetThrottleSettingsResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettingsResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->valid_flag);
    *bit_ofs += 8;
}

/*
 decode sine_esc_SetThrottleSettingsResponse, return true on failure, false on success
*/
bool _sine_esc_SetThrottleSettingsResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetThrottleSettingsResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->valid_flag);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetThrottleSettingsResponse sample_sine_esc_SetThrottleSettingsResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
