#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETMOTORSOUND_REQUEST_MAX_SIZE 32
#define SINE_ESC_SETMOTORSOUND_REQUEST_SIGNATURE (0x40C65964649CAE65ULL)
#define SINE_ESC_SETMOTORSOUND_REQUEST_ID 214

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetMotorSound_cxx_iface;
#endif

struct sine_esc_SetMotorSoundRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetMotorSound_cxx_iface;
#endif
    uint8_t beep_num;
    struct { uint8_t len; uint8_t data[30]; }beep_data;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetMotorSoundRequest_encode(struct sine_esc_SetMotorSoundRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetMotorSoundRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SetMotorSoundRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetMotorSoundRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetMotorSoundRequest* msg, bool tao);
static inline bool _sine_esc_SetMotorSoundRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetMotorSoundRequest* msg, bool tao);
void _sine_esc_SetMotorSoundRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetMotorSoundRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->beep_num);
    *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t beep_data_len = msg->beep_data.len > 30 ? 30 : msg->beep_data.len;
#pragma GCC diagnostic pop
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 5, &beep_data_len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < beep_data_len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->beep_data.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode sine_esc_SetMotorSoundRequest, return true on failure, false on success
*/
bool _sine_esc_SetMotorSoundRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetMotorSoundRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->beep_num);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->beep_data.len);
        *bit_ofs += 5;
    } else {
        msg->beep_data.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->beep_data.len > 30) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->beep_data.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->beep_data.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetMotorSoundRequest sample_sine_esc_SetMotorSoundRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
