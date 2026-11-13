#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETLEDCONTROL_REQUEST_MAX_SIZE 22
#define SINE_ESC_SETLEDCONTROL_REQUEST_SIGNATURE (0x40C65964649CAE65ULL)
#define SINE_ESC_SETLEDCONTROL_REQUEST_ID 218

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetLedControl_cxx_iface;
#endif

struct sine_esc_SetLedControlRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetLedControl_cxx_iface;
#endif
    uint8_t led_num;
    struct { uint8_t len; uint16_t data[10]; }led_data;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetLedControlRequest_encode(struct sine_esc_SetLedControlRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetLedControlRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SetLedControlRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetLedControlRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetLedControlRequest* msg, bool tao);
static inline bool _sine_esc_SetLedControlRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetLedControlRequest* msg, bool tao);
void _sine_esc_SetLedControlRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetLedControlRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->led_num);
    *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t led_data_len = msg->led_data.len > 10 ? 10 : msg->led_data.len;
#pragma GCC diagnostic pop
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 4, &led_data_len);
        *bit_ofs += 4;
    }
    for (size_t i=0; i < led_data_len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 16, &msg->led_data.data[i]);
        *bit_ofs += 16;
    }
}

/*
 decode sine_esc_SetLedControlRequest, return true on failure, false on success
*/
bool _sine_esc_SetLedControlRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetLedControlRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->led_num);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->led_data.len);
        *bit_ofs += 4;
    } else {
        msg->led_data.len = ((transfer->payload_len*8)-*bit_ofs)/16;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->led_data.len > 10) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->led_data.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->led_data.data[i]);
        *bit_ofs += 16;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetLedControlRequest sample_sine_esc_SetLedControlRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
