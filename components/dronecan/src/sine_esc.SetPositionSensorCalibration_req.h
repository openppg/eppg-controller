#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_SETPOSITIONSENSORCALIBRATION_REQUEST_MAX_SIZE 51
#define SINE_ESC_SETPOSITIONSENSORCALIBRATION_REQUEST_SIGNATURE (0xB970F68E635BAD53ULL)
#define SINE_ESC_SETPOSITIONSENSORCALIBRATION_REQUEST_ID 225

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_SetPositionSensorCalibration_cxx_iface;
#endif

struct sine_esc_SetPositionSensorCalibrationRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_SetPositionSensorCalibration_cxx_iface;
#endif
    uint16_t cmd_id;
    struct { uint8_t len; uint8_t data[48]; }data;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_SetPositionSensorCalibrationRequest_encode(struct sine_esc_SetPositionSensorCalibrationRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_SetPositionSensorCalibrationRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_SetPositionSensorCalibrationRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_SetPositionSensorCalibrationRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetPositionSensorCalibrationRequest* msg, bool tao);
static inline bool _sine_esc_SetPositionSensorCalibrationRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetPositionSensorCalibrationRequest* msg, bool tao);
void _sine_esc_SetPositionSensorCalibrationRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_SetPositionSensorCalibrationRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->cmd_id);
    *bit_ofs += 16;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t data_len = msg->data.len > 48 ? 48 : msg->data.len;
#pragma GCC diagnostic pop
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 6, &data_len);
        *bit_ofs += 6;
    }
    for (size_t i=0; i < data_len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode sine_esc_SetPositionSensorCalibrationRequest, return true on failure, false on success
*/
bool _sine_esc_SetPositionSensorCalibrationRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_SetPositionSensorCalibrationRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->cmd_id);
    *bit_ofs += 16;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->data.len);
        *bit_ofs += 6;
    } else {
        msg->data.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->data.len > 48) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->data.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_SetPositionSensorCalibrationRequest sample_sine_esc_SetPositionSensorCalibrationRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
