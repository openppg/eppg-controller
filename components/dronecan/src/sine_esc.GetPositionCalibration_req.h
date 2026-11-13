#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define SINE_ESC_GETPOSITIONCALIBRATION_REQUEST_MAX_SIZE 6
#define SINE_ESC_GETPOSITIONCALIBRATION_REQUEST_SIGNATURE (0xA895FDA3617029D2ULL)
#define SINE_ESC_GETPOSITIONCALIBRATION_REQUEST_ID 226

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class sine_esc_GetPositionCalibration_cxx_iface;
#endif

struct sine_esc_GetPositionCalibrationRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = sine_esc_GetPositionCalibration_cxx_iface;
#endif
    uint16_t cmd_id;
    int32_t resv_data;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t sine_esc_GetPositionCalibrationRequest_encode(struct sine_esc_GetPositionCalibrationRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool sine_esc_GetPositionCalibrationRequest_decode(const CanardRxTransfer* transfer, struct sine_esc_GetPositionCalibrationRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _sine_esc_GetPositionCalibrationRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetPositionCalibrationRequest* msg, bool tao);
static inline bool _sine_esc_GetPositionCalibrationRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetPositionCalibrationRequest* msg, bool tao);
void _sine_esc_GetPositionCalibrationRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct sine_esc_GetPositionCalibrationRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->cmd_id);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->resv_data);
    *bit_ofs += 32;
}

/*
 decode sine_esc_GetPositionCalibrationRequest, return true on failure, false on success
*/
bool _sine_esc_GetPositionCalibrationRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct sine_esc_GetPositionCalibrationRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->cmd_id);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->resv_data);
    *bit_ofs += 32;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_GetPositionCalibrationRequest sample_sine_esc_GetPositionCalibrationRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
