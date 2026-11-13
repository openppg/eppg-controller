#define CANARD_DSDLC_INTERNAL
#include <sine_esc.BroadcastMechanicalAngle.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t sine_esc_BroadcastMechanicalAngle_encode(struct sine_esc_BroadcastMechanicalAngle* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, SINE_ESC_BROADCASTMECHANICALANGLE_MAX_SIZE);
    _sine_esc_BroadcastMechanicalAngle_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool sine_esc_BroadcastMechanicalAngle_decode(const CanardRxTransfer* transfer, struct sine_esc_BroadcastMechanicalAngle* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > SINE_ESC_BROADCASTMECHANICALANGLE_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_sine_esc_BroadcastMechanicalAngle_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct sine_esc_BroadcastMechanicalAngle sample_sine_esc_BroadcastMechanicalAngle_msg(void) {
    struct sine_esc_BroadcastMechanicalAngle msg;

    msg.mech_angle = (int16_t)random_bitlen_signed_val(16);
    return msg;
}
#endif
