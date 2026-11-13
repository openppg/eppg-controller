#ifndef SINE_ESC_H_
#define SINE_ESC_H_

#include <CanardAdapter.h>
#include <dronecan_msgs.h>
#include <stdint.h>

struct SineEscModel
{
    bool hasBroadcastStatusInfo;
    sine_esc_BroadcastStatusInfo broadcastStatusInfo;

    bool hasSetLedControlResponse;
    sine_esc_SetLedControlResponse setLedControlResponse;

    bool hasSetMotorSoundResponse;
    sine_esc_SetMotorSoundResponse setMotorSoundResponse;

    bool hasSetThrottleSettings2Response;
    sine_esc_SetThrottleSettings2Response setThrottleSettings2Response;

    bool hasSetRotationSpeedSettingsResponse;
    sine_esc_SetRotationSpeedSettingsResponse setRotationSpeedSettingsResponse;

    bool hasGetHardwareInfoResponse;
    sine_esc_GetHwInfoResponse getHardwareInfoResponse;

    bool hasSetEscAddressResponse;
    sine_esc_SetEscAddressingResponse setEscAddressResponse;
};

class SineEsc : public CanardAdapterNode
{
public:
    enum LedPattern : uint8_t
    {
        LED_OFF = 0,
        LED_RED = 1,
        LED_GREEN = 2,
        LED_YELLOW = 3,
        LED_RED_BREATH = 4,
        LED_GREEN_BREATH = 5,
        LED_YELLOW_BREATH = 6
    };

    SineEsc(CanardAdapter &adapter);

    void begin(uint8_t nodeId);

    bool shouldAcceptTransfer(uint64_t* out_data_type_signature,
                                      uint16_t data_type_id,
                                      CanardTransferType transfer_type,
                                      uint8_t source_node_id) override;

    void onTransferReceived(CanardRxTransfer* transfer) override;

    const SineEscModel& getModel(void);
    void clearModelReceiveFlags(void);

    static uint16_t makeLedControlEntry(uint8_t pattern, uint8_t duration50ms);

    static void makeBeepEntry(uint8_t *out, uint8_t tone, uint8_t duration10ms, uint8_t volume);

    void setRotationSpeedSettings(int32_t speed);

    void setThrottleSettings2(uint16_t pwm_us);

    void setLedControl(const uint16_t *ledData, uint8_t ledNum);

    void setMotorSound(const uint8_t *beepData, uint8_t beepNum);

    void getHardwareInfo(void);

    void setEscAddress(uint8_t escId);

private:
    uint8_t _nodeId;
    uint8_t _setLedControlTransferId;
    uint8_t _setMotorSoundTransferId;
    uint8_t _setThrottleSettings2TransferId;
    uint8_t _setRotationSpeedSettingsTransferId;
    uint8_t _getHardwareInfoTransferId;
    uint8_t _setEscAddressTransferId;
    SineEscModel _model;

    void _handleSetThrottleSettings2Response(CanardRxTransfer *transfer);
    void _handleSetLedControlResponse(CanardRxTransfer *transfer);
    void _handleSetMotorSoundResponse(CanardRxTransfer *transfer);
    void _handleSetRotationSpeedSettingsResponse(CanardRxTransfer *transfer);
    void _handleBroadcastStatusInfo(CanardRxTransfer *transfer);
    void _handleGetHardwareInfoResponse(CanardRxTransfer *transfer);
    void _handleSetEscAddressingResponse(CanardRxTransfer *transfer);
};

#endif // SINE_ESC_H_
