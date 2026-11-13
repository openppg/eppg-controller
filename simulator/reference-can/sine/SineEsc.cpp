#include "SineEsc.h"
#include <Arduino.h>

SineEsc::SineEsc(CanardAdapter &adapter)
: CanardAdapterNode(adapter)
{
}

void SineEsc::begin(uint8_t nodeId)
{
    _nodeId = nodeId;
    _beginNode();
}

bool SineEsc::shouldAcceptTransfer(uint64_t* out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id)
{
    bool accept = false;

    if (transfer_type == CanardTransferTypeResponse && source_node_id == _nodeId) {
        switch(data_type_id)
        {
            case SINE_ESC_SETLEDCONTROL_ID:
                *out_data_type_signature = SINE_ESC_SETLEDCONTROL_SIGNATURE;
                accept = true;
                break;

            case SINE_ESC_SETMOTORSOUND_ID:
                *out_data_type_signature = SINE_ESC_SETMOTORSOUND_SIGNATURE;
                accept = true;
                break;

            case SINE_ESC_SETTHROTTLESETTINGS2_ID:
                *out_data_type_signature = SINE_ESC_SETTHROTTLESETTINGS2_SIGNATURE;
                accept = true;
                break;

            case SINE_ESC_SETROTATIONSPEEDSETTINGS_ID:
                *out_data_type_signature = SINE_ESC_SETROTATIONSPEEDSETTINGS_SIGNATURE;
                accept = true;
                break;

            case SINE_ESC_GETHWINFO_REQUEST_ID:
                *out_data_type_signature = SINE_ESC_GETHWINFO_REQUEST_SIGNATURE;
                accept = true;
                break;

            case SINE_ESC_SETESCADDRESSING_ID:
                *out_data_type_signature = SINE_ESC_SETESCADDRESSING_SIGNATURE;
                accept = true;
                break;

            default:
                break;
        }
    } else if (transfer_type == CanardTransferTypeBroadcast && source_node_id == _nodeId) {
        switch(data_type_id)
        {
            case SINE_ESC_BROADCASTSTATUSINFO_ID:
                *out_data_type_signature = SINE_ESC_BROADCASTSTATUSINFO_SIGNATURE;
                accept = true;
                break;

            default:
                break;
        }
    }

    return accept;
}

void SineEsc::onTransferReceived(CanardRxTransfer* transfer)
{
    CanardTransferType transfer_type = static_cast<CanardTransferType>(transfer->transfer_type);
    uint8_t source_node_id = transfer->source_node_id;
    uint16_t data_type_id = transfer->data_type_id;

    if (transfer_type == CanardTransferTypeResponse && source_node_id == _nodeId) {
        switch(data_type_id)
        {
            case SINE_ESC_SETLEDCONTROL_ID:
                _handleSetLedControlResponse(transfer);
                break;

            case SINE_ESC_SETMOTORSOUND_ID:
                _handleSetMotorSoundResponse(transfer);
                break;

            case SINE_ESC_SETTHROTTLESETTINGS2_ID:
                _handleSetThrottleSettings2Response(transfer);
                break;

            case SINE_ESC_SETROTATIONSPEEDSETTINGS_ID:
                _handleSetRotationSpeedSettingsResponse(transfer);
                break;

            case SINE_ESC_GETHWINFO_ID:
                _handleGetHardwareInfoResponse(transfer);
                break;

            case SINE_ESC_SETESCADDRESSING_ID:
                _handleSetEscAddressingResponse(transfer);
                break;

            default:
                break;
        }
    } else if (transfer_type == CanardTransferTypeBroadcast && source_node_id == _nodeId) {
        switch(data_type_id)
        {
            case SINE_ESC_BROADCASTSTATUSINFO_ID:
                _handleBroadcastStatusInfo(transfer);
                break;

            default:
                break;
        }
    }
}

const SineEscModel& SineEsc::getModel(void)
{
    return _model;
}

void SineEsc::clearModelReceiveFlags(void)
{
    _model.hasGetHardwareInfoResponse = false;
    _model.hasBroadcastStatusInfo = false;
    _model.hasSetLedControlResponse = false;
    _model.hasSetMotorSoundResponse = false;
    _model.hasSetThrottleSettings2Response = false;
    _model.hasSetRotationSpeedSettingsResponse = false;
    _model.hasSetEscAddressResponse = false;
}

uint16_t SineEsc::makeLedControlEntry(uint8_t pattern, uint8_t duration50ms)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(duration50ms) << 8) | pattern);
}

void SineEsc::makeBeepEntry(uint8_t *out, uint8_t tone, uint8_t duration10ms, uint8_t volume)
{
    out[0] = tone;
    out[1] = duration10ms;
    out[2] = volume;
}

void SineEsc::setRotationSpeedSettings(int32_t speed)
{
    sine_esc_SetRotationSpeedSettingsRequest req = {
            .speed = speed
        };

    uint8_t buffer[SINE_ESC_SETROTATIONSPEEDSETTINGS_REQUEST_MAX_SIZE];
    uint32_t bufferLen = sine_esc_SetRotationSpeedSettingsRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
            .transfer_type = CanardTransferTypeRequest,
            .data_type_signature = SINE_ESC_SETROTATIONSPEEDSETTINGS_REQUEST_SIGNATURE,
            .data_type_id = SINE_ESC_SETROTATIONSPEEDSETTINGS_REQUEST_ID,
            .inout_transfer_id = &_setRotationSpeedSettingsTransferId,
            .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
            .payload = buffer,
            .payload_len = static_cast<uint16_t>(bufferLen)
        };

    uint16_t ret = canardRequestOrRespondObj(_canard, _nodeId, &transfer);

    if (ret <= 0) {
        // Something went wrong
    }
}

void SineEsc::setLedControl(const uint16_t *ledData, uint8_t ledNum)
{
    sine_esc_SetLedControlRequest req = {};

    if (ledData == nullptr && ledNum > 0) {
        return;
    }

    req.led_data.len = ledNum > 10 ? 10 : ledNum;
    req.led_num = req.led_data.len;

    for (uint8_t i = 0; i < req.led_data.len; ++i) {
        req.led_data.data[i] = ledData[i];
    }

    uint8_t buffer[SINE_ESC_SETLEDCONTROL_REQUEST_MAX_SIZE];
    uint32_t bufferLen = sine_esc_SetLedControlRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
            .transfer_type = CanardTransferTypeRequest,
            .data_type_signature = SINE_ESC_SETLEDCONTROL_REQUEST_SIGNATURE,
            .data_type_id = SINE_ESC_SETLEDCONTROL_REQUEST_ID,
            .inout_transfer_id = &_setLedControlTransferId,
            .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
            .payload = buffer,
            .payload_len = static_cast<uint16_t>(bufferLen)
        };

    uint16_t ret = canardRequestOrRespondObj(_canard, _nodeId, &transfer);

    if (ret <= 0) {
        // Something went wrong
    }
}

void SineEsc::setMotorSound(const uint8_t *beepData, uint8_t beepNum)
{
    sine_esc_SetMotorSoundRequest req = {};

    if (beepData == nullptr && beepNum > 0) {
        return;
    }

    uint8_t clampedNum = beepNum > 10 ? 10 : beepNum;
    req.beep_num = clampedNum;
    req.beep_data.len = clampedNum * 3;

    for (uint8_t i = 0; i < req.beep_data.len; ++i) {
        req.beep_data.data[i] = beepData[i];
    }

    uint8_t buffer[SINE_ESC_SETMOTORSOUND_REQUEST_MAX_SIZE];
    uint32_t bufferLen = sine_esc_SetMotorSoundRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
            .transfer_type = CanardTransferTypeRequest,
            .data_type_signature = SINE_ESC_SETMOTORSOUND_REQUEST_SIGNATURE,
            .data_type_id = SINE_ESC_SETMOTORSOUND_REQUEST_ID,
            .inout_transfer_id = &_setMotorSoundTransferId,
            .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
            .payload = buffer,
            .payload_len = static_cast<uint16_t>(bufferLen)
        };

    uint16_t ret = canardRequestOrRespondObj(_canard, _nodeId, &transfer);

    if (ret <= 0) {
        // Something went wrong
    }
}

void SineEsc::setThrottleSettings2(uint16_t pwm_us)
{
    sine_esc_SetThrottleSettings2Request req = {
            .pwm_us = pwm_us
        };

    uint8_t buffer[SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_MAX_SIZE];
    uint32_t bufferLen = sine_esc_SetThrottleSettings2Request_encode(&req, buffer);

    CanardTxTransfer transfer = {
            .transfer_type = CanardTransferTypeRequest,
            .data_type_signature = SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_SIGNATURE,
            .data_type_id = SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_ID,
            .inout_transfer_id = &_setThrottleSettings2TransferId,
            .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
            .payload = buffer,
            .payload_len = static_cast<uint16_t>(bufferLen)
        };

    uint16_t ret = canardRequestOrRespondObj(_canard, _nodeId, &transfer);

    if (ret <= 0) {
        // Something went wrong
    }
}

void SineEsc::getHardwareInfo(void)
{
    sine_esc_GetHwInfoRequest req;

    uint8_t buffer[SINE_ESC_GETHWINFO_REQUEST_MAX_SIZE];
    uint32_t bufferLen = sine_esc_GetHwInfoRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
            .transfer_type = CanardTransferTypeRequest,
            .data_type_signature = SINE_ESC_GETHWINFO_REQUEST_SIGNATURE,
            .data_type_id = SINE_ESC_GETHWINFO_REQUEST_ID,
            .inout_transfer_id = &_getHardwareInfoTransferId,
            .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
            .payload = buffer,
            .payload_len = static_cast<uint16_t>(bufferLen)
        };

    uint16_t ret = canardRequestOrRespondObj(_canard, _nodeId, &transfer);

    if (ret <= 0) {
        // Something went wrong
    }
}

void SineEsc::setEscAddress(uint8_t escId)
{
    sine_esc_SetEscAddressingRequest req = {
            .esc_id = escId
        };

    uint8_t buffer[SINE_ESC_SETESCADDRESSING_REQUEST_MAX_SIZE];
    uint32_t bufferLen = sine_esc_SetEscAddressingRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
            .transfer_type = CanardTransferTypeRequest,
            .data_type_signature = SINE_ESC_SETESCADDRESSING_REQUEST_SIGNATURE,
            .data_type_id = SINE_ESC_SETESCADDRESSING_REQUEST_ID,
            .inout_transfer_id = &_setEscAddressTransferId,
            .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
            .payload = buffer,
            .payload_len = static_cast<uint16_t>(bufferLen)
        };

    uint16_t ret = canardRequestOrRespondObj(_canard, _nodeId, &transfer);

    // Change the node ID so that we can get the response
    _nodeId = escId;

    if (ret <= 0) {
        // Something went wrong
    }
}

void SineEsc::_handleSetThrottleSettings2Response(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_SetThrottleSettings2Response_decode(
                        transfer,
                        &_model.setThrottleSettings2Response);
    _model.hasSetThrottleSettings2Response = parsed;
}

void SineEsc::_handleSetLedControlResponse(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_SetLedControlResponse_decode(
                        transfer,
                        &_model.setLedControlResponse);
    _model.hasSetLedControlResponse = parsed;
}

void SineEsc::_handleSetMotorSoundResponse(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_SetMotorSoundResponse_decode(
                        transfer,
                        &_model.setMotorSoundResponse);
    _model.hasSetMotorSoundResponse = parsed;
}

void SineEsc::_handleSetRotationSpeedSettingsResponse(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_SetRotationSpeedSettingsResponse_decode(
                        transfer,
                        &_model.setRotationSpeedSettingsResponse);
    _model.hasSetRotationSpeedSettingsResponse = parsed;
}

void SineEsc::_handleBroadcastStatusInfo(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_BroadcastStatusInfo_decode(
                        transfer,
                        &_model.broadcastStatusInfo);
    _model.hasBroadcastStatusInfo = parsed;
}

void SineEsc::_handleGetHardwareInfoResponse(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_GetHwInfoResponse_decode(
                        transfer,
                        &_model.getHardwareInfoResponse);
    _model.hasGetHardwareInfoResponse = parsed;
}

void SineEsc::_handleSetEscAddressingResponse(CanardRxTransfer *transfer)
{
    bool parsed = !sine_esc_SetEscAddressingResponse_decode(
                        transfer,
                        &_model.setEscAddressResponse);
    _model.hasSetEscAddressResponse = parsed;
}
