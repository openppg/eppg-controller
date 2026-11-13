/* Test-only vendor encoder oracle. Synthetic values; no unlock credentials. */
#include <sine_esc.SetMotorSound_req.h>
#include <sine_esc.SetLedControl_req.h>
#include <sine_esc.StartFwUpgrade_req.h>
#include <sine_esc.SendFwData_req.h>
#include <uavcan.protocol.RestartNode_req.h>

unsigned eppg_test_codec(unsigned fixture, unsigned char *out) {
    if (fixture < 2) {
        struct sine_esc_SetMotorSoundRequest msg = {
            .beep_num = 2, .beep_data = {.len = 6, .data = {2, 20, 80, 8, 20, 80}}};
        if (fixture == 1) { msg.beep_data.data[0] = 8; msg.beep_data.data[3] = 2; }
        return sine_esc_SetMotorSoundRequest_encode(&msg, out);
    }
    if (fixture < 8) {
        unsigned color = (fixture - 2) / 2 + 1;
        struct sine_esc_SetLedControlRequest msg = {.led_num = 2,
            .led_data = {.len = 2, .data = {(60 << 8) | (color + 3), 20 << 8}}};
        if (fixture % 2) {
            msg.led_num = 4; msg.led_data.len = 4;
            msg.led_data.data[0] = 256 | color; msg.led_data.data[1] = 512;
            msg.led_data.data[2] = 256 | color; msg.led_data.data[3] = 30 << 8;
        }
        return sine_esc_SetLedControlRequest_encode(&msg, out);
    }
    if (fixture == 8) {
        struct sine_esc_SetLedControlRequest msg = {
            .led_num = 1, .led_data = {.len = 1, .data = {20 << 8}}};
        return sine_esc_SetLedControlRequest_encode(&msg, out);
    }
    if (fixture == 9) {
        struct uavcan_protocol_RestartNodeRequest msg = {
            .magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER};
        return uavcan_protocol_RestartNodeRequest_encode(&msg, out);
    }
    if (fixture == 10) {
        struct sine_esc_StartFwUpgradeRequest msg = {
            .hardware_id = 0x1234, .filse_size_kb = 0x5678, .file_crc = 0x9abc};
        return sine_esc_StartFwUpgradeRequest_encode(&msg, out);
    }
    struct sine_esc_SendFwDataRequest msg = {.index = 0};
    for (unsigned i = 0; i < 256; ++i) msg.data[i] = i;
    return sine_esc_SendFwDataRequest_encode(&msg, out);
}
