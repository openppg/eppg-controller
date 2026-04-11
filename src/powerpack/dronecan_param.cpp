#include "powerpack/dronecan_param.h"

// =============================================================================
// ParamValue encode/decode
// =============================================================================
// DroneCAN Value is a union with 3-bit tag prefix:
//   0=empty, 1=integer(int64), 2=real(float32), 3=boolean(uint8), 4=string(uint8[<=128])
// The outer struct has void5 padding before the tag to ensure alignment,
// BUT in GetSet request the value comes right after uint13 index (13 bits),
// so the 5-bit void is part of the alignment dance.

uint32_t paramValue_encode(const ParamValue* val, uint8_t* buffer, uint32_t bit_ofs) {
    uint32_t start_ofs = bit_ofs;

    // 3-bit union tag
    uint8_t tag = val->tag;
    canardEncodeScalar(buffer, bit_ofs, 3, &tag);
    bit_ofs += 3;

    switch (tag) {
        case PARAM_VALUE_TAG_EMPTY:
            // Empty - no additional data
            break;
        case PARAM_VALUE_TAG_INTEGER:
            canardEncodeScalar(buffer, bit_ofs, 64, &val->integer_value);
            bit_ofs += 64;
            break;
        case PARAM_VALUE_TAG_REAL:
            canardEncodeScalar(buffer, bit_ofs, 32, &val->real_value);
            bit_ofs += 32;
            break;
        case PARAM_VALUE_TAG_BOOLEAN:
            canardEncodeScalar(buffer, bit_ofs, 8, &val->boolean_value);
            bit_ofs += 8;
            break;
        case PARAM_VALUE_TAG_STRING: {
            // Length prefix (8 bits) + data
            canardEncodeScalar(buffer, bit_ofs, 8, &val->string_value.len);
            bit_ofs += 8;
            for (uint8_t i = 0; i < val->string_value.len; i++) {
                canardEncodeScalar(buffer, bit_ofs, 8, &val->string_value.data[i]);
                bit_ofs += 8;
            }
            break;
        }
        default:
            break;
    }

    return bit_ofs - start_ofs;
}

uint32_t paramValue_decode(const CanardRxTransfer* transfer, uint32_t bit_ofs, ParamValue* val) {
    uint32_t start_ofs = bit_ofs;

    // 3-bit union tag
    uint8_t tag = 0;
    canardDecodeScalar(transfer, bit_ofs, 3, false, &tag);
    bit_ofs += 3;
    val->tag = tag;

    switch (tag) {
        case PARAM_VALUE_TAG_EMPTY:
            break;
        case PARAM_VALUE_TAG_INTEGER:
            canardDecodeScalar(transfer, bit_ofs, 64, true, &val->integer_value);
            bit_ofs += 64;
            break;
        case PARAM_VALUE_TAG_REAL:
            canardDecodeScalar(transfer, bit_ofs, 32, false, &val->real_value);
            bit_ofs += 32;
            break;
        case PARAM_VALUE_TAG_BOOLEAN:
            canardDecodeScalar(transfer, bit_ofs, 8, false, &val->boolean_value);
            bit_ofs += 8;
            break;
        case PARAM_VALUE_TAG_STRING: {
            canardDecodeScalar(transfer, bit_ofs, 8, false, &val->string_value.len);
            bit_ofs += 8;
            if (val->string_value.len > 128) val->string_value.len = 128;
            for (uint8_t i = 0; i < val->string_value.len; i++) {
                canardDecodeScalar(transfer, bit_ofs, 8, false, &val->string_value.data[i]);
                bit_ofs += 8;
            }
            break;
        }
        default:
            break;
    }

    return bit_ofs - start_ofs;
}

// =============================================================================
// param.GetSet Request encode
// =============================================================================
// Layout:
//   uint13 index
//   void5         (padding before Value union)
//   Value value    (3-bit tag + payload)
//   void5         (padding before name)
//   uint8[<=92] name  (8-bit length prefix + data)

uint32_t paramGetSetRequest_encode(const ParamGetSetRequest* req, uint8_t* buffer) {
    uint32_t bit_ofs = 0;

    // uint13 index
    uint16_t index = req->index & 0x1FFF;
    canardEncodeScalar(buffer, bit_ofs, 13, &index);
    bit_ofs += 13;

    // void5 padding
    bit_ofs += 5;

    // Value (3-bit tag + payload)
    bit_ofs += paramValue_encode(&req->value, buffer, bit_ofs);

    // void5 padding before name
    bit_ofs += 5;

    // name: uint8[<=92] with TAO (tail array optimization)
    // Since this is the last field, length is implicit from remaining bytes
    for (uint8_t i = 0; i < req->name_len; i++) {
        uint8_t c = (uint8_t)req->name[i];
        canardEncodeScalar(buffer, bit_ofs, 8, &c);
        bit_ofs += 8;
    }

    return (bit_ofs + 7) / 8;  // Return byte length
}

// =============================================================================
// param.GetSet Response decode
// =============================================================================
// Layout:
//   void5
//   Value value
//   void5
//   Value default_value
//   void6
//   NumericValue max_value   (2-bit tag + payload)
//   void6
//   NumericValue min_value   (2-bit tag + payload)
//   uint8[<=92] name         (TAO)

bool paramGetSetResponse_decode(const CanardRxTransfer* transfer, ParamGetSetResponse* res) {
    uint32_t bit_ofs = 0;

    // void5
    bit_ofs += 5;

    // Value value
    bit_ofs += paramValue_decode(transfer, bit_ofs, &res->value);

    // We only need the value and name for our purposes.
    // Skip default_value, max_value, min_value for now.
    // void5
    bit_ofs += 5;

    // default_value
    bit_ofs += paramValue_decode(transfer, bit_ofs, &res->default_value);

    // void6 + NumericValue max (2-bit tag: 0=empty, 1=int64, 2=float32)
    bit_ofs += 6;
    uint8_t max_tag = 0;
    canardDecodeScalar(transfer, bit_ofs, 2, false, &max_tag);
    bit_ofs += 2;
    if (max_tag == 1) bit_ofs += 64;       // int64
    else if (max_tag == 2) bit_ofs += 32;  // float32

    // void6 + NumericValue min
    bit_ofs += 6;
    uint8_t min_tag = 0;
    canardDecodeScalar(transfer, bit_ofs, 2, false, &min_tag);
    bit_ofs += 2;
    if (min_tag == 1) bit_ofs += 64;
    else if (min_tag == 2) bit_ofs += 32;

    // name: TAO - remaining bytes are the name
    res->name_len = 0;
    uint16_t total_bits = transfer->payload_len * 8;
    while (bit_ofs + 8 <= total_bits && res->name_len < 92) {
        uint8_t c = 0;
        canardDecodeScalar(transfer, bit_ofs, 8, false, &c);
        res->name[res->name_len++] = (char)c;
        bit_ofs += 8;
    }
    res->name[res->name_len] = '\0';

    return false;  // success
}

// =============================================================================
// param.ExecuteOpcode Request encode
// =============================================================================
// Layout:
//   uint8 opcode
//   int48 argument

uint32_t paramExecuteOpcodeRequest_encode(const ParamExecuteOpcodeRequest* req, uint8_t* buffer) {
    uint32_t bit_ofs = 0;

    canardEncodeScalar(buffer, bit_ofs, 8, &req->opcode);
    bit_ofs += 8;

    canardEncodeScalar(buffer, bit_ofs, 48, &req->argument);
    bit_ofs += 48;

    return (bit_ofs + 7) / 8;
}

// =============================================================================
// param.ExecuteOpcode Response decode
// =============================================================================
// Layout:
//   int48 argument
//   bool ok

bool paramExecuteOpcodeResponse_decode(const CanardRxTransfer* transfer, ParamExecuteOpcodeResponse* res) {
    uint32_t bit_ofs = 0;

    canardDecodeScalar(transfer, bit_ofs, 48, true, &res->argument);
    bit_ofs += 48;

    uint8_t ok = 0;
    canardDecodeScalar(transfer, bit_ofs, 1, false, &ok);
    res->ok = (ok != 0);

    return false;  // success
}
