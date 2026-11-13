#pragma once
#include <sine_esc.GetBootStatus_req.h>
#include <sine_esc.GetBootStatus_res.h>

#define SINE_ESC_GETBOOTSTATUS_ID 200
#define SINE_ESC_GETBOOTSTATUS_SIGNATURE (0x8BA32EBA566F5713ULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_GetBootStatus, SINE_ESC_GETBOOTSTATUS_ID, SINE_ESC_GETBOOTSTATUS_SIGNATURE, SINE_ESC_GETBOOTSTATUS_REQUEST_MAX_SIZE, SINE_ESC_GETBOOTSTATUS_RESPONSE_MAX_SIZE);
#endif
