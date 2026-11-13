#pragma once
#include <sine_esc.GetHwInfo_req.h>
#include <sine_esc.GetHwInfo_res.h>

#define SINE_ESC_GETHWINFO_ID 209
#define SINE_ESC_GETHWINFO_SIGNATURE (0x65DC98CD51449263ULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_GetHwInfo, SINE_ESC_GETHWINFO_ID, SINE_ESC_GETHWINFO_SIGNATURE, SINE_ESC_GETHWINFO_REQUEST_MAX_SIZE, SINE_ESC_GETHWINFO_RESPONSE_MAX_SIZE);
#endif
