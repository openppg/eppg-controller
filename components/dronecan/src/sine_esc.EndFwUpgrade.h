#pragma once
#include <sine_esc.EndFwUpgrade_req.h>
#include <sine_esc.EndFwUpgrade_res.h>

#define SINE_ESC_ENDFWUPGRADE_ID 203
#define SINE_ESC_ENDFWUPGRADE_SIGNATURE (0xD1AC3920854F5803ULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_EndFwUpgrade, SINE_ESC_ENDFWUPGRADE_ID, SINE_ESC_ENDFWUPGRADE_SIGNATURE, SINE_ESC_ENDFWUPGRADE_REQUEST_MAX_SIZE, SINE_ESC_ENDFWUPGRADE_RESPONSE_MAX_SIZE);
#endif
