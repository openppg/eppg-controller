#pragma once
#include <sine_esc.StartFwUpgrade_req.h>
#include <sine_esc.StartFwUpgrade_res.h>

#define SINE_ESC_STARTFWUPGRADE_ID 201
#define SINE_ESC_STARTFWUPGRADE_SIGNATURE (0x9A5F16E0CDD390F7ULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_StartFwUpgrade, SINE_ESC_STARTFWUPGRADE_ID, SINE_ESC_STARTFWUPGRADE_SIGNATURE, SINE_ESC_STARTFWUPGRADE_REQUEST_MAX_SIZE, SINE_ESC_STARTFWUPGRADE_RESPONSE_MAX_SIZE);
#endif
