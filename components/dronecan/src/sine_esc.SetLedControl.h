#pragma once
#include <sine_esc.SetLedControl_req.h>
#include <sine_esc.SetLedControl_res.h>

#define SINE_ESC_SETLEDCONTROL_ID 218
#define SINE_ESC_SETLEDCONTROL_SIGNATURE (0x40C65964649CAE65ULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_SetLedControl, SINE_ESC_SETLEDCONTROL_ID, SINE_ESC_SETLEDCONTROL_SIGNATURE, SINE_ESC_SETLEDCONTROL_REQUEST_MAX_SIZE, SINE_ESC_SETLEDCONTROL_RESPONSE_MAX_SIZE);
#endif
