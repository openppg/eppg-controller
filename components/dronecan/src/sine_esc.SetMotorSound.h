#pragma once
#include <sine_esc.SetMotorSound_req.h>
#include <sine_esc.SetMotorSound_res.h>

#define SINE_ESC_SETMOTORSOUND_ID 214
#define SINE_ESC_SETMOTORSOUND_SIGNATURE (0x40C65964649CAE65ULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_SetMotorSound, SINE_ESC_SETMOTORSOUND_ID, SINE_ESC_SETMOTORSOUND_SIGNATURE, SINE_ESC_SETMOTORSOUND_REQUEST_MAX_SIZE, SINE_ESC_SETMOTORSOUND_RESPONSE_MAX_SIZE);
#endif
