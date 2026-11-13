#pragma once
#include <sine_esc.SendFwData_req.h>
#include <sine_esc.SendFwData_res.h>

#define SINE_ESC_SENDFWDATA_ID 202
#define SINE_ESC_SENDFWDATA_SIGNATURE (0xF8D50AFD350A8FBBULL)

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(sine_esc_SendFwData, SINE_ESC_SENDFWDATA_ID, SINE_ESC_SENDFWDATA_SIGNATURE, SINE_ESC_SENDFWDATA_REQUEST_MAX_SIZE, SINE_ESC_SENDFWDATA_RESPONSE_MAX_SIZE);
#endif
