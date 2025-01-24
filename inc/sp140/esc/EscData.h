#ifndef ESC_DATA_H
#define ESC_DATA_H

#include <Arduino.h>
#include "sp140/structs.h"


typedef struct {

STR_ESC_TELEMETRY_140 escTelemetryData;
telem_esc_t raw_esc_telemdata;

} EscData;

#endif