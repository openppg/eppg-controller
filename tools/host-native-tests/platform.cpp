#include "platform/runtime.h"
#include "sp140/esp32s3-config.h"
HardwareConfig board_config = {};
extern "C" int eppg_gpio_config(int, bool, bool) { return 0; }
extern "C" int eppg_adc_read(int) { return g_testAnalogReadValue; }
