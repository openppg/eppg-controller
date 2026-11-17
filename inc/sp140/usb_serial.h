// USBSerial compatibility shim for Arduino-ESP32 core 3.x.
//
// Core 2.x defined a global `HWCDC USBSerial` (USB Serial/JTAG peripheral)
// whenever ARDUINO_USB_MODE=1 and ARDUINO_USB_CDC_ON_BOOT=0, which is this
// board's configuration. Core 3.x renamed it to `HWCDCSerial` and only
// instantiates it when CDC-on-boot is enabled (which would also remap
// `Serial`). This header restores the old global name; the single instance is
// defined in src/sp140/usb_serial.cpp. Include it in any file that uses
// USBSerial. It is a no-op outside the ESP32 build (native tests use stubs).
#ifndef INC_SP140_USB_SERIAL_H_
#define INC_SP140_USB_SERIAL_H_

#if defined(__cplusplus) && defined(ESP_PLATFORM)
#include <HWCDC.h>

extern HWCDC USBSerial;
#endif

#endif  // INC_SP140_USB_SERIAL_H_
