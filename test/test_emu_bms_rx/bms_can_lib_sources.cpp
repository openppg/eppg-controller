// Builds the pinned Adafruit CAN base class, Adafruit MCP2515 driver and
// ANT-BMS-CAN for the host; the libraries themselves are lib_ignore'd. Their
// SPI traffic lands in the MCP2515 model in test/emu_stubs/can.
#include <CANController.cpp>
#include <Adafruit_MCP2515.cpp>
#include <BMS_CAN.cpp>
