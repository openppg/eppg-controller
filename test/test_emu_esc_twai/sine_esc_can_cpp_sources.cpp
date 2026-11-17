// Builds the C++ sources of the pinned SINE-ESC-CAN library (the TWAI
// CanardAdapter and the SineEsc node) for the host; the library itself is
// lib_ignore'd. Their TWAI calls land in the driver model in
// test_emu_esc_twai.cpp.
#include <CanardAdapter.cpp>
#include <SineEsc.cpp>
