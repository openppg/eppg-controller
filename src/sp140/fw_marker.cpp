// Copyright <Zach Whitehead>
// Self-identifying firmware marker embedded in the binary image.
//
// The OTA backend scans an uploaded .bin for the 8-byte magic "OPPGFWV1" and
// reads the version fields that immediately follow it, so every build is
// identifiable from the binary alone (no sidecar file). Keep this layout in
// sync with the OTA backend parser (openppg-ota: src/utils/firmware.ts).

#include <cstdint>
#include "version.h"

struct __attribute__((packed)) FwVersionMarker {
  char magic[8];    // "OPPGFWV1" (no NUL terminator) — locator + identity
  uint8_t major;
  uint8_t minor;
  uint32_t build;   // little-endian
};

// No code references this symbol. platformio.ini keeps it in the image with
// `-Wl,-u,fw_version_marker`, which makes the linker treat it as a GC root.
// extern "C" gives it the unmangled name that linker flag refers to.
extern "C" const FwVersionMarker fw_version_marker = {
    {'O', 'P', 'P', 'G', 'F', 'W', 'V', '1'},
    VERSION_MAJOR,
    VERSION_MINOR,
    VERSION_BUILD,
};
