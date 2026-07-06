// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// Factory-persistent settings ("openppg-factory" NVS namespace — exactly the
// 15-char NVS limit). Separate from the user "openppg" namespace on purpose:
// resetDeviceData() / user factory reset must NEVER wipe factory calibration
// or QC state. See FIRST_BOOT_QC.md.

#ifndef INC_SP140_FACTORY_SETTINGS_H_
#define INC_SP140_FACTORY_SETTINGS_H_

#include <stdint.h>
#include <stddef.h>

struct FactoryCal {
  bool calibrated;
  uint16_t potMin;
  uint16_t potMax;
};

// Create the module mutex + probe the namespace. Call once, single-threaded,
// early in setup() (before any other factory* call).
void factorySettingsInit();

// --- QC gate state ---
bool factoryQcPassed();
bool factoryRerunRequested();     // qc_rerun flag (set by the run_qc command)
void factorySetRerunFlag();       // called by the "run_qc" serial command
void factoryClearRerunFlag();     // consumed at boot by the QC gate

// --- Results ---
// qc_passed + qc_fw in one commit.
void factoryWriteQcResult(bool passed, uint16_t fwEncoded);
// pot_min/pot_max + pot_calibrated=1 in one commit.
void factoryWriteCal(uint16_t potMin, uint16_t potMax);
// Migration guard: existing (pre-QC firmware) unit — back-fill qc_passed
// without calibration so the installed fleet never sees the QC flow.
void factoryMarkLegacyUnit(uint16_t fwEncoded);

FactoryCal factoryGetCal();

// POST helper: write+read+erase a scratch key in the factory namespace.
// Proves NVS is healthy end-to-end. Returns true on round-trip success.
bool factoryNvsRoundTrip();

// --- QC record blob (BLE fleet-sync surface reads this) ---
bool factoryWriteQcRecordBlob(const void* data, size_t len);
// Returns bytes read (0 if absent/too large for the buffer).
size_t factoryReadQcRecordBlob(void* out, size_t maxLen);

// Encode VERSION_MAJOR/VERSION_MINOR into the u16 stored as qc_fw.
inline uint16_t factoryEncodeFw(uint8_t major, uint8_t minor) {
  return (uint16_t)((uint16_t)major << 8 | minor);
}

#endif  // INC_SP140_FACTORY_SETTINGS_H_
