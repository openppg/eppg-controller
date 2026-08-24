// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// FIRST_BOOT_QC — factory self-test + per-unit throttle calibration capture.
// Runs as a blocking guided flow inside setup() at the Phase 4/5 boundary
// (display + hardware up, no app tasks running). See FIRST_BOOT_QC.md.
//
// Entry paths: truly fresh NVS, prior failed/aborted attempt (qc_attempted),
// or the serial "run_qc" command flag. The installed fleet (user settings,
// never attempted) is back-filled as passed and never sees the flow.

#ifndef INC_SP140_FIRST_BOOT_QC_H_
#define INC_SP140_FIRST_BOOT_QC_H_

// Capture boot context BEFORE refreshDeviceData() runs. refreshDeviceData()
// writes defaults into the "openppg" namespace on a fresh unit, which would
// make a brand-new board indistinguishable from an existing fleet unit — so
// the fresh-vs-legacy probe must happen first. Single-threaded setup() only.
void qcCaptureBootContext();

// Evaluate the gate (and perform the legacy back-fill / rerun-flag consume
// side effects). Returns true if the QC flow should run this boot.
bool qcShouldRun();

// Run the blocking QC flow. Call at the Phase 4/5 boundary in setup().
// Never arms and never sends throttle/setpoint commands to the ESC.
void runFirstBootQc();

#endif  // INC_SP140_FIRST_BOOT_QC_H_
