// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// Factory QC screen (FIRST_BOOT_QC). Three views on one screen object:
//   - checklist: title + up to 8 POST rows (name left, status right)
//   - prompt:    big instruction + live value + progress bar (guided steps)
//   - banner:    full-screen PASSED / FAILED result
// Driven single-threaded from the QC flow in first_boot_qc.cpp; also compiled
// into the native screenshot harness.

#ifndef INC_SP140_LVGL_LVGL_QC_SCREEN_H_
#define INC_SP140_LVGL_LVGL_QC_SCREEN_H_

#include <lvgl.h>
#include "sp140/qc_logic.h"

// Max rows in the checklist view (POST checks).
#define QC_SCREEN_MAX_ROWS 8

// Create + load the QC screen (checklist view visible, all rows pending).
void setupQcScreen(bool darkMode);

// Update one checklist row. `value` is optional right-aligned detail text
// (e.g. "1002 hPa"); pass nullptr for none.
void qcScreenSetCheck(uint8_t row, const char* name, QcCheckStatus status,
                      const char* value);

// Switch to the guided prompt view. `instruction` is the big line
// ("RELEASE THROTTLE"), `subtext` the smaller helper line.
void qcScreenPrompt(const char* instruction, const char* subtext);

// Update the large live value on the prompt view (pre-formatted text —
// raw pot counts, countdown seconds, etc.).
void qcScreenPromptValue(const char* text);

// Update the prompt progress bar (0-100). Used for stability progress and
// confirm countdowns.
void qcScreenPromptProgress(uint8_t pct);

// Return to the checklist view.
void qcScreenShowChecklist();

// Full-screen final banner. `detail` lists failed/skipped checks (may be "").
void qcScreenBanner(bool passed, const char* detail);

// Delete the QC screen and load `nextScreen` (normally main_screen).
void teardownQcScreen(lv_obj_t* nextScreen);

#endif  // INC_SP140_LVGL_LVGL_QC_SCREEN_H_
