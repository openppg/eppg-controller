# Rust migration advice

Prepared 2026-11-12 against commit `35eba22` **and the existing uncommitted working tree**.
The existing `plans/` directory contains a separate factory-QC plan, so this assessment uses `advisor-plans/`.

| Plan | Scope | Priority | Effort | Depends on | Status |
| --- | --- | --- | --- | --- | --- |
| [001](001-rust-port.md) | Remove Arduino; port the complete active ESP32-S3 controller application to Rust, preserving interfaces and behavior | P1 | L | Compatibility baseline and feasibility gates in the plan | TODO |

The assessment is complete; implementation has not started. No firmware source was changed.
`001-baseline.sha256` records the source/configuration files present during this assessment, including untracked additions.

Decisions considered:

- **Recommended:** Rust application + ESP-IDF/FreeRTOS, retaining LVGL 9.5 and initially the pinned C DroneCAN engine/codecs.
- **Deferred:** an ESP-IDF-free `esp-hal`/Embassy/radio migration. It changes substantially more infrastructure than removing Arduino requires.
- **Rejected for this migration:** replacing LVGL, changing on-wire protocols, changing throttle calibration/mapping, or rewriting the browser UI solely to change its language.

Full reasoning, specific benefits, compatibility inventory, architecture, implementation sequence, and release gates are in plan 001.
