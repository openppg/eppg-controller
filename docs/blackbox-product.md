# Black Box Flight Logging – Product Overview

Audience: Product/Support
Goal: Describe what we log, how often, how it’s stored, and how support can request data. This is the **default SPIFFS-based implementation** (no partition changes). An optional custom-partition upgrade is outlined at the end.

## What We Log
- **Fast flight data** (armed/cruising): throttle PWM, pack volts/amps, ESC eRPM, device state, performance mode, temp max (ESC/BMS).
- **Slow system data** (1 Hz): pack power, SOC, cell min/max/delta, altitude, watt-hours used (cumulative), BMS/ESC connection flags, MOS/balance/T-sensor temps, ESC error bitmasks.
- **Cell snapshots**: full 24-cell voltage table + min/max indices.
- **Events**: state transitions (ARM/DISARM/CRUISE), ESC/BMS connect/disconnect, error/warning changes, altitude zeroing, performance mode changes.

## Logging Rates (state-aware, ultra-extended)
- **Armed/Flight**: Fast **1 Hz**; Slow **0.1 Hz**; Cells every **100 s**.
- **Disarmed**: Optional minimal logging (e.g., Slow **0.02 Hz**, Cells **300 s**) or OFF.
- **Events**: On change (edge-triggered), not periodic.

## Storage (Default – No Partition Change)
- Uses existing **SPIFFS** partition (about **1.35 MB usable** after filesystem overhead).
- Data stored as 4 KB “block” files inside `/blackbox/` (ring buffer; oldest data overwritten).
- Capacity (1 Hz fast, 0.1 Hz slow, cells 100 s):
  - **~14 hours** continuous armed logging.
  - Typical day (1 h flight + 23 h ground): **~6–7 days** of history with minimal disarmed logging; **~14 days** if disarmed logging is OFF.
  - Multiple flights per day are supported; ring rolls over when full.

## Offload (USB, DISARMED only)
- Commands via existing WebSerial JSON:
  - `{"bb":"manifest"}` → summary and flight list.
  - `{"bb":"read_range","start_block":N,"end_block":M}` → bulk download.
  - `{"bb":"stats"}` → queue usage, drops, totals.
  - `{"bb":"erase"}` → two-step confirmed erase.
- Python helper tools (planned): pull → decode (CSV/JSON) → optional plots.

## Safety & Performance
- Flight control never blocked: logging is async with droppable fast-frames under backpressure.
- DISARM gate on all offload/erase commands.
- SPIFFS wear-leveling handles flash longevity automatically.

## Optional: Custom Partition (More Capacity)
- **What it gives**: Dedicated 2–3 MB log partition → **60–90+ minutes** at 10–20 Hz, lower overhead, more predictable timing.
- **Pros / Cons (Quick View)**:

| Mode | Pros | Cons |
|---|---|---|
| **Default (SPIFFS)** | No partition change; OTA-safe; settings preserved; debuggable files; wear-leveling handled | ~69 min capacity at 10 Hz; filesystem overhead; slightly slower/less deterministic |
| **Custom Partition** | Higher capacity (60–90+ min); faster raw writes; smaller overhead; more deterministic | Requires custom `partitions.csv` flash; shrinks SPIFFS assets; more invasive deployment; careful migration for USB full-flash |

- **When to use**: Endurance flights >70 min or customers needing maximum forensic window.
- **Deployment note**: Keep SPIFFS as default; offer an opt-in “extended logging” firmware with clear migration steps.

