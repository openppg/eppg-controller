# Flight Data "Black Box" Plan (ESP32-S3)

Goal: capture 30–90 minute flights with rolling storage, survive power loss, and offload over USB serial without affecting real-time control.

## Signals and Rates (from current code)
- Fast path (20–25 Hz, built in `throttleTask`): millis, device state, performance_mode, throttle PWM (final), ESC volts/amps/eRPM, wattHoursUsed, altitude + vertical speed (if baro valid).
- Slow path (1 Hz, built in `bmsTask`): pack V/I/P, SOC, highest/lowest cell V, delta V, max temps (MOS/MCU/CAP/Motor + BMS MOS/balance/T1–T4), charge/discharge MOS flags, battery cycle, ESC/BMS connection state.
- Cell detail (every 20 s or when delta V > 10 mV): full 24-cell table as uint16 mV.
- Events (edge-triggered via MultiLogger sink): state transitions, ESC/BMS connect/disconnect, running_error/selfcheck_error changes, MOS open/close, baro present flag, watchdog reset.

## Record Types (fixed, quantized)
- Fast frame ~24 B: {ts32, state8, mode8, throttle16, v_pack16 (0.01 V), i_pack16 (0.1 A), rpm16, wh16 (0.01 Wh), alt16 (0.1 m), vsi16 (0.01 m/s), esc_temp_max8, flags8}.
- Slow frame ~32–40 B: {ts32, v_pack16, i_pack16, p_pack16 (0.1 W), soc8 (0.5%), v_hi16, v_lo16, dv16, mos_temp8, mcu_temp8, cap_temp8, motor_temp8, bms_mos8, balance8, t1..t4 temps8, charge_mos1, discharge_mos1, bmsState2, escState2}.
- Cell snapshot ~60 B: {ts32, v_min16 + idx8, v_max16 + idx8, 24x uint16 mV}.
- Event frame 8–12 B: {ts32, event_id8, payload (bitfield or small ints)}.

## Flash Layout
- Add partition `data,log,0x3F0000,4M` (adjust to available space) with 4 KB erase blocks.
- Block header (per 4 KB): magic, block_seq, flight_session_id, start_ts, record_count, CRC32, state {empty|writing|closed}.
- Two superblocks (block 0 and 1) store head/tail block indices and flight counter; update only on block close (alternate copies).
- True ring: when head catches tail, erase tail and advance.

## Runtime Pipeline
- Producers: existing `pushTelemetrySnapshot()` keeps queue 1; add dedicated logger queue (len 4–8) so logger does not race monitors.
- Fast frame builder: runs inside `throttleTask` at 50 Hz; samples throttle/ESC/altimeter; quantizes and pushes to logger queue (droppable).
- Slow frame builder: runs in `bmsTask` every 1 s; pushes non-droppable slow frame.
- Cell snapshot builder: timer/conditioned (delta V > 10 mV).
- Event hook: register a new MultiLogger sink that emits event frames on alert edges.
- Writer task (low prio): drains queues into a 4 KB staging buffer (PSRAM if present, else static). Pre-erases next block; writes, then marks header closed with CRC.
- Backpressure: if queue near full, drop only fast frames; never drop slow or event frames.

## Serial Offload Protocol (reuses webSerialTask, DISARMED gate)
- Commands (JSON):  
  - `{"bb":"manifest"}` → head/tail seq, flight ids, block size, log size.  
  - `{"bb":"read","seq":N,"offset":O,"len":L}` → stream bytes, end with CRC32.  
  - `{"bb":"format"}` → double-confirm then erase ring.  
  - Optional `{"bb":"bookmark"}` to mark a flight boundary.
- Flow control: ACK/NACK window or XON/XOFF; resume supported via seq+offset.

## Recovery and Wear
- On boot: scan from superblock head forward until first empty/invalid; rebuild head/tail; latest valid superblock wins.
- Power loss: at most lose current writing block; closed blocks are CRC-checked.
- Wear leveling: rotate blocks evenly; count erases in RAM per flight, persist occasionally.

## Capacity Check (4 MB example, 90 min)
- Fast frames: ~2.6 MB (24 B @20 Hz).  
- Slow frames: ~0.22 MB.  
- Cells/events/headers/CRC: <0.4 MB.  
- Fits with margin; still roll when full.

## Integration Steps (no code here)
1) Define record structs and block/superblock headers (quantized fields as above).  
2) Add `data,log` partition entry and locate it with `esp_partition_find_first`.  
3) Add logger queue + writer task; hook fast builder in `throttleTask`, slow in `bmsTask`, cell timer, event sink.  
4) Implement ring block manager (pre-erase, close, CRC, overwrite tail).  
5) Extend `parse_serial_commands()` with manifest/read/format plus flow control.  
6) Test: max-rate logging + induced brownouts; offload resume; ring overwrite after capacity; wear distribution.  
7) Ship small host script (Python) to pull manifest and decode frames.
