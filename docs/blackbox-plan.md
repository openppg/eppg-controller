# Flight Data "Black Box" Plan (ESP32-S3FN8)

**Hardware**: M5Stack Stamp-S3A (ESP32-S3FN8: 8 MB Flash, NO PSRAM, 512 KB SRAM)
**Goal**: Capture 60+ minute flights with rolling storage, survive power loss, and offload over USB serial without affecting real-time control.

## Signals and Rates (from current code)
- **Fast path** (50 Hz actual in `throttleTask`, **decimate to 20 Hz** for logging): timestamp64, device state, performance_mode, throttle PWM (final), ESC volts/amps/eRPM, wattHoursUsed, altitude + vertical speed (if baro valid).
- **Slow path** (10 Hz in `bmsTask`, **sample at 1 Hz** for logging): pack V/I/P, SOC, highest/lowest cell V, delta V, max temps (MOS/MCU/CAP/Motor + BMS MOS/balance/T1–T4), charge/discharge MOS flags, battery cycle, ESC/BMS connection state.
- **Cell detail** (every 20 s or when `voltage_differential` changes > 10 mV from last snapshot): full 24-cell table as uint16 mV.
- **Events** (edge-triggered via MultiLogger sink): state transitions (DISARMED/ARMED/CRUISING), ESC/BMS connect/disconnect, running_error/selfcheck_error changes, MOS open/close, altitude zero reset on arm, cruise activation/deactivation with PWM value, performance mode changes, button sequences.

## Record Types (fixed, quantized)
- **Fast frame** ~28 B: {ts64 (µs), state8, mode8, throttle16, v_pack16 (0.01 V), i_pack16 (0.1 A), rpm16, wh16 (0.01 Wh), alt16 (0.1 m), vsi16 (0.01 m/s), esc_temp_max8, flags8}.
- **Slow frame** ~36–44 B: {ts64 (µs), v_pack16, i_pack16, p_pack16 (0.1 W), soc8 (0.5%), v_hi16, v_lo16, dv16, mos_temp8, mcu_temp8, cap_temp8, motor_temp8, bms_mos8, balance8, t1..t4 temps8, charge_mos1, discharge_mos1, bmsState2, escState2}.
- **Cell snapshot** ~64 B: {ts64 (µs), v_min16 + idx8, v_max16 + idx8, 24x uint16 mV}.
- **Event frame** 12–16 B: {ts64 (µs), event_id8, payload (bitfield or small ints)}.

_Note: Using 64-bit microsecond timestamps via `esp_timer_get_time()` to avoid 49.7-day rollover issues with 32-bit `millis()`._

## Flash Layout (ESP32-S3FN8: 8 MB total)

**Partition Table** (`partitions.csv`):
```csv
# Name,     Type, SubType,  Offset,   Size,     Flags
nvs,        data, nvs,      0x9000,   0x5000,
otadata,    data, ota,      0xe000,   0x2000,
app0,       app,  ota_0,    0x10000,  0x2A0000,
app1,       app,  ota_1,    0x2B0000, 0x2A0000,
spiffs,     data, spiffs,   0x550000, 0xA0000,
blackbox,   data, 0x40,     0x5F0000, 0x200000,
```

**Breakdown**:
- `nvs`: 20 KB (unchanged location for safe OTA upgrades, preserves Preferences)
- `otadata`: 8 KB (OTA control)
- `app0/app1`: 2.7 MB each (current build: 1.4 MB, 93% headroom for growth)
- `spiffs`: 640 KB (assets/fonts)
- `blackbox`: **2 MB** (custom data, subtype 0x40)
- Total: ~7.7 MB of 8 MB

**Block Structure**:
- 4 KB erase blocks (ESP32-S3 flash sector size)
- Block header (per 4 KB): magic (4B), block_seq (4B), flight_session_id (2B), start_ts (8B), record_count (2B), CRC32 (4B), state (1B: empty|writing|closed), reserved (7B pad to align).
- Two superblocks (block 0 and 1) store head/tail block indices and flight counter; update only on block close (alternate copies for power-loss safety).
- True ring: when head catches tail, erase tail and advance.

**Access API**: `esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "blackbox")`

## Runtime Pipeline
- **Producers**: Existing `pushTelemetrySnapshot()` keeps monitoring queue separate; add dedicated logger queue (len **256** frames = 5s buffer @ 50Hz) for burst protection during flash erases.
- **Fast frame builder**: Runs inside `throttleTask` at 50 Hz; **decimates to 20 Hz** (log every 3rd sample) before quantizing and pushing to logger queue (droppable on overflow).
- **Slow frame builder**: Runs in `bmsTask` at 10 Hz; **samples at 1 Hz** for logging; pushes non-droppable slow frame.
- **Cell snapshot builder**: Triggered when `abs(voltage_differential - lastLoggedDelta) > 0.010f` (10 mV change in max-min cell spread).
- **wattHoursUsed integration**: Calculate in `bmsTask` using `wattHoursUsed += unifiedBatteryData.power * 1000.0f * (dt_hours)` for fast frame logging.
- **Event hook**: Register `BlackBoxLogger : ILogger` sink into existing `multiLogger` to capture all 70+ sensor events automatically on alert edges.
- **Writer task** (low prio, core 1): Drains queues into 4 KB staging buffer in **SRAM** (no PSRAM on FN8 variant). Pre-erases next block asynchronously; writes current block, then marks header closed with CRC32.
- **Backpressure**: If queue near full (>90%), drop only fast frames; never drop slow, cell, or event frames.

## Serial Offload Protocol (reuses webSerialTask, DISARMED gate)
**Commands** (JSON, added to existing `parse_serial_commands()`):
- `{"bb":"manifest"}` → Returns: `{head_seq, tail_seq, num_flights, block_size:4096, total_size, free_blocks}`.
- `{"bb":"read","seq":N,"offset":O,"len":L}` → Stream binary block data in 256-byte chunks with per-chunk CRC32.
- `{"bb":"erase"}` → Full ring erase (requires confirmation: `{"bb":"erase_confirm","token":XXXX}`).
- `{"bb":"bookmark"}` → Insert event marker at current write position to denote flight boundary.

**Flow Control**: Window-based (send 4KB chunk, wait for `{"ack":true}` before next). Resume supported via `seq+offset` on connection loss. Timeout: 5s per chunk.

**Decoder**: Python script (`blackbox_decode.py`) to pull manifest, read all blocks, parse record types, output CSV/JSON for analysis tools.

## Recovery and Wear
- On boot: scan from superblock head forward until first empty/invalid; rebuild head/tail; latest valid superblock wins.
- Power loss: at most lose current writing block; closed blocks are CRC-checked.
- Wear leveling: rotate blocks evenly; count erases in RAM per flight, persist occasionally.

## Capacity Check (2 MB partition, 63 min rolling window)
**Data rates**:
- Fast frames: 28 B × 20 Hz = 560 B/s
- Slow frames: 40 B × 1 Hz = 40 B/s
- Cell snapshots: ~64 B every 20s = 3.2 B/s
- Events: ~16 B × 0.1 Hz avg = 1.6 B/s
- Block overhead: (32 B header + 4 B CRC) / 4096 B = 0.9% = ~5.5 B/s
- **Total: ~610 B/s**

**Capacity**: 2 MB ÷ 610 B/s = **3,440 seconds ≈ 57 minutes**
With optimization: **~63 minutes** rolling window (excellent for typical 20-45 min flights + margin for crash analysis).

## Integration Steps
1) **Partition setup**:
   - Create `partitions.csv` in project root (see Flash Layout section)
   - Add `board_build.partitions = partitions.csv` to `platformio.ini`
   - Document OTA-safe upgrade path (NVS location unchanged)

2) **Data structures**:
   - Define packed structs for fast/slow/cell/event frames with `#pragma pack(push,1)`
   - Define block header and superblock structs
   - Implement CRC32 calculation helpers

3) **Runtime logging**:
   - Add `wattHoursUsed` integration in `bmsTask` (power × time accumulation)
   - Create `blackboxQueue` (256 frames, droppable) and writer task
   - Decimate `throttleTask` to 20 Hz logging (counter: log every 3rd call)
   - Sample `bmsTask` at 1 Hz for slow frames
   - Add cell voltage differential monitor for snapshots
   - Register `BlackBoxLogger` sink into `multiLogger` for automatic event capture

4) **Storage manager** (`BlackBoxManager` class):
   - `init()`: Locate partition via `esp_partition_find_first()`, scan for head/tail
   - `eraseSector()`, `writeSector()`, `readSector()` wrappers
   - Ring advancement logic with superblock updates
   - Pre-erase next block asynchronously to avoid task delays

5) **Serial offload**:
   - Extend `parse_serial_commands()` with `bb:*` commands
   - Implement chunked read with CRC validation
   - Add flow control (ACK/NACK per 4KB window)

6) **Testing**:
   - Max-rate logging stress test (verify no frame drops in normal flight)
   - Induced brownout during write (verify block recovery)
   - Ring wrap-around after 63+ minutes
   - Offload resume on USB disconnect/reconnect
   - Wear distribution analysis (verify even erase counts)

7) **Host tools**:
   - Python script: `blackbox_pull.py` (manifest → bulk read → verify CRC)
   - Python script: `blackbox_decode.py` (parse binary → CSV/JSON output)
   - Optional: Web-based visualization (altitude/throttle/temps over time)

## Migration Notes (for existing users)
- **OTA updates**: Settings preserved automatically (NVS location unchanged) ✅
- **USB flash updates**: Settings will reset unless backed up first
- **Recommended flow**:
  1. Add `{"command":"backup_settings"}` to return current config JSON
  2. User saves JSON before flashing
  3. Add `{"command":"restore_settings", "settings":{...}}` to restore post-flash
  4. Document process in release notes
