# Flight Data "Black Box" Plan (ESP32-S3FN8)

**Hardware**: M5Stack Stamp-S3A (ESP32-S3FN8: 8 MB Flash, NO PSRAM, 512 KB SRAM)
**Goal**: Capture 60+ minute flights with rolling storage, survive power loss, and offload over USB serial without affecting real-time control.

## Signals and Rates (state-based logging)
- **Fast path** (50 Hz actual in `throttleTask`):
  - **ARMED/CRUISING**: Decimate to **20 Hz** for high-resolution flight data
  - **DISARMED**: Decimate to **1 Hz** for diagnostics/boot monitoring
  - Fields: timestamp64, device state, performance_mode, throttle PWM (final), ESC volts/amps/eRPM, wattHoursUsed, altitude + vertical speed (if baro valid).
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

## Code Architecture (async, flight-safe)

**Module Structure**: `inc/sp140/blackbox.h` + `src/sp140/blackbox.cpp`

**Task Priority & Core Assignment**:
```
Priority 3 (Critical):  throttleTask, watchdogTask         [Core 0]
Priority 2 (Important): uiTask, bmsTask, vibeTask          [Core 1]
Priority 1 (Background): BLE, monitoring, audio, webSerial, blackboxWriter [Core 1]
```

**Queues** (async producer/consumer pattern):
- `blackboxFastQueue`: 256 frames (droppable, non-blocking send)
- `blackboxSlowQueue`: 16 frames (important, 10ms timeout)
- `blackboxEventQueue`: 32 frames (critical, 10ms timeout)

**Producer Pattern** (similar to existing `pushTelemetrySnapshot()`):
- Called from `throttleTask` and `bmsTask` with state-based decimation
- Non-blocking `xQueueSend(..., 0)` for fast frames → drops if full, **never blocks flight control**
- Short timeout for slow/event frames (important but not real-time critical)

**Consumer Pattern** (similar to `updateBLETask`):
- `blackboxWriterTask()` runs at priority 1 on core 1 (background)
- Drains queues: Events > Slow > Fast (priority order)
- Accumulates into 4KB staging buffer
- Writes to flash when buffer full, pre-erases next block asynchronously
- Yields frequently (`vTaskDelay`) to avoid starving other background tasks

**Event Capture** (via MultiLogger sink):
- `BlackBoxEventLogger : ILogger` registered into existing `multiLogger`
- Automatically captures all 70+ sensor alert transitions
- No manual hooks needed—leverages existing monitoring infrastructure

**Integration Points**:
1. `setup()`: Call `initBlackBox()` after monitoring system
2. `setupTasks()`: Create `blackboxWriterTask()` with priority 1, core 1
3. `throttleTask()`: Add decimation counter + `pushBlackboxFastFrame()`
4. `bmsTask()`: Add 1Hz sampling + `pushBlackboxSlowFrame()`

## Runtime Pipeline
- **Producers**: State-based decimation in flight-critical tasks; async queue push with backpressure protection
- **Fast frame builder**: Runs inside `throttleTask` at 50 Hz with **state-based decimation**:
  ```cpp
  static uint8_t blackboxCounter = 0;
  if (currentState == DISARMED) {
    // Log at 1 Hz when disarmed (every 50th sample)
    if (++blackboxCounter >= 50) {
      blackboxCounter = 0;
      pushBlackboxFastFrame();
    }
  } else {  // ARMED or ARMED_CRUISING
    // Log at 20 Hz during flight (every 3rd sample)
    if (++blackboxCounter >= 3) {
      blackboxCounter = 0;
      pushBlackboxFastFrame();
    }
  }
  ```
- **Slow frame builder**: Runs in `bmsTask` at 10 Hz; **samples at 1 Hz** for logging; pushes non-droppable slow frame.
- **Cell snapshot builder**: Triggered when `abs(voltage_differential - lastLoggedDelta) > 0.010f` (10 mV change in max-min cell spread).
- **wattHoursUsed integration**: Calculate in `bmsTask` using `wattHoursUsed += unifiedBatteryData.power * 1000.0f * (dt_hours)` for fast frame logging.
- **Event hook**: Register `BlackBoxLogger : ILogger` sink into existing `multiLogger` to capture all 70+ sensor events automatically on alert edges.
- **Writer task** (low prio, core 1): Drains queues into 4 KB staging buffer in **SRAM** (no PSRAM on FN8 variant). Pre-erases next block asynchronously; writes current block, then marks header closed with CRC32.
- **Backpressure**: If queue near full (>90%), drop only fast frames; never drop slow, cell, or event frames.

## Serial Offload Protocol (USB CDC, DISARMED gate)

**Integration**: Add `handleBlackboxCommand()` to existing `parse_serial_commands()` in `webSerialTask`

**Commands** (JSON over USB):

### 1. **Manifest** - Get ring buffer status
```json
{"bb": "manifest"}
```
**Response**:
```json
{
  "bb": "manifest",
  "ok": true,
  "partition_size": 2097152,
  "block_size": 4096,
  "total_blocks": 512,
  "used_blocks": 234,
  "head_block": 456,
  "tail_block": 222,
  "flights": [
    {"id": 1, "start_block": 222, "end_block": 300, "duration_sec": 1420},
    {"id": 2, "start_block": 301, "end_block": 455, "duration_sec": 1680}
  ]
}
```

### 2. **Read Block** - Stream block data
```json
{"bb": "read", "block": 222, "chunk_size": 256}
```
**Response** (binary chunks with JSON wrapper):
```json
{"bb":"read_start","block":222,"total_chunks":16}
// ... binary chunk 1 (256 bytes) ...
{"bb":"read_chunk","block":222,"chunk":1,"crc32":"0xABCD1234"}
// ... binary chunk 2 (256 bytes) ...
{"bb":"read_chunk","block":222,"chunk":2,"crc32":"0x5678EFAB"}
// ... continues for all 16 chunks ...
{"bb":"read_done","block":222}
```

### 3. **Read Range** - Smart bulk download
```json
{"bb": "read_range", "start_block": 222, "end_block": 455}
```
Downloads multiple blocks sequentially with ACK between blocks for flow control.

### 4. **Stats** - Quick diagnostics
```json
{"bb": "stats"}
```
**Response**:
```json
{
  "bb": "stats",
  "ok": true,
  "queue_fast_used": 12,
  "queue_slow_used": 2,
  "queue_event_used": 0,
  "frames_dropped_fast": 234,
  "frames_logged_total": 125456,
  "current_block": 456,
  "bytes_written_total": 1957888,
  "uptime_sec": 12456
}
```

### 5. **Erase** - Clear ring buffer
```json
{"bb": "erase"}
```
**Response** (requires confirmation):
```json
{"bb": "erase", "confirm_required": true, "token": "A3F9B2"}
```
**Confirm**:
```json
{"bb": "erase_confirm", "token": "A3F9B2"}
```
**Response**:
```json
{"bb": "erase_confirm", "ok": true, "blocks_erased": 512}
```

### 6. **Bookmark** - Mark flight boundary
```json
{"bb": "bookmark", "name": "Flight 3 - Sunset Ridge"}
```
Inserts special event frame to denote user-marked boundary.

### 7. **Pause/Resume** - Control logging
```json
{"bb": "pause"}
{"bb": "resume"}
```
Useful for ground testing without filling buffer.

**Flow Control Strategy**:
- **Block-level ACK**: After each block downloaded, wait for `{"ack": true}` from host
- **Timeout**: 5s per block; if no ACK, resend last block
- **Resume**: Track last successful block in manifest, resume from there
- **Throttling**: Add 10ms delay between chunks to avoid USB buffer overflow

**Safety Gates**:
- All commands **require DISARMED state** (checked in `webSerialTask`)
- Erase requires two-step confirmation with random token
- Read commands never interfere with writing (read from partition directly)

**Python Host Tools**:

### `blackbox_pull.py` - Download tool
```bash
# Pull all flight data
python blackbox_pull.py --port /dev/ttyACM0 --output flights.bb

# Pull specific range
python blackbox_pull.py --port /dev/ttyACM0 --blocks 222-455 --output flight2.bb

# Check stats only
python blackbox_pull.py --port /dev/ttyACM0 --stats
```

### `blackbox_decode.py` - Parser/analyzer
```bash
# Decode to CSV
python blackbox_decode.py flights.bb --format csv --output flights.csv

# Decode to JSON
python blackbox_decode.py flights.bb --format json --output flights.json

# Generate summary
python blackbox_decode.py flights.bb --summary
# Output: Flight 1: 23:42 duration, max alt 450m, max throttle 87%, 3 warnings

# Filter by time range
python blackbox_decode.py flights.bb --start 12:30:00 --end 12:45:00 --output segment.csv
```

### `blackbox_plot.py` - Visualization
```bash
# Generate plots
python blackbox_plot.py flights.csv --output flight_analysis.html
# Creates interactive plots: altitude, throttle, temps, voltage, current over time
```

**Protocol Advantages**:
- ✅ **Resume-friendly**: Can restart download mid-flight if USB disconnects
- ✅ **Efficient**: Only downloads used blocks, not empty space
- ✅ **Safe**: DISARMED gate prevents mid-flight interference
- ✅ **Debuggable**: JSON responses easy to inspect with serial monitor
- ✅ **Flexible**: Can pull entire ring or specific flights

## Recovery and Wear
- On boot: scan from superblock head forward until first empty/invalid; rebuild head/tail; latest valid superblock wins.
- Power loss: at most lose current writing block; closed blocks are CRC-checked.
- Wear leveling: rotate blocks evenly; count erases in RAM per flight, persist occasionally.

## Capacity Check (2 MB partition, 2+ hour flight window)

**Data rates (state-based)**:

**When ARMED** (full resolution):
- Fast frames: 28 B × 20 Hz = 560 B/s
- Slow frames: 40 B × 1 Hz = 40 B/s
- Cell snapshots: ~64 B every 20s = 3.2 B/s
- Events: ~16 B × 0.1 Hz avg = 1.6 B/s
- Block overhead: ~5.5 B/s
- **Total while armed: ~610 B/s**

**When DISARMED** (minimal logging):
- Fast frames: 28 B × 1 Hz = 28 B/s
- Slow frames: 40 B × 1 Hz = 40 B/s
- Cell snapshots: ~3.2 B/s
- Events: ~1.6 B/s (rare when disarmed)
- Block overhead: ~1 B/s
- **Total while disarmed: ~74 B/s**

**Blended Capacity Examples**:
1. **Continuous armed flight**: 2 MB ÷ 610 B/s = **57 minutes** of pure flight data
2. **Typical usage** (30 min armed + 90 min disarmed):
   - Armed: 30 min × 610 B/s = 1,098 KB
   - Disarmed: 90 min × 74 B/s = 400 KB
   - Total: 1,498 KB → **Leaves 500 KB for additional flights**
   - **Can store 2+ hours of mixed usage** (multiple flights + ground time)
3. **Multiple short flights**: 5× 20-minute flights + ground time between = **2+ hours total**

This state-based approach provides excellent coverage for typical paramotor operations while maintaining full resolution where it matters most—during actual flight.

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
   - Implement **state-based decimation** in `throttleTask`:
     - 20 Hz (every 3rd sample) when ARMED/CRUISING
     - 1 Hz (every 50th sample) when DISARMED
   - Sample `bmsTask` at 1 Hz for slow frames
   - Add cell voltage differential monitor for snapshots
   - Register `BlackBoxLogger` sink into `multiLogger` for automatic event capture

4) **Storage manager** (internal to `blackbox.cpp`):
   - Locate partition: `esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "blackbox")`
   - Maintain ring state: head/tail block indices, current write offset
   - Superblock management: Dual copies (blocks 0 & 1) with alternating updates
   - Pre-erase next block asynchronously to avoid blocking writer task
   - Block operations: `readBlock()`, `writeBlock()`, `eraseBlock()`
   - Flight detection: Insert boundary markers on ARM/DISARM transitions

5) **Serial offload** (extend `parse_serial_commands()`):
   - Add `handleBlackboxCommand(doc)` for `{"bb": ...}` commands
   - Implement manifest generation (scan superblocks, build flight list)
   - Chunked block reader with CRC32 per chunk (256B chunks × 16 = 4KB)
   - Flow control: Block-level ACK with 5s timeout, resume support
   - Safety: All commands gated on `currentState == DISARMED`
   - Erase: Two-step confirmation with random token

6) **Testing**:
   - Max-rate logging stress test (verify no frame drops in normal flight)
   - Induced brownout during write (verify block recovery)
   - Ring wrap-around after 63+ minutes
   - Offload resume on USB disconnect/reconnect
   - Wear distribution analysis (verify even erase counts)

7) **Host tools** (Python, in `scripts/` directory):
   - `blackbox_pull.py`: Download via USB CDC (manifest → block range → verify CRC → save .bb file)
   - `blackbox_decode.py`: Parse binary .bb file → CSV/JSON with timestamp alignment
   - `blackbox_plot.py`: Generate interactive HTML plots (Plotly) for flight analysis
   - `blackbox_stats.py`: Quick summary (duration, max alt, warnings, battery usage)
   - All tools support resume on disconnect, progress bars, error recovery

## Migration Notes (for existing users)
- **OTA updates**: Settings preserved automatically (NVS location unchanged) ✅
- **USB flash updates**: Settings will reset unless backed up first
- **Recommended flow**:
  1. Add `{"command":"backup_settings"}` to return current config JSON
  2. User saves JSON before flashing
  3. Add `{"command":"restore_settings", "settings":{...}}` to restore post-flash
  4. Document process in release notes
