# Flight Data "Black Box" Plan (ESP32-S3FN8)

**Hardware**: M5Stack Stamp-S3A (ESP32-S3FN8: 8 MB Flash, NO PSRAM, 512 KB SRAM)
**Goal**: Capture 60+ minute flights with rolling storage, survive power loss, and offload over USB serial without affecting real-time control.

## Signals and Rates (state-based logging)
- **Fast path** (50 Hz actual in `throttleTask`):
  - **ARMED/CRUISING**: Decimate to **10 Hz** for flight data (100ms resolution)
  - **DISARMED**: Decimate to **1 Hz** for diagnostics/boot monitoring
  - Fields: timestamp64, device state, performance_mode, throttle PWM (final), ESC volts/amps/eRPM, wattHoursUsed, altitude + vertical speed (if baro valid).
- **Slow path** (10 Hz in `bmsTask`, **sample at 1 Hz** for logging): pack V/I/P, SOC, highest/lowest cell V, delta V, max temps (MOS/MCU/CAP/Motor + BMS MOS/balance/T1–T4), charge/discharge MOS flags, battery cycle, ESC/BMS connection state.
- **Cell detail** (every 20 s or when `voltage_differential` changes > 10 mV from last snapshot): full 24-cell table as uint16 mV.
- **Events** (edge-triggered via MultiLogger sink): state transitions (DISARMED/ARMED/CRUISING), ESC/BMS connect/disconnect, running_error/selfcheck_error changes, MOS open/close, altitude zero reset on arm, cruise activation/deactivation with PWM value, performance mode changes, button sequences.

## Record Types (fixed, quantized)

**All frames start with type byte for unambiguous parsing:**

- **Fast frame** (type=0x01) **28 B**:
  ```cpp
  struct BlackboxFastFrame {
    uint8_t type;               // 1 B - 0x01
    uint8_t state_mode;         // 1 B - state (3 bits) | mode (1 bit) | flags (4 bits)
    int64_t timestamp_us;       // 8 B - esp_timer_get_time()
    uint16_t throttle_pwm;      // 2 B - ESC PWM microseconds
    uint16_t v_pack_centivolts; // 2 B - voltage * 100
    int16_t i_pack_deciamps;    // 2 B - current * 10 (signed for regen)
    uint16_t erpm_hundreds;     // 2 B - eRPM / 100
    uint16_t wh_centiwatthours; // 2 B - wattHoursUsed * 100
    int16_t alt_decimeters;     // 2 B - relative altitude * 10
    int16_t vsi_cm_per_sec;     // 2 B - vertical speed * 100
    uint8_t esc_temp_max;       // 1 B - max of MOS/MCU/CAP/Motor temps
    uint8_t bms_temp_max;       // 1 B - max BMS temp
    uint8_t padding;            // 1 B - align to 28 B
  };
  ```

- **Slow frame** (type=0x02) **36 B**:
  ```cpp
  struct BlackboxSlowFrame {
    uint8_t type;               // 1 B - 0x02
    uint8_t flags;              // 1 B - charge_mos, discharge_mos, bms_conn, esc_conn
    int64_t timestamp_us;       // 8 B
    uint16_t v_cell_hi_mv;      // 2 B - highest cell mV
    uint16_t v_cell_lo_mv;      // 2 B - lowest cell mV
    uint16_t v_cell_diff_mv;    // 2 B - differential mV
    uint8_t soc_half_percent;   // 1 B - SOC * 2 (0-200 = 0-100%)
    uint8_t esc_mos_temp;       // 1 B - ESC MOS temp
    uint8_t esc_mcu_temp;       // 1 B - ESC MCU temp
    uint8_t esc_cap_temp;       // 1 B - ESC CAP temp
    uint8_t motor_temp;         // 1 B - Motor temp
    uint8_t bms_mos_temp;       // 1 B - BMS MOS temp
    uint8_t bms_balance_temp;   // 1 B - BMS balance temp
    uint8_t bms_t1_temp;        // 1 B - BMS T1 temp
    uint8_t bms_t2_temp;        // 1 B - BMS T2 temp
    uint8_t bms_t3_temp;        // 1 B - BMS T3 temp
    uint8_t bms_t4_temp;        // 1 B - BMS T4 temp
    uint32_t battery_cycle;     // 4 B - cycle count
    uint16_t running_error;     // 2 B - ESC running error bitmask
    uint16_t selfcheck_error;   // 2 B - ESC selfcheck error bitmask
  };
  ```

- **Cell snapshot** (type=0x03) **60 B**:
  ```cpp
  struct BlackboxCellFrame {
    uint8_t type;               // 1 B - 0x03
    uint8_t cell_count;         // 1 B - number of valid cells (usually 24)
    int64_t timestamp_us;       // 8 B
    uint16_t cells_mv[24];      // 48 B - cell voltages in mV
    uint8_t min_idx;            // 1 B - index of lowest cell
    uint8_t max_idx;            // 1 B - index of highest cell
  };
  ```

- **Event frame** (type=0x04) **16 B**:
  ```cpp
  struct BlackboxEventFrame {
    uint8_t type;               // 1 B - 0x04
    uint8_t event_id;           // 1 B - SensorID or special event code
    int64_t timestamp_us;       // 8 B
    uint8_t alert_level;        // 1 B - AlertLevel enum
    uint8_t reserved;           // 1 B - padding
    int32_t payload;            // 4 B - value (float*10 or int or bitfield)
  };
  ```

**Special Event IDs** (0xE0-0xFF range, above SensorID enum):
- `0xE0`: Flight start (ARM transition) - payload = flight_session_id
- `0xE1`: Flight end (DISARM transition) - payload = duration_sec
- `0xE2`: Cruise start - payload = cruise_pwm
- `0xE3`: Cruise end - payload = 0
- `0xE4`: Performance mode change - payload = new_mode
- `0xE5`: Altitude zeroed - payload = 0
- `0xE6`: User bookmark - payload = bookmark_id

_Note: Using `esp_timer_get_time()` (int64_t µs) for timestamps. Provides ~292,000 years before overflow._

## Flash Layout (using existing SPIFFS partition - NO custom partitions.csv)

**Using Default 8 MB Partition** (from `default_8MB.csv`):
```csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x330000,
app1,     app,  ota_1,   0x340000,0x330000,
spiffs,   data, spiffs,  0x670000,0x180000,
coredump, data, coredump,0x7F0000,0x10000,
```

**Strategy**: Write blackbox data as raw binary files in **existing SPIFFS partition** (1.5 MB).

**Advantages**:
- ✅ **No partition table changes** - existing users can OTA update safely
- ✅ **No settings reset** - NVS untouched
- ✅ **Simpler deployment** - works with current flash layout
- ✅ **Debuggable** - can mount SPIFFS and inspect files

**SPIFFS Layout**:
```
/spiffs/
├── blackbox/
│   ├── super.bin      (4 KB - superblock with ring state)
│   ├── block_000.bin  (4 KB - data block)
│   ├── block_001.bin  (4 KB - data block)
│   └── ...            (up to ~370 blocks)
└── (other assets if any)
```

**Capacity**: 1.5 MB SPIFFS - ~10% filesystem overhead = **~1.35 MB usable**
- At 610 B/s armed = ~37 minutes continuous flight
- With state-based logging (20 Hz armed / 1 Hz disarmed) = **45+ minutes flight + hours of ground time**

**Access API**:
```cpp
#include <SPIFFS.h>

bool initBlackBox() {
  if (!SPIFFS.begin(true)) {  // true = format if mount fails
    return false;
  }
  SPIFFS.mkdir("/blackbox");
  return true;
}
```

**Block Structure** (same format, stored as files):
- 4 KB erase blocks (ESP32-S3 flash sector size)
- Block header (32 B, at start of each block):
  ```cpp
  struct BlackboxBlockHeader {
    uint32_t magic;             // 4 B - 0xBB0X0001 ("BlackBox v1")
    uint32_t block_seq;         // 4 B - monotonic sequence number
    uint16_t flight_session_id; // 2 B - increments on each ARM
    uint8_t state;              // 1 B - 0=empty, 1=writing, 2=closed
    uint8_t record_count;       // 1 B - number of frames in block (max ~140)
    int64_t first_timestamp;    // 8 B - timestamp of first frame
    int64_t last_timestamp;     // 8 B - timestamp of last frame
    uint32_t crc32;             // 4 B - CRC of header + all frames
  };
  ```
- **Usable space per block**: 4096 - 32 = 4064 bytes for frames
- **Block filling**: Frames written sequentially. When next frame won't fit, pad remainder with 0xFF, compute CRC, mark block closed, advance to next block.

**Superblock Structure** (blocks 0 and 1, alternating writes):
```cpp
struct BlackboxSuperblock {
  uint32_t magic;             // 4 B - 0xBB0XFFFE
  uint32_t version;           // 4 B - format version
  uint32_t write_count;       // 4 B - increments each superblock write (for freshness)
  uint32_t head_block;        // 4 B - next block to write
  uint32_t tail_block;        // 4 B - oldest valid block
  uint16_t current_flight_id; // 2 B - current/last flight session
  uint16_t total_flights;     // 2 B - total flights recorded
  uint32_t total_erases;      // 4 B - cumulative erase count (wear tracking)
  uint32_t crc32;             // 4 B - CRC of superblock
  uint8_t reserved[4064];     // Pad to 4KB
};
```
- On boot: Read both superblocks, use one with higher `write_count` and valid CRC
- On block close: Write to alternate superblock (0→1→0→1...)
- Ring wrap: When head catches tail, erase tail block, increment tail, then write head

**File Operations** (instead of raw partition access):
```cpp
// Write a block
File f = SPIFFS.open("/blackbox/block_042.bin", "w");
f.write(stagingBuffer, 4096);
f.close();

// Read a block
File f = SPIFFS.open("/blackbox/block_042.bin", "r");
f.read(buffer, 4096);
f.close();

// Delete oldest block (ring wrap)
SPIFFS.remove("/blackbox/block_000.bin");
```

**Trade-offs vs Raw Partition**:
- ❌ Slightly slower writes (filesystem overhead)
- ❌ ~10% space overhead for filesystem metadata
- ❌ Less predictable timing (wear leveling)
- ✅ Much simpler deployment
- ✅ Can inspect files for debugging
- ✅ SPIFFS handles wear leveling automatically

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
4. `bmsTask()`: Add 1Hz sampling + `pushBlackboxSlowFrame()` + cell snapshot trigger
5. `changeDeviceState()`: Emit flight start/end events on ARM/DISARM

**Code Changes Required** (issues found in codebase review):

1. **Track final throttle PWM** - Currently `finalPwm` is local to `handleThrottle()`. Add:
   ```cpp
   // In globals.h
   extern volatile uint16_t lastThrottlePwm;

   // In sp140.ino handleThrottle(), after setESCThrottle(finalPwm):
   lastThrottlePwm = finalPwm;
   ```

2. **Implement wattHoursUsed calculation** - Currently declared but never updated:
   ```cpp
   // In bmsTask(), add after battery data update:
   static uint32_t lastWattHourUpdate = 0;
   uint32_t now = millis();
   if (lastWattHourUpdate > 0 && unifiedBatteryData.power > 0) {
     float hours = (now - lastWattHourUpdate) / 3600000.0f;
     wattHoursUsed += unifiedBatteryData.power * 1000.0f * hours;  // kW to W
   }
   lastWattHourUpdate = now;
   ```

3. **Cell snapshot trigger** - Add to `bmsTask()`:
   ```cpp
   static float lastLoggedDelta = 0;
   if (abs(bmsTelemetryData.voltage_differential - lastLoggedDelta) > 0.010f) {
     pushBlackboxCellFrame(bmsTelemetryData);
     lastLoggedDelta = bmsTelemetryData.voltage_differential;
   }
   ```

4. **Flight boundary events** - Add to `changeDeviceState()`:
   ```cpp
   if (newState == ARMED && oldState == DISARMED) {
     emitBlackboxEvent(0xE0, currentFlightId);  // Flight start
   } else if (newState == DISARMED && oldState != DISARMED) {
     uint32_t duration = (millis() - armedAtMillis) / 1000;
     emitBlackboxEvent(0xE1, duration);  // Flight end
   }
   ```

**RAM Usage Estimate**:
- Fast queue: 256 × 28 B = 7,168 B
- Slow queue: 16 × 36 B = 576 B
- Event queue: 32 × 16 B = 512 B
- Staging buffer: 4,096 B
- BlackBox state: ~100 B
- **Total: ~12.5 KB** (of ~320 KB available heap - OK ✅)

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
    // Log at 10 Hz during flight (every 5th sample @ 50 Hz)
    if (++blackboxCounter >= 5) {
      blackboxCounter = 0;
      pushBlackboxFastFrame();
    }
  }
  ```
- **Slow frame builder**: Runs in `bmsTask` at 10 Hz; **samples at 1 Hz** for logging; pushes non-droppable slow frame.
- **Cell snapshot builder**: Triggered in `bmsTask` when `abs(voltage_differential - lastLoggedDelta) > 0.010f` (10 mV change in max-min cell spread). Uses existing `bmsTelemetryData.cell_voltages[]` array.
- **wattHoursUsed integration**: Calculate in `bmsTask` using time-delta accumulation. Reset to 0 on DISARM.
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

**Boot Recovery**:
1. Read superblock 0 and 1, verify CRC on each
2. Use superblock with higher `write_count` and valid CRC
3. If both invalid, scan all blocks for valid headers, rebuild ring state
4. Start writing from `head_block` position

**Power Loss Handling**:
- Closed blocks: CRC-verified, always recoverable
- Writing block: May be partially written, detected by:
  - `state == 1` (writing) in header
  - CRC mismatch
  - Frame with invalid type byte (0xFF from unprogrammed flash)
- Recovery: Discard writing block, rewind to last closed block

**Wear Leveling**:
- SPIFFS handles wear leveling automatically (built-in)
- Track `total_writes` in superblock for monitoring
- Expected life: SPIFFS distributes writes across partition
- No manual wear management needed

**CRC Implementation**: Use ESP32 ROM CRC32 for speed:
```cpp
#include "esp_rom_crc.h"
uint32_t crc = esp_rom_crc32_le(0, data, length);
```

**SPIFFS Considerations**:
- Mount with `SPIFFS.begin(true)` to auto-format if corrupted
- Check `SPIFFS.totalBytes()` and `SPIFFS.usedBytes()` for capacity monitoring
- Use `SPIFFS.gc()` periodically to reclaim space from deleted files

## Capacity Check (1.35 MB in SPIFFS, 69+ min flight window)

**Data rates (state-based)**:

**When ARMED** (10 Hz resolution):
- Fast frames: 28 B × 10 Hz = 280 B/s
- Slow frames: 36 B × 1 Hz = 36 B/s
- Cell snapshots: ~60 B every 20s = 3 B/s
- Events: ~16 B × 0.1 Hz avg = 1.6 B/s
- Block overhead: ~5 B/s
- **Total while armed: ~326 B/s**

**When DISARMED** (minimal logging):
- Fast frames: 28 B × 1 Hz = 28 B/s
- Slow frames: 36 B × 1 Hz = 36 B/s
- Cell snapshots: ~3 B/s
- Events: ~1.6 B/s (rare when disarmed)
- Block overhead: ~1 B/s
- **Total while disarmed: ~70 B/s**

**Capacity with 1.35 MB SPIFFS**:
1. **Continuous armed flight**: 1.35 MB ÷ 326 B/s = **69 minutes** of pure flight data
2. **Typical single flight** (45 min armed + 30 min ground):
   - Armed: 45 min × 326 B/s = 880 KB
   - Disarmed: 30 min × 70 B/s = 126 KB
   - Total: 1,006 KB → **Fits with 350 KB margin**
3. **Multiple flights** (with rolling overwrite):
   - 2× 30-minute flights + ground time = **60+ minutes total flight data**
   - Oldest data rolls off as new data comes in
4. **Long endurance flight**: 60+ minutes continuous → fully captured ✅

**This is sufficient for**:
- ✅ Long paramotor flights (up to 69 min continuous)
- ✅ Full crash analysis at 100ms resolution (excellent for flight dynamics)
- ✅ Multiple flight sessions in a day (rolling buffer)
- ✅ Typical use case: entire flying session captured

**10 Hz Resolution Context**:
- Aviation flight data recorders typically use 1-4 Hz
- 100ms captures all meaningful flight dynamics
- Sufficient to analyze: throttle response, altitude changes, temperature trends, crash sequences

## Integration Steps
1) **SPIFFS setup** (no partition changes needed!):
   - Add `SPIFFS` to lib_deps if not present (usually built-in)
   - Call `SPIFFS.begin(true)` in `setup()` before `initBlackBox()`
   - Create `/blackbox/` directory on first run

2) **Data structures**:
   - Define packed structs for fast/slow/cell/event frames with `#pragma pack(push,1)`
   - Define block header and superblock structs
   - Implement CRC32 calculation helpers

3) **Runtime logging**:
   - Add `wattHoursUsed` integration in `bmsTask` (power × time accumulation)
   - Create `blackboxQueue` (256 frames, droppable) and writer task
   - Implement **state-based decimation** in `throttleTask`:
     - **10 Hz** (every 5th sample @ 50 Hz) when ARMED/CRUISING
     - **1 Hz** (every 50th sample) when DISARMED
   - Sample `bmsTask` at 1 Hz for slow frames
   - Add cell voltage differential monitor for snapshots
   - Register `BlackBoxLogger` sink into `multiLogger` for automatic event capture

4) **Storage manager** (internal to `blackbox.cpp`):
   - Initialize SPIFFS: `SPIFFS.begin(true)` with auto-format
   - Create `/blackbox/` directory if not exists
   - Maintain ring state in `/blackbox/super.bin`
   - Block files: `/blackbox/block_NNN.bin` (NNN = 000-369)
   - Ring management: Track head/tail indices, delete oldest when full
   - Flight detection: Insert boundary markers on ARM/DISARM transitions
   - **No pre-erase needed** - SPIFFS handles this internally

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
- **OTA updates**: Settings preserved automatically ✅
- **USB flash updates**: Settings preserved (same partition layout) ✅
- **SPIFFS data**: May be cleared on first boot if format needed, but this only affects blackbox data (not settings)
- **No backup/restore needed** - using existing partition table means seamless upgrades!

## Future Expansion
If more capacity is needed later, can migrate to custom partition:
- Create `partitions.csv` with dedicated 2-3 MB blackbox partition
- Announce as major version update with migration instructions
- Keep SPIFFS approach as fallback/compatibility mode
