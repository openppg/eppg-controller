# Inside the OpenPPG Controller — 6-Part Content Series

Ready-to-record short-form video scripts + companion blog posts about the SP140
controller firmware, written for a paramotor-pilot audience (EV / aviation /
tech enthusiasts — mostly *not* engineers, but curious about the details).

Narrator voice: **Zach Whitehead, technical co-founder.** Each video targets
**~2 minutes** (spoken voiceover ≈ 300–330 words). Every file contains:

- a **video script** (hook → timed beat sheet with shot + on-screen-text cues → CTA)
- a **clean full voiceover** read (with exact word count) you can record straight through
- a **B-roll / shot list** and **on-screen text** list
- a **companion blog post** (~800–1,100 words, standalone, with an optional
  "technical bits for the nerds" section)
- a **ready-to-paste social caption + hashtags**

## The episodes

| # | File | Title | Spoken words | The hook |
|---|------|-------|:---:|----------|
| 1 | [01-what-is-an-rtos.md](01-what-is-an-rtos.md) | What's an RTOS? Why Your Throttle Runs 13 Jobs at Once | 312 | How the throttle stays instant while the screen redraws and Bluetooth streams |
| 2 | [02-why-esp-idf.md](02-why-esp-idf.md) | Why We Build on ESP-IDF (and What It Unlocks for Pilots) | 304 | The one build-line change that made every v8.0 reliability feature possible |
| 3 | [03-bluetooth-ota-updates.md](03-bluetooth-ota-updates.md) | Updating Your Paramotor Over the Air — No Cable, No Risk | 330 | Your phone updates itself overnight; now your throttle does too — safely |
| 4 | [04-reliability-hardening.md](04-reliability-hardening.md) | How the Controller Survives a Crash — The Software Kind | 316 | What happens if the software in your throttle freezes mid-flight? |
| 5 | [05-controller-evolution.md](05-controller-evolution.md) | From Arduino Nano to ESP32: The Story of Our Controller | 330 | An 8-bit hobby chip → a dual-core brain you trust in flight, in 8 years |
| 6 | [06-can-bus.md](06-can-bus.md) | CAN Bus: How Your Throttle, Motor, and Battery Talk to Each Other | 315 | Your throttle used to shout one dumb number; now it has a real conversation |

**Suggested posting order:** 1 → 2 → 3 → 4 → 5 → 6. They cross-reference each
other (Ep 2 points back to Ep 1's RTOS; Ep 3 hands off to Ep 4's rollback; Ep 5
flags the CAN milestone and teases Ep 6; Ep 6 nods back to the EICAS alerts in
Ep 4), but each also stands alone. Episode 5 (the origin story) and Episode 6
(CAN, "the digital nervous system") make a natural two-part finale — and either
is a strong evergreen / pinned piece.

## How these were built (accuracy notes)

Every technical claim is grounded in **(a) the firmware source in this repo**
(verified at `master` / v8.0-rc1, with file/commit citations gathered during
research) and **(b) the internal knowledge-base wiki** at
`~/Dev/repos/knowledge-base/wiki`. Web research filled in general background
(what FreeRTOS / BLE / OTA / CAN are, MCU specs) and the pilot-facing analogies.

Key facts the scripts are careful to get right:

- **The RTOS story starts on the RP2040, not the ESP32.** FreeRTOS was adopted in
  firmware **v6.0 (Aug 2023)** on the RP2040; the ESP32-S3 firmware inherited and
  expanded it. (Episodes 1 and 5 both say this explicitly.)
- **Processor timeline:** Arduino Nano (2018, X4 kit) → SAMD21 / Feather M0
  (late 2018, SP140 V1; first public SP140 v5.1 in 2021) → RP2040 (2022, SP140 V2;
  FreeRTOS in v6.0/2023) → ESP32-S3 / M5Stack StampS3 (v7.0, May 2025, SP140 V2.5)
  → v8.0 (2026, "Cloud OTA").
- **v8.0 = the hybrid `framework = arduino, espidf` build**, which unlocks the
  native OTA APIs, crash core-dumps, watchdog/panic config, A/B partitions, and
  automatic rollback.
- **Cloud OTA pipeline:** `ota.openppg.com` (Cloudflare) → phone app downloads
  over the internet → pushes to the controller over Bluetooth. The controller
  **never touches the internet itself.** Staged rollout + emergency-pause
  kill-switch; publish-time validation via the embedded `OPPGFWV1` marker + SHA-256;
  in-device BLE OTA uses A/B partitions, CRC16 per chunk, armed-state hard block,
  idle timeout.
- **CAN bus (Episode 6, milestone in Episode 5):** arrived with v7.0 (2025, ESP32-S3).
  The ESC rides the chip's **built-in CAN controller (Espressif's "TWAI" — just
  their name for standard CAN)**; the BMS rides a separate **MCP2515** CAN chip.
  It's **two-way** — the controller reads telemetry *and* writes to the ESC
  (throttle via `setThrottleSettings2` at 0.1 µs resolution, startup config, status
  LED colors, beep tones). Safety nuance: the ESC expects a throttle message
  ~every 300 ms or it stops, so the CAN heartbeat is itself a safety feature.
  Libraries `SINE-ESC-CAN` and `ANT-BMS-CAN` are open source. (Comms evolution:
  analog → UART telemetry (v5.8) → CAN (v7.0).)
- **EICAS-style alerts:** 40+ live signals (ESC, BMS, internal), yellow/red with
  haptic vibration — the controller borrowing a real big-aircraft idea. Those
  signals arrive over CAN.
- **Honesty guardrail:** the scripts state plainly that this is **not certified
  avionics** — it *borrows* proven reliability ideas from aviation and EVs. Keep
  this framing if you edit.

## Production tips

- The timed beat sheets assume a fast-cut talking-head + B-roll style. If you
  prefer pure screen-record/animation, the "clean full voiceover" block is your
  script; the beat sheet becomes your edit timeline.
- On-screen text callouts are written in CAPS in the scripts — drop them in as
  lower-thirds / captions at the noted timecodes.
- Lower-third for every video: **"Zach Whitehead · Co-founder, OpenPPG."**
- Recurring CTA theme across all six: *it's all open source — read the firmware
  on GitHub.* (Reinforces the brand identity.)

---

*Drafts generated from repo + wiki research. Numbers and specifics are sourced;
the voice/phrasing is a starting point — make it sound like you before you record.*
