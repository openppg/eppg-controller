# From Arduino Nano to ESP32: The Story of Our Controller
*Series: "Inside the OpenPPG Controller" · Episode 5 of 6 · ~2 minutes*

## 🎬 Video Script (~2 min)

**HOOK (0:00–0:10):** The throttle in your hand has been upgraded again and again since 2018 — four brains, three big leaps. Same idea my brother and I sketched in an Ohio garage, just a whole lot smarter now.

| Time | On-screen / B-roll | Spoken (voiceover) |
|------|-------------------|--------------------|
| 0:10–0:25 | Old garage / workbench photos, racing drones, early X4 kit laid out in pieces. TEXT: "OpenPPG · est. 2018 · Canfield, Ohio" | My brother Paul and I started OpenPPG in 2018 in Canfield, Ohio — out of racing drones, bootstrapped, with one promise from day one: fully open source. Our first product was the X4 — a quad-motor kit you soldered together over about six hours. |
| 0:25–0:40 | Macro of tiny Arduino Nano on a breadboard, OLED blinking, a few LEDs. TEXT: "2018 — Arduino Nano · 8-bit · 16 MHz" | The brain back then was an 8-bit Arduino Nano: just enough to spin the motors, drive a little OLED, and blink a few status lights. Call it the original flip phone. |
| 0:40–0:55 | Feather M0 board, second button + haptic buzzer, first real SP140 display. TEXT: "Late 2018 — SAMD21 · 32-bit · 48 MHz · first public SP140 2021" | Late 2018 we jumped to a 32-bit SAMD21, and suddenly we had headroom — a second button, a haptic buzzer that taps your hand, a real display. That chip carried us into the first public SP140 in 2021. |
| 0:55–1:15 | RP2040 board, split-screen "two cores," smooth animated UI. TEXT: "2022 — RP2040 · DUAL-core · 133 MHz" then "2023 (v6.0) — FreeRTOS · multitasking starts HERE" | 2022 brought the RP2040 — dual-core, much faster. In 2023, version 6.0, we added FreeRTOS, a real-time operating system, so the controller could do many jobs at once. That was the architectural turning point — where the multitasking story actually begins, not with the chip you're about to see. |
| 1:15–1:40 | ESP32-S3 StampS3 hero shot, vibrant LVGL color UI, animated graphic of throttle ↔ ESC ↔ BMS collapsing onto one wire pair. TEXT: "May 2025 — ESP32-S3 · v7.0 · biggest release ever" + "🚩 MILESTONE: CAN BUS — one digital nervous system" | Then, May 2025, our biggest release ever: the ESP32-S3 — dual-core, 240 megahertz, Wi-Fi and Bluetooth built in. The milestone to remember: CAN bus. Before this, the throttle barely heard the motor talk back. Now the motor's ESC and the battery's BMS share one rugged digital network — the same kind cars use — turning three dumb parts into one connected, two-way system. A digital nervous system. |
| 1:40–1:50 | Screen showing live RPM, temps, cell voltages; ESC LED changing color on command. TEXT: "40+ live signals · two-way · rich color UI" | It goes both ways: we don't just read the motor, we write to it over CAN — throttle, status lights, even its beeps. That's 40-plus live signals on a rich color display. We go deep on CAN in Episode 6. |
| 1:50–2:00 | Montage: solder-it-yourself X4 dissolving into a fully assembled SP140 a pilot clips in and flies. TEXT: "2026 — v8.0 · Cloud OTA · crash recovery" | In 2026, version 8.0 brings maturity: a pro ESP-IDF build, cloud over-the-air updates, crash recovery. From a kit you soldered to a paramotor anyone can fly. |

**CALL TO ACTION (last ~10s):** Open source on GitHub — stick around for Episode 6, where we crack open CAN bus.

**🎙️ Full voiceover (clean read):**
The throttle in your hand has been upgraded again and again since 2018 — four brains, three big leaps. My brother Paul and I started OpenPPG in 2018 in Canfield, Ohio — out of racing drones, bootstrapped, with one promise from day one: fully open source. Our first product was the X4 — a quad-motor kit you soldered together over about six hours. The brain back then was an 8-bit Arduino Nano: just enough to spin the motors, drive a little OLED, and blink a few status lights. Call it the original flip phone. Late 2018 we jumped to a 32-bit SAMD21, and suddenly we had headroom — a second button, a haptic buzzer that taps your hand, a real display. That chip carried us into the first public SP140 in 2021. 2022 brought the RP2040 — dual-core, much faster. In 2023, version 6.0, we added FreeRTOS, a real-time operating system, so the controller could do many jobs at once. That was the architectural turning point — where the multitasking story actually begins, not with the chip you're about to see. Then, May 2025, our biggest release ever: the ESP32-S3 — dual-core, 240 megahertz, Wi-Fi and Bluetooth built in. The milestone to remember: CAN bus. Before this, the throttle barely heard the motor talk back. Now the motor's ESC and the battery's BMS share one rugged digital network — the same kind cars use — turning three dumb parts into one connected, two-way system. A digital nervous system. It goes both ways: we don't just read the motor, we write to it over CAN — throttle, status lights, even its beeps. That's 40-plus live signals on a rich color display. We go deep on CAN in Episode 6. In 2026, version 8.0 brings maturity: a pro ESP-IDF build, cloud over-the-air updates, crash recovery. From a kit you soldered to a paramotor anyone can fly. Open source on GitHub — stick around for Episode 6, where we crack open CAN bus.
*(Word count: 330)*

**🅱️ B-roll / shot list:**
- Old garage / workbench shots, early racing-drone footage, the two brothers working
- X4 kit photographed in pieces, then mid-solder (soldering iron, hands)
- Macro of Arduino Nano on a breadboard; OLED blinking; status LEDs
- SAMD21 Feather M0; finger pressing the second button; haptic buzz close-up
- RP2040 board with a "two cores" split-screen graphic; smooth UI animation
- ESP32-S3 StampS3 hero rotation; vibrant LVGL color UI in sunlight
- Animated diagram: a bundle of analog wires collapsing into one CAN pair
- Screen scrolling live RPM / temps / per-cell voltages; ESC LED changing color on command; arm/disarm beep
- Final montage: solder-it-yourself kit dissolving into an assembled SP140 a pilot clips in and launches with

**🔤 On-screen text / lower-thirds:**
- "Zach Whitehead · Co-founder, OpenPPG"
- "OpenPPG · est. 2018 · Canfield, Ohio"
- "2018 — Arduino Nano · 8-bit · 16 MHz"
- "Late 2018 — SAMD21 · 32-bit · 48 MHz · first public SP140 2021"
- "2022 — RP2040 · DUAL-core · 133 MHz"
- "2023 (v6.0) — FreeRTOS · multitasking starts HERE"
- "May 2025 — ESP32-S3 · v7.0 · biggest release ever"
- "🚩 MILESTONE: CAN BUS — one digital nervous system"
- "40+ live signals · two-way · rich color UI"
- "2026 — v8.0 · Cloud OTA · crash recovery"
- "Open source on GitHub · Next: Episode 6 — CAN Bus"

## 📝 Companion Blog Post

# From an 8-bit Nano to a Wi-Fi ESP32 — Every Upgrade in Your Hand
*How the OpenPPG controller went from a solder-it-yourself Arduino Nano kit to a Wi-Fi-equipped ESP32 — and why each jump made your flying better.*

People ask me what's actually *inside* the throttle they're squeezing. The honest answer: a small computer that's been quietly upgraded over and over since my brother Paul and I started OpenPPG in 2018 — four different brains, three big leaps. Same core idea we sketched in Canfield, Ohio — a clean, open-source electric paramotor — just dramatically smarter each generation. The easiest way to understand it is the way you understand your own pocket: every processor jump was a phone upgrade.

## It started with a soldering iron (2018)

OpenPPG was bootstrapped — no investors, just two brothers out of racing-drone manufacturing who decided to put everything on GitHub from day one. Our first product was the **X4**, a quad-motor kit you assembled yourself in about six hours with a soldering iron. The brain was an **8-bit Arduino Nano** — roughly the simplest microcontroller that could do the job. It spun the motors, drove a tiny OLED, and blinked a few status LEDs. That was it, and that was enough to prove electric paramotoring could be open and DIY. Call it the original flip phone: does calls and texts, nothing fancy.

## Finding headroom (late 2018 → 2021)

The Nano was always running close to its limits, so late in 2018 we moved to a **32-bit SAMD21** (the Adafruit Feather M0). More speed, more room to breathe. That headroom bought pilots real features: a second button, a haptic buzzer that taps your hand instead of relying on your eyes, and a proper display. This chip is the one that carried us into the **first public SP140 in 2021** — the single-motor machine most people picture when they think OpenPPG.

## The real turning point: multitasking (2022–2023)

In 2022 we jumped again to the **RP2040** — a genuinely dual-core chip, much faster, with far more memory. But the chip wasn't the big story. The big story came in 2023, with version 6.0, when we introduced **FreeRTOS** — a real-time operating system — and rewrote the display around it.

I want to be precise here, because it's a common mix-up: the multitasking story *starts on the RP2040 in 2023*, not on the ESP32 that came later. That's when the controller learned to juggle many jobs at once — reading inputs, driving the screen, watching for faults — without one task ever blocking another. It was the architectural foundation everything since has been built on, and the ESP32-S3 firmware kept and expanded it.

## 🚩 The milestone: CAN bus and a digital nervous system (May 2025)

Then came May 2025 and version 7.0 — our **biggest release ever** — built on the **ESP32-S3**: dual-core, 240 MHz, with Wi-Fi and Bluetooth built right in.

The headline isn't the megahertz, though. It's **CAN bus** — and I'm flagging it as the milestone of this whole story.

Here's what changed. For years, the throttle talked to the motor over a pretty simple link. It started as a plain analog throttle signal, later gained basic UART telemetry — but the controller still couldn't really *hear* much back from the motor. With the ESP32-S3, we put the motor's **ESC** (its speed controller) *and* the battery's **BMS** (its management system) onto **CAN bus** — the same rugged, noise-resistant digital network that cars, aircraft, and EVs rely on. Three previously dumb, loosely-wired parts became **one connected, two-way system**. A digital nervous system.

And here's the part pilots don't expect: it goes *both directions*. We don't just *read* the motor — we **write** to it over CAN. Every throttle movement is a precise digital command; we also set the ESC's onboard status-light colors and tell it which tones and beeps to play. Reading deep diagnostics back is what feeds those 40-plus live signals on the rich new color display. CAN is so central it gets its own episode — **Episode 6** — so I'll save the deep dive for there.

## What this means when you're flying

A digital backbone instead of a bundle of analog wires means fewer connectors to fail and strong noise immunity right next to a high-power motor. It means the screen in your hand is showing real numbers from the motor and battery, not guesses — and the warnings you get are based on what the hardware is *actually* reporting in real time. That's the difference between a gauge and a conversation.

## The technical bits (for the nerds 🤓) — skippable

- **Processors:** ATmega328 (8-bit, ~16 MHz, ~2 KB RAM) → SAMD21 (Cortex-M0+, 48 MHz) → RP2040 (dual Cortex-M0+, 133 MHz, 264 KB RAM) → ESP32-S3 (dual Xtensa LX7, 240 MHz, ~512 KB SRAM, Wi-Fi + BLE). Four processors, three upgrade jumps.
- **Two CAN links:** the ESC uses the ESP32-S3's built-in CAN controller (Espressif brands it "TWAI" — Two-Wire Automotive Interface, just their name for standard CAN; RX pin 3, TX pin 2, library `openppg/SINE-ESC-CAN`). The BMS rides a separate **MCP2515** CAN chip on SPI (library `openppg/ANT-BMS-CAN`, thread-safe via a FreeRTOS mutex).
- **Protocol:** node-addressed; the ESC answers at node id **0x20**. Throttle commands go out via `setThrottleSettings2` at 0.1-microsecond resolution. The ESC expects a throttle message at least every ~300 ms or it stops — so the steady CAN heartbeat is itself a safety feature.
- **v7.0 also:** full LVGL color UI rewrite, expanded BLE via NimBLE, ESP32 Preferences replacing EEPROM. **v8.0 (2026):** hybrid Arduino + ESP-IDF build, cloud OTA, core-dumps, watchdog, automatic rollback.

## Where this fits

This was the standalone story of how a hand-soldered kit became a paramotor anyone can clip in and fly — and how each processor jump unlocked the next leap. Next up in Episode 6, we open the hood on the milestone I flagged here: CAN bus, and exactly how your throttle, motor, and battery talk to each other.

## 📌 Caption + hashtags

From an 8-bit Arduino Nano and a solder-it-yourself X4 kit to a Wi-Fi-equipped ESP32-S3 — that's the story of the throttle in your hand: four brains, three big upgrades since 2018. The biggest leap? Putting your motor and battery on CAN bus, one two-way "digital nervous system." Episode 5 of 6 in our open-source build-in-public series — full code on GitHub, and CAN gets its own episode next. 🪂

#OpenPPG #electricparamotor #paramotor #PPG #openhardware #opensource #ESP32 #CANbus #buildinpublic #avgeek
