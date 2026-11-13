# What's an RTOS? Why Your Throttle Runs 13 Jobs at Once
*Series: "Inside the OpenPPG Controller" · Episode 1 of 6 · ~2 minutes*

## 🎬 Video Script (~2 min)

**HOOK (0:00–0:10):** When you squeeze the throttle, the screen is redrawing, Bluetooth is streaming to your phone, the battery is being watched, and a buzzer is standing by — all at once. Here's the trick that keeps any of that from ever slowing down your throttle.

| Time | On-screen / B-roll | Spoken (voiceover) |
|------|--------------------|--------------------|
| 0:10–0:25 | Close-up of the SP140 controller in hand, thumb on the throttle. Cut to old simple code scrolling. CAPTION: "ONE LOOP. ONE JOB AT A TIME." | A basic Arduino program is one list of instructions running top to bottom, over and over. Read throttle, draw screen, check battery — in a fixed line. Fine for a blinking LED. Not fine when your throttle is stuck behind a slow screen redraw. |
| 0:25–0:45 | Motion graphic: a single lane of traffic jamming up, then splitting into an organized kitchen with stations. CAPTION: "RTOS = MANY JOBS AT ONCE" | So our controller runs a real-time operating system — FreeRTOS. Instead of one loop, it runs many independent jobs called tasks, each with a priority, and a scheduler always runs the most urgent one first. Think of a kitchen full of stations with a head chef who can interrupt any cook the second a VIP order comes in. |
| 0:45–1:05 | Graphic: chip labeled ESP32-S3 splitting into CORE 0 and CORE 1, tasks slotting into each. CAPTION: "13 TASKS · 2 CORES" | Today's controller has two CPU cores and runs 13 of these tasks. Your throttle task runs at the highest priority and gets its own core — Core 0 — next to the watchdog and the buttons. The heavy stuff — screen, Bluetooth, audio — lives on Core 1. |
| 1:05–1:25 | Split screen: left = throttle reading smoothly; right = screen redrawing, phone streaming. CAPTION: "THROTTLE NEVER WAITS" | That's the whole point. Redrawing the display or streaming telemetry physically cannot delay reading your throttle, because they're on a different core. And the scheduler is preemptive — the moment the throttle needs the processor, it bumps everything lower out of the way. |
| 1:25–1:40 | Graphic: a single key on a hook passed between tasks for "SPI bus" / "I2C". CAPTION: "ONE KEY. NO COLLISIONS." | When two jobs share the same hardware — like the display bus — we use a mutex: one key only one task can hold at a time, so they never scribble over each other. And a telemetry hub cleanly hands sensor data off to the Bluetooth task. |
| 1:40–1:55 | Founder to camera, controller in hand. CAPTION: "NOT MAGIC — CAREFUL DESIGN." | Now — an RTOS isn't magic. We still have to choose the priorities, the memory sizes, and the locks carefully. But done right, it buys the one thing that matters in flight: deterministic, predictable timing. |
| 1:55–2:05 | Tight shot of throttle squeeze, motor spins up. CAPTION: "A THROTTLE THAT ALWAYS ANSWERS." | A throttle that always answers, instantly, no matter what else is going on. That's not a nice-to-have. That's safety. |

**CALL TO ACTION (last ~10s):** This is Episode 1 of six on what's actually inside your controller — it's all open source on GitHub, so follow along and come build with us.

**🎙️ Full voiceover (clean read):**

When you squeeze the throttle, the screen is redrawing, Bluetooth is streaming to your phone, the battery is being watched, and a buzzer is standing by — all at once. Here's the trick that keeps any of that from ever slowing down your throttle. A basic Arduino program is one list of instructions running top to bottom, over and over. Read throttle, draw screen, check battery — in a fixed line. Fine for a blinking LED. Not fine when your throttle is stuck behind a slow screen redraw. So our controller runs a real-time operating system — FreeRTOS. Instead of one loop, it runs many independent jobs called tasks, each with a priority, and a scheduler always runs the most urgent one first. Think of a kitchen full of stations with a head chef who can interrupt any cook the second a VIP order comes in. Today's controller has two CPU cores and runs 13 of these tasks. Your throttle task runs at the highest priority and gets its own core — Core 0 — next to the watchdog and the buttons. The heavy stuff — screen, Bluetooth, audio — lives on Core 1. That's the whole point. Redrawing the display or streaming telemetry physically cannot delay reading your throttle, because they're on a different core. And the scheduler is preemptive — the moment the throttle needs the processor, it bumps everything lower out of the way. When two jobs share the same hardware — like the display bus — we use a mutex: one key only one task can hold at a time, so they never scribble over each other. And a telemetry hub cleanly hands sensor data off to the Bluetooth task. Now — an RTOS isn't magic. We still have to choose the priorities, the memory sizes, and the locks carefully. But done right, it buys the one thing that matters in flight: deterministic, predictable timing. A throttle that always answers, instantly, no matter what else is going on. That's not a nice-to-have. That's safety. This is Episode 1 of six on what's actually inside your controller — it's all open source on GitHub, so follow along and come build with us.

*(Word count: 312)*

**🅱️ B-roll / shot list:**
- Hand holding the SP140 controller, thumb resting on and squeezing the throttle (multiple angles)
- Macro of the color display redrawing / animating
- Phone running the app with live telemetry scrolling
- Screen-recording of simple top-to-bottom code vs. a motion graphic of parallel tasks
- Motion graphic: ESP32-S3 chip splitting into Core 0 / Core 1 with task chips dropping in
- Motion graphic: a single "key on a hook" being handed between two tasks
- Founder to-camera, controller in hand, for the "not magic" beat
- Closing: throttle squeeze → motor/prop spins up

**🔤 On-screen text / lower-thirds:**
- "ONE LOOP. ONE JOB AT A TIME."
- "RTOS = MANY JOBS AT ONCE"
- "13 TASKS · 2 CORES"
- "THROTTLE = HIGHEST PRIORITY · CORE 0"
- "THROTTLE NEVER WAITS"
- "ONE KEY. NO COLLISIONS." (mutex)
- "NOT MAGIC — CAREFUL DESIGN."
- "DETERMINISTIC = A THROTTLE THAT ALWAYS ANSWERS"
- Lower-third: "Zach Whitehead · Co-founder, OpenPPG"

## 📝 Companion Blog Post

# Why Your Throttle Runs 13 Jobs at the Same Time (and Never Misses Yours)
*A plain-English look at the real-time operating system inside your OpenPPG controller — and why it makes the throttle safer.*

Squeeze the throttle on your SP140 and a surprising amount is happening in that same instant. The color screen is redrawing. Your phone app is pulling live telemetry over Bluetooth. The battery management system is being polled. A buzzer and a vibration motor are standing by for alerts. The barometer is reading your altitude. And somewhere in all of that, the most important job of all has to happen on a rock-steady schedule: read the pilot's throttle and command the motor.

The reason all of that coexists without your throttle ever stuttering comes down to one piece of software design most pilots have never heard of: a real-time operating system. Let me explain what that is, and why I care about it so much.

## The old way: one loop, one job at a time

If you've ever poked at an Arduino, you know the basic shape of the code: a single `loop()` that runs top to bottom, over and over. Read this, then draw that, then check the other thing, then start again. It's beautifully simple, and for our earliest controllers — tiny 8-bit chips with barely any memory — it was exactly right. The controller read a throttle and lit up an LED strip. That's it.

The problem with one loop shows up as the device grows. The moment you add a graphical screen, a Bluetooth link, battery telemetry, audio, and haptics, your throttle is suddenly stuck in line behind all of them. If the screen redraw runs long, your throttle input waits. Worse, the timing shifts every time you add a feature — so you can never quite promise how fast the throttle gets serviced. For a paramotor, "it depends" is not an answer I'm willing to ship.

## The new way: many tasks, each with a priority

A real-time operating system flips the model. Instead of one loop doing everything in order, the firmware is broken into many independent jobs — called **tasks** — that run concurrently. Each task has a **priority**, and a piece of software called the **scheduler** always runs the most urgent task that's ready to go.

We use **FreeRTOS** for this. It's not something we invented in a lab — it's the de-facto standard real-time kernel across the embedded industry, open source, and battle-tested in countless devices. Importantly, this isn't new for us: we first moved to FreeRTOS back in our RP2040 generation (firmware v6.0, in 2023). Today's ESP32-S3 controller kept that model and expanded it.

Here's the analogy I like. A single loop is one cook in a kitchen doing every step in a fixed order — chop, plate, wash, answer the phone. If the washing runs long, the food on the pass goes cold. An RTOS is a real kitchen: separate stations all working at once, with a head chef (the scheduler) who can interrupt any cook the instant a higher-priority order lands. In our controller, "read the throttle" is the order that always jumps the line.

## Two cores, 13 tasks, and a reserved lane for your throttle

Our current controller runs on the ESP32-S3, which has **two CPU cores** — two brains in one chip. The firmware runs **13 tasks** across them, and we deliberately assign which job runs on which core.

The safety-critical work lives on **Core 0**: the **throttle task runs at the highest priority and is pinned there**, right alongside the **watchdog** (the safety timer that reboots us if anything ever freezes) and the **buttons**. The heavier "experience" work — the display/UI, the battery readout, Bluetooth notifications, audio, haptics — runs on **Core 1**.

Why does that split matter to you? Because redrawing the screen or streaming telemetry to your phone happens on a *different core* than the one reading your throttle. They literally cannot steal each other's time. It's like a two-lane bridge where we've reserved one lane for emergency vehicles only: a traffic jam of pretty graphics in Lane 1 physically can't block the ambulance in Lane 0.

On top of that, the scheduler is **preemptive**. The moment the throttle task needs the processor, it interrupts whatever lower-priority work is running. Bluetooth and audio get told to hold. Your throttle is always serviced first.

## Keeping tasks from colliding — and an honest caveat

If many tasks run at once, what stops two of them from grabbing the same piece of hardware at the same instant and corrupting it? The answer is a **mutex** — think of it as the single bathroom key on a hook at a gas station. There's exactly one key. You take it, do your thing, hang it back. Anyone else simply waits. We use mutexes to protect shared hardware like the display's SPI bus and the I2C line to the altimeter, so two tasks never barge in at once. Separately, a **telemetry hub** acts as a clean drop-off point: sensor tasks publish their latest readings, and the Bluetooth task picks them up without either side having to wait on the other.

Here's the honest part, straight from the FreeRTOS folks themselves: an RTOS is not magic. It doesn't automatically make software correct. We still have to choose each task's priority, size its memory carefully, and place the locks just right — get any of that wrong and you've traded one problem for another. The benefit is real, but it's earned through careful design, not handed to us for free.

## What this means when you're flying

All of this lands on one word: **deterministic**. A real-time operating system lets us guarantee that reading your throttle and commanding the motor happens on a tight, predictable schedule — no matter what the screen, the phone link, or the alerts are doing. Predictable timing isn't a luxury feature. On a paramotor, it's a safety property. It means your throttle always answers, instantly, the same way, every flight.

### The technical bits (for the nerds 🤓)
*Totally skippable.* The controller runs FreeRTOS (the kernel bundled inside ESP-IDF) in dual-core SMP mode on the ESP32-S3 (dual Xtensa LX7 @ 240 MHz). 13 tasks, each created with explicit stack size, priority (higher number = higher priority), and core pinning via `xTaskCreatePinnedToCore`. The `throttle` task is priority 3 (highest), pinned to core 0, 4352-byte stack; `watchdog` is also priority 3, core 0; `ButtonHandler` is priority 2, core 0. Core 1 carries `UI`, `BMS`, `BLENotify` (8192-byte stack — the NimBLE notify path through GATT/ATT/L2CAP/HCI is deep), `Audio`, and `Vibration`. (A few background tasks — `CtrlSensor`, `WebSerial`, `Monitoring` — are left unpinned and can float to either core; the clean "Core 0 = safety, Core 1 = everything else" split is the design intent, not a literal rule for all 13.) Shared resources are guarded by FreeRTOS mutexes: `spiBusMutex`, `i2cMutex`, `lvglMutex`, `stateMutex`, and a telemetry-hub mutex (with short ~2 ms timeouts so nothing blocks). Tasks communicate through FreeRTOS queues (throttle-update, melody/audio, vibration, alert events). FreeRTOS has been in our firmware since the RP2040 era (v6.0, 2023); the ESP32-S3 firmware kept and expanded the model — note that even our Arduino-only ESP32 builds ran FreeRTOS underneath, because Arduino-ESP32 itself sits on ESP-IDF.

Next time in the series, I'll dig into *why* we build on ESP-IDF and what that unlocks for you as a pilot. This is firmware you can actually read — it's all open source on GitHub.

## 📌 Caption + hashtags

Ever wonder how your throttle stays instant while the screen redraws and your phone streams telemetry at the same time? It comes down to a real-time operating system (FreeRTOS) running 13 separate jobs across two CPU cores — with your throttle pinned to its own core at the highest priority so it never waits in line. Episode 1 of our 6-part "Inside the OpenPPG Controller" series breaks it down for pilots, no engineering degree required. 🪂

#paramotor #ppg #electricparamotor #ePPG #openppg #electricflight #freeflight #embeddedsystems #ESP32 #firmware #FreeRTOS #avgeek
