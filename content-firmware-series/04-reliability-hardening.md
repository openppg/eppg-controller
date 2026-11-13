# How the Controller Survives a Crash — The Software Kind
*Series: "Inside the OpenPPG Controller" · Episode 4 of 6 · ~2 minutes*

## 🎬 Video Script (~2 min)
**HOOK (0:00–0:10):** Here's a question that should bug every electric pilot: what happens if the software in your throttle ever freezes mid-flight? In v8.0, we borrowed four ideas from airliners and EVs so that the answer is "you barely notice."

| Time | On-screen / B-roll | Spoken (voiceover) |
|------|-------------------|--------------------|
| 0:10–0:35 | Close-up of the controller display cycling alerts; a battery cell number turns YELLOW, then RED; pilot's hand feels the buzz. ON-SCREEN: "EICAS-STYLE ALERTS · 40+ LIVE SIGNALS" | The biggest one first: the controller is an always-watching co-pilot. Real airliners call it EICAS — Engine Indication and Crew Alerting System. Ours watches 40-plus live signals: motor and battery temperatures, the highest and lowest cell voltages, current, state of charge, even its own CPU. Yellow means warning. Red means critical — and it buzzes the throttle right in your hand. |
| 0:35–0:55 | Motion graphic: a little "heartbeat" pulse feeding a timer; timer stalls; device reboots clean. ON-SCREEN: "WATCHDOG · DEAD-MAN'S SWITCH" | Second: the watchdog. Picture the safety clip on a treadmill. A dedicated high-priority task has to keep tapping a hardware timer, "I'm still here." If the software ever truly hangs, nobody taps it — and the controller reboots itself clean in seconds. A frozen throttle is the one thing we refuse to allow. |
| 0:55–1:20 | Animation of a flash chip; a small "black box" icon writes a record that survives a reboot; JSON appears on a laptop over USB. ON-SCREEN: "CORE DUMPS · THE BLACK BOX" | Third: the black box. If the firmware ever does crash, it writes a forensic record to flash — which task, where, and why — and that record survives the reboot. Plug in or connect the app later, and the controller reads it back as clean, decoded data you can share with us. No more guessing. Same idea as an aircraft flight recorder, for software. |
| 1:20–1:45 | Split screen: new firmware loads into a spare slot; it has to "prove" it boots; if not, arrow snaps back to old version. ON-SCREEN: "AUTOMATIC ROLLBACK · A/B UPDATE" | Fourth: automatic rollback. Just like A/B updates in your car or phone, a fresh update lands in a spare slot and has to prove it boots cleanly. If it can't, the controller quietly reverts to the last firmware that was flying fine yesterday. A bad update can't brick your throttle. |
| 1:45–1:55 | Founder to camera, controller in hand. ON-SCREEN: "NOT CERTIFIED AVIONICS — BORROWED IDEAS" | I'll be straight with you: this is not certified avionics. We're borrowing proven reliability ideas from aviation and EVs and putting them in a paramotor throttle. |
| 1:55–2:05 | Wide shot, pilot launching at sunset. ON-SCREEN: "FEWER SURPRISES. CATCH IT. CONTAIN IT. LEARN." | The payoff: fewer surprises in the air. And when something does go wrong, we catch it, contain it, and learn from it. |

**CALL TO ACTION (last ~10s):** It's all open source — read the code yourself on GitHub, and subscribe for the rest of the series.

**🎙️ Full voiceover (clean read):** Here's a question that should bug every electric pilot: what happens if the software in your throttle ever freezes mid-flight? In v8.0, we borrowed four ideas from airliners and EVs so that the answer is "you barely notice." The biggest one first: the controller is an always-watching co-pilot. Real airliners call it EICAS — Engine Indication and Crew Alerting System. Ours watches 40-plus live signals: motor and battery temperatures, the highest and lowest cell voltages, current, state of charge, even its own CPU. Yellow means warning. Red means critical — and it buzzes the throttle right in your hand. Second: the watchdog. Picture the safety clip on a treadmill. A dedicated high-priority task has to keep tapping a hardware timer, "I'm still here." If the software ever truly hangs, nobody taps it — and the controller reboots itself clean in seconds. A frozen throttle is the one thing we refuse to allow. Third: the black box. If the firmware ever does crash, it writes a forensic record to flash — which task, where, and why — and that record survives the reboot. Plug in or connect the app later, and the controller reads it back as clean, decoded data you can share with us. No more guessing. Same idea as an aircraft flight recorder, for software. Fourth: automatic rollback. Just like A/B updates in your car or phone, a fresh update lands in a spare slot and has to prove it boots cleanly. If it can't, the controller quietly reverts to the last firmware that was flying fine yesterday. A bad update can't brick your throttle. I'll be straight with you: this is not certified avionics. We're borrowing proven reliability ideas from aviation and EVs and putting them in a paramotor throttle. The payoff: fewer surprises in the air. And when something does go wrong, we catch it, contain it, and learn from it. It's all open source — read the code yourself on GitHub, and subscribe for the rest of the series. *(Word count: 316)*

**🅱️ B-roll / shot list:**
- Macro of the controller display with a cell voltage flipping yellow then red
- Pilot's hand on the throttle reacting to a haptic buzz
- Motion graphic of a "heartbeat" feeding a watchdog timer, then a clean reboot
- Flash-chip / "black box" animation writing a record that survives a power cycle
- Laptop screen showing decoded crash JSON over USB
- A/B partition animation: new firmware into a spare slot, snap-back to old version on failure
- Founder to camera with controller in hand
- Sunset launch / flight wide shot

**🔤 On-screen text / lower-thirds:**
- "EICAS-STYLE ALERTS · 40+ LIVE SIGNALS"
- "YELLOW = WARNING · RED = CRITICAL (BUZZES YOUR HAND)"
- "WATCHDOG · DEAD-MAN'S SWITCH"
- "CORE DUMPS · THE BLACK BOX"
- "AUTOMATIC ROLLBACK · A/B UPDATE"
- "NOT CERTIFIED AVIONICS — BORROWED IDEAS"
- "FEWER SURPRISES. CATCH IT. CONTAIN IT. LEARN."

## 📝 Companion Blog Post

# Your Throttle Has a Co-Pilot, a Black Box, and a Safety Net
*How OpenPPG's v8.0 firmware borrows four proven reliability ideas from aviation and EVs — and what each one means when you're in the air.*

Let me say the honest part up front: a paramotor controller is not certified avionics. We don't pretend it is. But there's a whole world of reliability engineering that big aircraft and modern EVs have spent decades refining, and a lot of those ideas translate beautifully to the handheld throttle you fly with. In v8.0 — our reliability-hardening release — we leaned into four of them. Here's what they are, and why they matter the next time you clip in.

## The always-watching co-pilot (EICAS-style alerts)

Real airliners run a system called EICAS — Engine Indication and Crew Alerting System. Instead of making the crew scan a wall of individual gauges, EICAS centralizes the monitoring and only shouts when something needs attention. We borrowed that idea wholesale.

Our controller continuously watches **40-plus live signals**. From the motor's ESC: MOSFET, MCU, capacitor, and motor temperatures, plus over-current, locked rotor, over-temp, over-voltage, and voltage-drop conditions. From the battery's BMS: cell-group temperatures, per-cell high and low voltage, state of charge, pack voltage, voltage differential, and the discharge-MOSFET state. And from itself: CPU temperature and whether the barometer came up cleanly at boot.

When something drifts out of range, you get a **yellow warning**. When it crosses into dangerous territory, you get a **red critical alert** — and the controller fires a haptic buzz right in your throttle hand. (Temperature alerts use a small hysteresis band so they don't flicker annoyingly on and off at the threshold.) This is the pillar I'm proudest of, because it's the one you'll actually feel: you don't have to be staring at the screen to know your battery is getting hot. The controller taps you on the shoulder.

**What this means when you're flying:** you find out about a hot pack or a sagging cell *while you still have options*, not after.

## The dead-man's switch (watchdog)

Software, on rare occasions, can get stuck. The nightmare scenario for any throttle is a freeze — a controller that's locked up but still holding a command. We refuse to allow that.

Think of the safety clip on a treadmill, or the dead-man's switch on a chainsaw: as long as you're gripping it, things run; the instant you let go, power cuts. Our firmware has a **dedicated, high-priority watchdog task** whose entire job is to keep "petting" a hardware timer — a steady *I'm still here, I'm still here*. If the software ever genuinely hangs and stops petting it, the hardware watchdog assumes the worst and **automatically reboots the controller into a clean state** in seconds. No button press from you. By the time it's recovering, the motor controller's own failsafe has already idled the motor.

**What this means when you're flying:** the one outcome we treat as unacceptable — a frozen throttle — gets caught and reset automatically instead of leaving you holding a dead stick.

## The black box (core dumps)

Here's the difference between a hobby gadget and gear you trust: when something rare goes wrong, do you *learn* from it, or do you shrug?

An aircraft's orange flight recorder is built to survive the very event it's recording, so investigators can later read out exactly what happened and stop it from happening again. We built the software equivalent. If our firmware ever crashes, it writes a forensic record to a protected area of flash — **which task faulted, where in the code, the exception cause, and the call-stack trail that led there**. Crucially, that record **survives the reboot**. The next time you plug in over USB or connect the app, the controller reads it back as clean, decoded data you can share with us.

The honest boundary: unlike an aircraft recorder that logs the whole flight continuously, ours captures the *crash* — a snapshot of the failure, not a movie of your day. But that snapshot turns a useless "it just rebooted once" report into "here's the exact function that failed and why."

**What this means when you're flying:** a one-in-a-thousand glitch on your unit becomes a fix in everyone's next update, instead of a ghost we chase forever.

## The safety net (automatic rollback)

Updates are great until one isn't. Your phone and your car already solved this with **A/B updates**: the new version installs to a spare slot while the version you're using stays untouched, and the device only switches over once the new one proves itself.

Our controller does the same. A freshly flashed update lands in a spare partition and boots in a "pending verify" state — it has to come up cleanly and confirm it's healthy. If it can't, on the next restart the bootloader **automatically reverts to the last known-good firmware**. The firmware you were flying yesterday is never overwritten until the new one earns its place. A bad update simply cannot brick your throttle. (This ties directly into our over-the-air update story — that's Episode 3.)

Backing all of this is a deterministic, phased boot — a kind of preflight checklist where no part of the system starts running until everything it depends on already exists — and version-pinned builds, so the same firmware tag always produces the same binary.

**What this means when you're flying:** you can accept an update without that little knot in your stomach.

## The technical bits (for the nerds 🤓 — skippable)

v8.0-rc1 moved to a hybrid `framework = arduino, espidf` build, which is what unlocked direct access to the ESP-IDF primitives below.

- **EICAS-style alerts:** added in v7.2; an `AlertAgg` task (priority 1, 4096-byte stack) coalesces alerts from multiple producers via three FreeRTOS queues, with ±1 °C hysteresis on temperature thresholds. Critical alerts dispatch a `VibeRequest` to the vibration task.
- **Watchdog:** a dedicated `watchdog` task (priority 3, core 0) plus the throttle task are the only two subscribed to the ESP-IDF Task Watchdog Timer. `WDT_TIMEOUT_SECONDS = 5`, and `CONFIG_ESP_TASK_WDT_PANIC=y` turns a timeout into a panic + reset rather than just a warning.
- **Core dumps:** `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH` + `CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF`, read back via `esp_core_dump_get_summary()` in `diagnostics.cpp` (task name, PC, exc_cause, exc_vaddr, 6-frame backtrace), decoded to JSON at boot. Diagnostics also decode the reset reason (POWERON / PANIC / TASK_WDT / INT_WDT / BROWNOUT…) and keep a 16-slot boot history.
- **Rollback:** `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`; the image boots `PENDING_VERIFY` and must call `esp_ota_mark_app_valid_cancel_rollback()` by the end of `setup()` or the bootloader reverts. OTA writes to the spare A/B app partition (dual OTA slots in an 8 MB layout).

None of this is secret — it's all open source. Read it, fork it, or just trust that it's there.

This is one piece of a six-part series on what's actually inside the controller you fly with. Next door: Episode 3 covers how that firmware reaches your device over the air in the first place — no cable, no risk.

## 📌 Caption + hashtags

What happens if your throttle's software freezes mid-flight? In firmware v8.0 we borrowed four reliability ideas from airliners and EVs so you barely notice: an EICAS-style co-pilot watching 40+ live signals (and buzzing your hand when something's wrong), a watchdog that reboots a hung controller automatically, a "black box" that records any crash so we can fix it, and automatic A/B rollback so a bad update can't brick your unit. Not certified avionics — just proven ideas, fewer surprises in the air, and it's all open source. 🪂

#paramotor #ePPG #electricparamotor #openppg #electricflight #paramotoring #embeddedsystems #ESP32 #firmware #flightsafety #avgeek #opensourcehardware
