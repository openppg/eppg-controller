# Why We Build on ESP-IDF (and What It Unlocks for Pilots)
*Series: "Inside the OpenPPG Controller" · Episode 2 of 6 · ~2 minutes*

## 🎬 Video Script (~2 min)
**HOOK (0:00–0:10):** Every reliability feature we shipped in v8.0 — wireless updates, crash black-box, auto-rollback — none of it was possible until we changed one line in our build. Let me show you what that line unlocked.

| Time | On-screen / B-roll | Spoken (voiceover) |
|------|--------------------|--------------------|
| 0:10–0:25 | Close-up of the SP140 controller in hand, screen glowing. Motion-graphic text: ARDUINO | When we put the ESP32 brain in the controller, we built the firmware on Arduino. And honestly? Arduino is fantastic. It got us moving fast, and it gives us a huge library ecosystem — the display, the sensors, Bluetooth — all of it. |
| 0:25–0:45 | Simple layered diagram: "ARDUINO" sitting ON TOP of a wider block labeled "ESP-IDF + FreeRTOS". Arrow pointing down. | Here's the part most people don't know: Arduino for the ESP32 is actually built on top of Espressif's professional framework — ESP-IDF. So even back in v7.0, ESP-IDF was already running underneath. We just couldn't reach it directly. |
| 0:45–1:05 | Animation: a sealed panel labeled "FACTORY CONTROLS" cracking open. Text: framework = arduino, espidf | In v8.0 we flipped the build around. Instead of Arduino hiding ESP-IDF, we now build AS an ESP-IDF project — with Arduino along for the ride as a component. Same proven engine. We just got the keys to the factory controls. |
| 1:05–1:25 | Quick-cut icons: phone pushing update / "black box" / shield with checkmark / partition A-B graphic | And that's exactly what made v8.0 possible. Native over-the-air update APIs. Crash core-dumps. Watchdog and panic settings. Dual A/B partitions. And automatic rollback — so a bad update can't brick your controller. |
| 1:25–1:45 | The cloud OTA flow: phone → controller over Bluetooth. Text: OPPGFWV1 marker · A/B update · auto-rollback | The cloud update system, the version marker our backend checks, the safety net that reverts a broken build — every one of those needs that lower-level access. None of it lives in the Arduino layer. |
| 1:45–1:55 | Founder to camera, controller in hand. Text: SAME ENGINE. FULL CONTROL. | We didn't throw away eight years of work. We kept the libraries we trust and gained the professional toolbox underneath. We graduated from a great starter kit to factory-level control. |
| 1:55–2:05 | OpenPPG logo + GitHub URL on screen | And like everything we do — it's all open source. Every line is on GitHub. |

**CALL TO ACTION (last ~10s):** Read the actual firmware yourself on GitHub — and stick around for episode three, where we send a real update over the air with no cable, and a built-in safety net if anything goes wrong.

**🎙️ Full voiceover (clean read):** Every reliability feature we shipped in v8.0 — wireless updates, crash black-box, auto-rollback — none of it was possible until we changed one line in our build. Let me show you what that line unlocked. When we put the ESP32 brain in the controller, we built the firmware on Arduino. And honestly? Arduino is fantastic. It got us moving fast, and it gives us a huge library ecosystem — the display, the sensors, Bluetooth — all of it. Here's the part most people don't know: Arduino for the ESP32 is actually built on top of Espressif's professional framework — ESP-IDF. So even back in v7.0, ESP-IDF was already running underneath. We just couldn't reach it directly. In v8.0 we flipped the build around. Instead of Arduino hiding ESP-IDF, we now build as an ESP-IDF project — with Arduino along for the ride as a component. Same proven engine. We just got the keys to the factory controls. And that's exactly what made v8.0 possible. Native over-the-air update APIs. Crash core-dumps. Watchdog and panic settings. Dual A/B partitions. And automatic rollback — so a bad update can't brick your controller. The cloud update system, the version marker our backend checks, the safety net that reverts a broken build — every one of those needs that lower-level access. None of it lives in the Arduino layer. We didn't throw away eight years of work. We kept the libraries we trust and gained the professional toolbox underneath. We graduated from a great starter kit to factory-level control. And like everything we do — it's all open source. Every line is on GitHub. Read the actual firmware yourself on GitHub — and stick around for episode three, where we send a real update over the air with no cable, and a built-in safety net if anything goes wrong. *(Word count: 304)*

**🅱️ B-roll / shot list:**
- Hands holding the SP140 controller, screen on, slow rotate
- Macro of the M5Stack StampS3 module / board detail
- Screen-capture of `platformio.ini` scrolling to the `framework = arduino, espidf` line
- Screen-capture of the GitHub repo (file tree, commit history)
- Phone next to the controller, Bluetooth-update mockup
- Motion-graphic: layered stack (Arduino on top of ESP-IDF/FreeRTOS)
- Motion-graphic: sealed "factory controls" panel opening
- Motion-graphic: A/B partition swap with a rollback arrow

**🔤 On-screen text / lower-thirds:**
- ARDUINO = great starter kit
- ESP-IDF = Espressif's production framework
- Arduino-for-ESP32 is BUILT ON ESP-IDF
- v8.0: `framework = arduino, espidf`
- esp_ota · esp_core_dump · watchdog panic · A/B partitions · auto-rollback
- "OPPGFWV1" — the version marker our cloud checks
- SAME ENGINE. FULL CONTROL.
- 100% open source — github.com/openppg

## 📝 Companion Blog Post

# We Didn't Switch Frameworks. We Unlocked the One We Already Had.
*Why the OpenPPG controller's v8.0 firmware now builds on ESP-IDF — and why that quiet change is the foundation for everything that makes your update safe.*

If you read the v8.0 release notes, the headline features are the exciting ones: updating your paramotor over the air, a crash "black box," automatic rollback so a bad update can't brick your controller. But none of those would exist without a much less glamorous change buried in the build settings. This post is about that change — what it is, why it's honest, and why it matters when you're standing in a field about to fly.

## Arduino got us here, and that's not an insult

Let me be clear up front: Arduino is great. When we moved the controller to the ESP32-S3 brain (that was v7.0, back in May 2025), we built the firmware on the Arduino framework. Arduino is how a small, bootstrapped company in Canfield, Ohio gets a color display, barometric altimeter, CAN bus, and Bluetooth all talking to each other without reinventing every wheel. It gives us a massive library ecosystem that's been tested by millions of people. For getting a real product out the door, it's hard to beat.

So this isn't a story about Arduino being bad. It's a story about Arduino being a starter kit — a very good one — and us needing the full toolbox underneath it.

## The part almost nobody knows: Arduino runs ON TOP of ESP-IDF

Here's the fact that makes this whole change honest instead of dramatic.

ESP-IDF is Espressif's official, professional development framework for the ESP32 family — the one used in commercial, shipping products like smart speakers and industrial gear. And Arduino-for-ESP32 isn't a competitor to it. Arduino-for-ESP32 is literally *built on top of* ESP-IDF. Espressif's own documentation says you can use Arduino as an ESP-IDF component and get "full flexibility of the ESP-IDF."

That means even back in v7.0, when our firmware was "just Arduino," ESP-IDF and its real-time operating system, FreeRTOS, were already running underneath. (We dug into that RTOS in episode one — it's what lets your throttle run as its own protected job while the screen and Bluetooth do their thing.) The professional foundation was always there. We just couldn't reach down and operate it directly. Arduino's simplicity comes from hiding that machinery — which is perfect for a hobby project, and limiting for a flight controller.

## v8.0: we flipped the build around

The actual change in v8.0 was one setting: instead of building an Arduino project, we now build an **ESP-IDF project with Arduino as a component**. Same engine. Same libraries we rely on. We just inverted who's on top.

Think of it like the difference between a sealed factory engine map and a manufacturer handing you the real tuning port. The engine block is identical. But now we can set the rev limiter, decide which CPU core handles which job, and read the fault codes after a misfire. Nothing got replaced — a locked panel got opened.

Or, if you like: Arduino-only is the same bicycle with training wheels and a fixed seat. You can ride immediately, but you can't lean into a corner. We took the training wheels off the *same bike*.

## What opening that panel actually unlocked

Once we were building as an ESP-IDF project, we could call the professional-grade tools directly — the ones that simply don't exist in the Arduino layer:

- **Native over-the-air update APIs** (`esp_ota`) — the engine behind updating your motor wirelessly.
- **A/B dual partitions** — your firmware lives in one slot; a new update lands in the spare slot, so the version you're flying is never overwritten mid-update.
- **Automatic rollback** — a freshly flashed update has to prove it boots and runs; if it can't, the controller quietly reverts to the last version that was flying fine. A bad update can't strand you.
- **Crash core-dumps** — a software "black box" that survives a reboot so we can find and fix a rare glitch instead of losing it (we'll dig into the crash black box in a later episode).
- **Watchdog and panic settings** — fine-grained control over how the controller recovers if something ever hangs.
- **Full build configuration** (`sdkconfig`) — every important setting recorded in one place, so our builds are reproducible and auditable.

Here's the punchline: every one of v8.0's marquee features needs that lower-level access. The version marker our cloud backend validates before it'll publish a build, the A/B partition update mechanism, the auto-rollback safety net — none of it lives in the Arduino layer. The framework change wasn't a side effect of v8.0. It was the *gate* that made v8.0 possible.

## What this means when you're flying

You won't see "ESP-IDF" anywhere on your screen, and that's the point. What you'll see is a controller that can take a wireless update safely, that won't brick itself if an update goes sideways, and that records what happened if it ever hiccups. The boring build change is the reason the exciting safety features can exist at all. This is us hardening the firmware to the standard you deserve when you're hanging under a wing.

## The technical bits (for the nerds 🤓)
*Totally skippable.*

- The change: `platformio.ini` went from `framework = arduino` (v7.0) to `framework = arduino, espidf` (v8.0-rc1, tagged 2026-06-22). Platform `espressif32@6.13.0`, board `m5stack-stamps3` (ESP32-S3, dual Xtensa LX7 @ 240 MHz).
- Build hierarchy inverted: the project now uses ESP-IDF's CMake (`include($ENV{IDF_PATH}/tools/cmake/project.cmake)`) with Arduino pulled in as a required component (`src/CMakeLists.txt REQUIRES arduino`). FreeRTOS was always present underneath — Arduino-ESP32 wraps your `setup()`/`loop()` in a single hidden FreeRTOS task — so v8.0 did **not** "add an RTOS." It exposed direct API access.
- APIs this unlocked, called directly in the firmware: `esp_ota_ops` (`esp_ota_begin`/`write`/`end`, `esp_ota_set_boot_partition`, `esp_ota_get_next_update_partition`) in `ota_service.cpp`; `esp_core_dump` (ELF to flash) in `diagnostics.cpp`; `esp_task_wdt` for the watchdog.
- `sdkconfig` switches now under our control: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` (auto-rollback via pending-verify + `esp_ota_mark_app_valid_cancel_rollback()`), `CONFIG_ESP_TASK_WDT_PANIC`, `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH`.
- Partitions: `default_8MB.csv`, 8 MB flash, dual OTA app slots (~3M each) plus an otadata partition.
- Publish-time validation on our backend checks for a genuine ESP32-S3 image and an embedded `OPPGFWV1` marker baked in by our build (`fw_marker.cpp`), plus a SHA-256 hash — so only firmware we actually built can be published.

This is the second of six in "Inside the OpenPPG Controller." Episode one covered the RTOS that runs your throttle as its own protected job; next up, we take everything this framework change unlocked and put it to work — sending a real firmware update over the air, no cable, and a built-in safety net if anything goes wrong.

## 📌 Caption + hashtags

We didn't switch frameworks for v8.0 — we unlocked the one we were already using. Arduino-for-ESP32 is built on top of ESP-IDF, Espressif's pro framework, so by building as an ESP-IDF project (with Arduino as a component) we kept every library we rely on AND got the keys to the factory controls: native OTA, crash black-box, watchdog panic, and automatic rollback. Same proven engine, full control — and as always, 100% open source on GitHub. 🪂

#paramotor #ppg #electricparamotor #eppg #openppg #electricflight #esp32 #espidf #embeddedsystems #opensource #firmware #freeflight
