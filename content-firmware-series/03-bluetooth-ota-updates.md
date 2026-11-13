# Updating Your Paramotor Over the Air — No Cable, No Brick
*Series: "Inside the OpenPPG Controller" · Episode 3 of 6 · ~2 minutes*

## 🎬 Video Script (~2 min)

**HOOK (0:00–0:10):** Your phone updates itself overnight, no cable, no fuss. Starting with v8.0, your OpenPPG throttle does too, and we built it so a wireless update can't strand you.

| Time | On-screen / B-roll | Spoken (voiceover) |
|------|--------------------|--------------------|
| 0:10–0:25 | Close-up of the controller; phone app pairing screen; small Bluetooth icon animating. ON-SCREEN TEXT: "BLUETOOTH LOW ENERGY · NimBLE" then "FastLink V3 · ~50× per second" | Every flight, your controller already talks to your phone over Bluetooth Low Energy. We run a lean stack called NimBLE, and the everyday link, FastLink V3, streams tightly-packed telemetry your app reads about fifty times a second. |
| 0:25–0:45 | Big title card. ON-SCREEN TEXT: "v8.0 — FIRST OVER-THE-AIR UPDATE". B-roll: hand unplugging a USB cable and setting it down. | And v8.0 is the first OpenPPG firmware you can update over the air. Before this, updating meant plugging into a computer at config.openppg.com. Now your app does it wirelessly, just like your phone updating itself. |
| 0:45–1:05 | Simple motion graphic: cloud → phone → controller, arrows animating. ON-SCREEN TEXT: "ota.openppg.com → your phone → controller (Bluetooth only)" then "Controller never touches the internet" | The pipeline is simple. Our cloud, ota.openppg.com, hosts the firmware. Your app downloads it over the internet, then hands it to the controller over Bluetooth. And here's the part I care about most: the controller itself never touches the internet. It only accepts an update passed to it by your paired phone. |
| 1:05–1:30 | Split-screen list building, each line ticking in. ON-SCREEN TEXT: "WE BUILT IT — version marker + checksum" then "STAGED ROLLOUT · EMERGENCY PAUSE" | So how do we keep a wireless update safe? First, you only get firmware we actually built, because our cloud verifies an embedded version marker and a checksum before anything ships, so nothing else can pose as OpenPPG firmware. Second, we roll new versions out gradually, and we can hit an emergency pause to stop a bad build fleet-wide before most pilots see it. |
| 1:30–1:50 | Animation: two slots, new firmware fills the spare while the running slot glows "active". ON-SCREEN TEXT: "A/B UPDATE — spare slot — falls back to last-good" then "CRC on every chunk" | Third, it's an A/B update. The new firmware writes to a spare slot while the firmware you're flying on stays untouched, so a botched update falls back to the version that was flying fine yesterday. Fourth, every chunk is checksum-verified, so a dropped Bluetooth packet can't quietly corrupt it. |
| 1:50–2:05 | Controller showing ARMED state; an update attempt bounces off with a red "BLOCKED". ON-SCREEN TEXT: "HARD-BLOCKED while armed" then "stalled transfer = clean abort" | Fifth, updates are hard-blocked while the motor is armed or in flight. And if a transfer stalls, it times out and aborts cleanly. |
| 2:05–2:15 | Founder to camera, controller in hand. ON-SCREEN TEXT: "Phone-style convenience. Aviation-grade caution." | That's the whole idea: phone-style convenience, with the caution you'd want from anything you trust in the air. |

**CALL TO ACTION (last ~10s):** All open source, so read the OTA code on our GitHub, and subscribe for the next episode on how the controller survives a crash.

**🎙️ Full voiceover (clean read):** Your phone updates itself overnight, no cable, no fuss. Starting with v8.0, your OpenPPG throttle does too, and we built it so a wireless update can't strand you. Every flight, your controller already talks to your phone over Bluetooth Low Energy. We run a lean stack called NimBLE, and the everyday link, FastLink V3, streams tightly-packed telemetry your app reads about fifty times a second. And v8.0 is the first OpenPPG firmware you can update over the air. Before this, updating meant plugging into a computer at config.openppg.com. Now your app does it wirelessly, just like your phone updating itself. The pipeline is simple. Our cloud, ota.openppg.com, hosts the firmware. Your app downloads it over the internet, then hands it to the controller over Bluetooth. And here's the part I care about most: the controller itself never touches the internet. It only accepts an update passed to it by your paired phone. So how do we keep a wireless update safe? First, you only get firmware we actually built, because our cloud verifies an embedded version marker and a checksum before anything ships, so nothing else can pose as OpenPPG firmware. Second, we roll new versions out gradually, and we can hit an emergency pause to stop a bad build fleet-wide before most pilots see it. Third, it's an A/B update. The new firmware writes to a spare slot while the firmware you're flying on stays untouched, so a botched update falls back to the version that was flying fine yesterday. Fourth, every chunk is checksum-verified, so a dropped Bluetooth packet can't quietly corrupt it. Fifth, updates are hard-blocked while the motor is armed or in flight. And if a transfer stalls, it times out and aborts cleanly. That's the whole idea: phone-style convenience, with the caution you'd want from anything you trust in the air. All open source, so read the OTA code on our GitHub, and subscribe for the next episode on how the controller survives a crash. *(Word count: 330)*

**🅱️ B-roll / shot list:**
- Controller in hand, screen on, Bluetooth icon visible
- Phone app pairing screen and "check for update" / download progress bar
- Hand unplugging a USB cable and setting it aside (the "no cable" beat)
- Motion graphic: cloud → phone → controller pipeline with animated arrows
- Animation of two partition "slots," new firmware filling the spare slot while the active slot glows
- Controller in ARMED state with an update attempt bouncing off ("BLOCKED")
- Founder talking to camera, controller in hand, closing line

**🔤 On-screen text / lower-thirds:**
- "BLUETOOTH LOW ENERGY · NimBLE"
- "FastLink V3 · ~50× per second"
- "v8.0 — FIRST OVER-THE-AIR UPDATE"
- "ota.openppg.com → your phone → controller (Bluetooth only)"
- "Controller never touches the internet"
- "WE BUILT IT — version marker + checksum"
- "STAGED ROLLOUT · EMERGENCY PAUSE"
- "A/B UPDATE — spare slot — falls back to last-good"
- "CRC on every chunk"
- "HARD-BLOCKED while armed"
- "Phone-style convenience. Aviation-grade caution."

## 📝 Companion Blog Post

# No Cable, No Brick: How Your Paramotor Updates Itself Over the Air
*With v8.0, the OpenPPG controller gets its first wireless firmware updates — and we built the whole thing to fail safe.*

I still remember when updating your throttle meant finding a laptop, digging out a USB cable, and plugging into config.openppg.com. It worked, but it was a chore — and chores are the things pilots skip. So for v8.0, we did something we've wanted to do for years: your firmware now updates **over the air**, straight from the phone app, no cable required. Here's exactly how it works, and — more importantly — how we made sure a wireless update can't leave you with a dead throttle.

## The everyday link: Bluetooth, NimBLE, and FastLink V3

Your controller already talks to your phone every flight over **Bluetooth Low Energy**. Under the hood we run a lean, BLE-only stack called **NimBLE**, which uses much less memory than the older Bluetooth stack we started on — memory we reinvest in features like the display, telemetry, and now updates.

The everyday connection is what we call **FastLink V3**: a unified, tightly-packed telemetry feed your app reads roughly **50 times a second**. That tight packing is the trick — it lets your phone show smooth, live instruments (battery, ESC temps, altitude) without flooding the Bluetooth link. Think of it like a stock ticker: instead of your phone constantly asking "battery? battery? battery?", the controller just pushes the new numbers the instant they change.

FastLink is the normal, day-in-day-out link. **Over-the-air updates are a separate, occasional mode** that uses the same Bluetooth connection — just for pushing a new firmware image instead of streaming telemetry.

## The pipeline: cloud → phone → controller

Here's the headline: **v8.0 is the first OpenPPG firmware you can update over the air.** The pipeline is deliberately simple:

1. Our cloud, **ota.openppg.com** (a Cloudflare service), hosts the firmware.
2. Your **app downloads it over the internet** via a short-lived, signed link.
3. Your app then **hands it to the controller over Bluetooth**.

And this is the part I want to underline, because it's a real security decision, not a footnote: **the controller itself never touches the internet.** It has no idea ota.openppg.com exists. It only ever accepts firmware passed to it over Bluetooth by *your paired phone*. That's a much smaller attack surface — the thing strapped to your motor isn't out there talking to the wider internet, it's only listening to a device you've personally bonded to it.

## The safety engineering (this is the part that matters)

Convenience is easy. Convenience that's *safe* is the engineering. Six layers stand between you and a bad update:

**1. We only publish firmware we built.** Before anything can ship from the cloud, our backend verifies it's a genuine ESP32-S3 image, checks an embedded version marker that our build bakes into every binary, and confirms a checksum. Only firmware OpenPPG actually built can be published — nothing else can pose as us.

**2. Staged rollout with an emergency pause.** New versions don't hit everyone at once. They start small and get promoted gradually, and we keep an **emergency pause kill-switch**. If a build turns out to be bad, we can stop it fleet-wide before most pilots ever see it.

**3. A/B updates — a spare slot you're not flying on.** This is the big one. Your controller has two firmware slots. The new firmware writes to the **spare slot while the firmware you're flying on stays completely untouched.** Only after the new image checks out does the controller switch over. Because the firmware you rely on is never overwritten, a botched update simply falls back to the version that was flying fine yesterday — it can't brick your controller. (More on the automatic rollback that backs this up in the next episode.)

**4. Every chunk is checksum-verified.** Bluetooth is a radio link, and radio links drop packets. So the firmware arrives in chunks, and **every chunk gets a CRC checksum** — a math fingerprint that catches any bit that changed in transit. A garbled chunk is rejected and re-requested, so a dropped packet can never quietly become corrupt firmware on your motor.

**5. Hard-blocked while armed.** You **cannot** flash firmware while the motor is armed or in flight. The moment the throttle is live, an update request is rejected outright. No exceptions.

**6. Stalled transfers abort cleanly.** If your phone walks out of range or the app crashes mid-update, the transfer doesn't hang forever. It **times out and aborts cleanly**, leaving your old firmware exactly where it was.

## What this means when you're flying

Practically? Updates stop being a chore you skip and become a thing that just happens — tap, wait a few seconds while disarmed, done. But the deeper payoff is *trust*. You can take an update without that nagging "what if it bricks?" feeling, because the firmware you rely on is never overwritten, every byte is verified, and the motor is locked out of the whole process. If a new build ever misbehaves, the controller falls back to the version that was flying fine yesterday. You get phone-style convenience with the kind of caution you'd want from anything you trust in the air.

## The technical bits (for the nerds 🤓)
*Totally skippable — but if you want the specifics:*

- **BLE stack:** Apache NimBLE via NimBLE-Arduino, migrated off Bluedroid. Modular GATT services, one per file: `config_service`, `fastlink_service`, and `ota_service`.
- **OTA transport:** custom protocol on Espressif's standard OTA UUIDs, built directly on ESP-IDF `esp_ota_ops` (`esp_ota_begin` / `write` / `end` / `set_boot_partition`). The OTA data and command characteristics use BLE **indicate** (acknowledged), distinct from the **notify** path FastLink uses for telemetry. The hybrid Arduino + ESP-IDF build in v8.0 is what unlocks these low-level APIs.
- **A/B partitions:** `esp_ota_get_next_update_partition()` targets the *other* OTA app slot (app0/app1, 3 MB each on an 8 MB flash). The running image is never touched.
- **Integrity:** firmware streams in **4096-byte sectors**, each verified with **CRC16-CCITT** (poly 0x1021, init 0 — matching Espressif's reference Android OTA app). Image size is validated against the partition before any write; on completion, total received bytes must equal the announced length before commit.
- **Robustness:** **30-second idle timeout** auto-aborts a stalled transfer; a failed flash write aborts cleanly with no half-written image; a BLE disconnect mid-transfer triggers an immediate abort.
- **Interlock:** OTA `CMD_START` is rejected if device state is `ARMED` or `ARMED_CRUISING`.
- **Speed:** the link switches to fast connection parameters (~15 ms interval) during a transfer and relaxes afterward; telemetry streaming is paused so it doesn't compete with update packets.

This is the third episode in *Inside the OpenPPG Controller* — and it sets up the next one perfectly. Over-the-air updates are only as safe as what happens if a new build misbehaves, so next time we get into how the controller survives a crash, the software kind, and rolls itself back automatically.

## 📌 Caption + hashtags

Big one for v8.0: your OpenPPG throttle now updates **over the air** — no cable, no laptop, straight from the phone app. We built it to fail safe: the new firmware writes to a spare slot so a bad update falls back to your last-good version, every chunk is CRC-verified, and updates are hard-blocked while you're armed. Phone-style convenience, aviation-grade caution — and it's all open source. 🪂⚡

#paramotor #ppg #electricparamotor #eppg #openppg #electricflight #OTA #firmware #ESP32 #embedded #avgeek #flysafe
