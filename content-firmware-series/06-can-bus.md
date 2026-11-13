# CAN Bus: How Your Throttle, Motor, and Battery Talk to Each Other
*Series: "Inside the OpenPPG Controller" · Episode 6 of 6 · ~2 minutes*

## 🎬 Video Script (~2 min)

**HOOK (0:00–0:10):** For years, your throttle just shouted one dumb number down a wire — and the motor and battery couldn't shout back. In 2025, we gave them a real conversation. It's the same tech your car uses to keep its engine, brakes, and airbags talking.

| Time | On-screen / B-roll | Spoken (voiceover) |
|------|--------------------|--------------------|
| 0:10–0:25 | Close-up of old analog throttle wire; simple needle gauge animation. ON-SCREEN: "OLD WAY: ONE DUMB SIGNAL" | Here's the old way. An analog throttle just sent a single voltage down a wire — "give me this much power." The motor obeyed, but it couldn't really tell you anything back. Three parts, basically strangers. |
| 0:25–0:45 | Animation: thick wire bundle collapsing into two thin wires; multiple "nodes" lighting up in turn. ON-SCREEN: "CAN BUS · BOSCH · 1980s" | So we switched to CAN bus. Controller Area Network — a rugged two-wire digital network Bosch invented for cars back in the 1980s. It's in basically every vehicle, plus aircraft and industrial gear. Lots of devices share one pair of wires, take turns by priority, with built-in error checking. |
| 0:45–1:05 | Diagram: controller in center, CAN lines to ESC (motor) and BMS (battery). ON-SCREEN: "v7.0 · 2025 · DIGITAL NERVOUS SYSTEM" | In our v7.0 ESP32-S3 release, the controller now runs CAN to both the motor's speed controller and the battery's management system. That bundle of analog wires? Gone. In its place, one small, reliable digital backbone — a real nervous system for your paramotor. |
| 1:05–1:25 | Screen telemetry filling in: RPM, voltage, temps, cell data. Then arrows flipping direction back to ESC. ON-SCREEN: "IT'S TWO-WAY" | And here's the best part: it's two-way. We don't just read rich telemetry — RPM, voltage, temperatures, even cell-level battery data. We write back to the motor over CAN. Your throttle goes out as a precise digital value. We configure the ESC at startup, and we even drive its status light colors and beep tones, from the controller in your hand. |
| 1:25–1:45 | Animation: heartbeat pulse every fraction of a second; pulse stops, motor spins down. ON-SCREEN: "NO SIGNAL = MOTOR STOPS (BY DESIGN)" | There's a safety nuance pilots love. The motor controller expects a throttle message about every 300 milliseconds — or it stops. So that steady heartbeat isn't just data, it's a safety feature. A silent or disconnected controller means the motor shuts down, by design. |
| 1:45–1:55 | Montage: telemetry on screen, EICAS yellow/red alert flashing. ON-SCREEN: "SAME SIGNALS → YOUR ALERTS" | This is also what powers those EICAS cockpit alerts we covered earlier — those warnings ride in over CAN. Motor, battery, throttle: one connected system, not three separate parts. |

**CALL TO ACTION (last ~10s):** Our CAN libraries — SINE-ESC-CAN and ANT-BMS-CAN — are open source on our GitHub. Go dig in, and fly safe.

**🎙️ Full voiceover (clean read):** For years, your throttle just shouted one dumb number down a wire — and the motor and battery couldn't shout back. In 2025, we gave them a real conversation. It's the same tech your car uses to keep its engine, brakes, and airbags talking. Here's the old way. An analog throttle just sent a single voltage down a wire — "give me this much power." The motor obeyed, but it couldn't really tell you anything back. Three parts, basically strangers. So we switched to CAN bus. Controller Area Network — a rugged two-wire digital network Bosch invented for cars back in the 1980s. It's in basically every vehicle, plus aircraft and industrial gear. Lots of devices share one pair of wires, take turns by priority, with built-in error checking. In our v7.0 ESP32-S3 release, the controller now runs CAN to both the motor's speed controller and the battery's management system. That bundle of analog wires? Gone. In its place, one small, reliable digital backbone — a real nervous system for your paramotor. And here's the best part: it's two-way. We don't just read rich telemetry — RPM, voltage, temperatures, even cell-level battery data. We write back to the motor over CAN. Your throttle goes out as a precise digital value. We configure the ESC at startup, and we even drive its status light colors and beep tones, from the controller in your hand. There's a safety nuance pilots love. The motor controller expects a throttle message about every 300 milliseconds — or it stops. So that steady heartbeat isn't just data, it's a safety feature. A silent or disconnected controller means the motor shuts down, by design. This is also what powers those EICAS cockpit alerts we covered earlier — those warnings ride in over CAN. Motor, battery, throttle: one connected system, not three separate parts. Our CAN libraries are open source on our GitHub. Go dig in, and fly safe. *(Word count: 315)*

**🅱️ B-roll / shot list:**
- Macro shot of the old single analog throttle wire / connector
- Animation: thick analog wire bundle morphing into a clean two-wire pair
- Diagram of controller as hub with CAN links to ESC and BMS
- Live telemetry filling the color display (RPM, voltage, temps)
- ESC onboard LED changing color (green/yellow/red) and a beep sound cue
- Heartbeat pulse animation that stops, followed by motor spin-down
- EICAS yellow then red alert flashing on the display with haptic buzz suggestion
- GitHub repo page scroll showing SINE-ESC-CAN and ANT-BMS-CAN

**🔤 On-screen text / lower-thirds:**
- "Zach Whitehead · Co-founder, OpenPPG"
- "OLD WAY: ONE DUMB ANALOG SIGNAL"
- "CAN BUS · Invented by Bosch · 1980s"
- "v7.0 (2025) · ESP32-S3 · Full CAN Integration"
- "IT'S TWO-WAY: read AND write"
- "No message every ~300ms = motor stops (by design)"
- "Open source: SINE-ESC-CAN · ANT-BMS-CAN"

## 📝 Companion Blog Post

# CAN Bus: Why Your Throttle, Motor, and Battery Finally Speak the Same Language
*The 2025 upgrade that turned three separate parts into one connected digital cockpit — using the same proven backbone your car and aircraft rely on.*

If you drive an EV or have ever watched a mechanic plug a laptop into a car, you've already met the technology in this episode — even if you didn't know its name. It's called CAN bus, and in our v7.0 release it became the digital nervous system of the OpenPPG controller. This is the last episode in the series, and honestly it might be my favorite, because CAN is the thing that makes everything else we've talked about possible.

## The problem: a throttle that could only shout

For most of paramotoring history, a throttle did exactly one thing. You squeezed it, and it sent a single analog signal — basically a voltage — down a wire to the motor: "give me this much power." That's it. One number, one direction.

The motor obeyed, but it couldn't meaningfully talk back. The battery was off in its own corner entirely. So you had three critical components — throttle, motor controller, and battery — behaving like strangers who happened to be bolted to the same frame. If you wanted to know what the motor was actually doing, or how each battery cell was holding up, that information had nowhere to go. Add in a high-power motor pumping out electrical noise right next to thin signal wires, and "dumb analog" starts to feel like a real limitation.

## What CAN bus actually is

CAN stands for Controller Area Network. Bosch invented it for cars in the 1980s, and today it's in basically every vehicle on the road, plus aircraft, industrial machines, and EVs. It's a rugged, noise-resistant, two-wire digital network. Here's the key idea: instead of running a separate wire to every component, many devices ("nodes") all share one pair of wires. They take turns sending messages by priority, and the whole thing has built-in error checking and automatic retransmission.

The car analogy is perfect here. CAN bus is the same kind of network that lets your car's engine, ABS brakes, and airbag system all coordinate with each other over a shared connection. It replaces a thick, failure-prone bundle of individual wires with one small, reliable digital backbone. That's exactly what we wanted for the controller.

## How we use it: two CAN links, one nervous system

CAN arrived with our v7.0 release in May 2025 — the ESP32-S3 generation, our largest update ever. The controller now runs CAN to two things at once:

1. **The motor's ESC** (the electronic speed controller) — this rides on the ESP32-S3's built-in CAN controller.
2. **The battery's BMS** (battery management system) — this uses a separate dedicated CAN chip.

Both are standard CAN bus. Together they replaced the old bundle of analog wiring with one clean digital backbone. Fewer wires and connectors means fewer things that can fail, and the digital design shrugs off the electrical noise that surrounds a powerful motor.

## The best part: it's a two-way conversation

This is the headline. CAN isn't just a better way to listen — it's a real back-and-forth.

**What we read:** From the ESC, we pull live RPM, voltage, current, multiple temperature readings, and runtime. From the BMS, we get state-of-charge, pack voltage, current, per-cell voltages, temperatures, and cycle count. That's the rich telemetry you see on the color display and stream to your phone.

**What we write:** This is what surprises people. The controller writes back to the motor over CAN. Every throttle movement goes out as a precise digital value — not a fuzzy voltage, but an exact command. At startup, the controller configures the ESC with sensible defaults and a safe idle. And we even command the ESC's onboard status LED — its color and gentle breathing pattern for arm, disarm, and alerts — plus its beeps and tones, all from the controller in your hand.

## What this means when you're flying

Here's the safety nuance pilots love. The ESC expects to hear a throttle message at least every ~300 milliseconds. If it doesn't, it stops. That means the steady CAN heartbeat from your controller is itself a safety feature: a disconnected, frozen, or silent controller results in the motor shutting down, by design. Silence isn't ambiguous — it's a stop command.

And remember the EICAS-style cockpit alerts from Episode 4? Those 40+ live signals from the motor, the battery, and the controller's own internals — the yellow warnings and red criticals that buzz in your hand — they all arrive over CAN. Without this digital backbone, those alerts simply couldn't exist.

## The technical bits (for the nerds 🤓)
*Totally skippable if you just want to fly.*

- The ESC link uses the ESP32-S3's built-in CAN controller, which Espressif brands "TWAI" (Two-Wire Automotive Interface). To be clear: TWAI is just Espressif's name for standard CAN — not a different protocol. In firmware it's `driver/twai.h`, with CAN RX on pin 3 and TX on pin 2.
- The BMS link uses a separate MCP2515 CAN-controller chip on the SPI bus, made thread-safe with a FreeRTOS mutex.
- It's a node-addressed protocol: the controller sets a local node id, and the ESC answers at node id 0x20.
- Throttle commands go out via `setThrottleSettings2` at 0.1-microsecond resolution.
- The two open-source libraries are `openppg/SINE-ESC-CAN` (motor) and `openppg/ANT-BMS-CAN` (battery), both on our GitHub.

A quick, honest note: this is not certified avionics. We borrow proven automotive and aviation tech and apply it carefully — but we never claim certification.

That wraps our six-part tour inside the OpenPPG controller — and CAN bus is the thread that ties it together. It's why your motor, battery, and throttle behave like one connected system instead of three separate parts, on the same backbone cars and aircraft have trusted for decades. Thanks for flying along.

## 📌 Caption + hashtags

Your old throttle could only shout one dumb number down a wire. In our 2025 v7.0 release we gave your motor, battery, and throttle a real two-way conversation over CAN bus — the same rugged network Bosch built for cars in the 1980s. We don't just read RPM, voltage, and cell-level battery data; we send throttle, configure the motor, and even drive its lights and tones over CAN. And the heartbeat is a safety feature: no message every ~300ms, and the motor stops by design. Libraries are open source on our GitHub. 🪂

#paramotor #PPG #electricparamotor #openppg #canbus #ESP32 #opensource #aviationtech #flyelectric #embeddedsystems
