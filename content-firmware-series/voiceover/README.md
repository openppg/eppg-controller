# Voiceover scripts (ElevenLabs-ready)

Word-for-word narration for each episode, marked up for **ElevenLabs text-to-speech**
so you can generate an audio-only rough cut. The wording is identical to the
"Full voiceover (clean read)" in each episode file — I only added delivery markup
(no words changed). Each `.txt` contains **only** the narration, so you can
select-all → paste straight into ElevenLabs.

| Episode | File |
|---|---|
| 1 — RTOS | [01-what-is-an-rtos.elevenlabs.txt](01-what-is-an-rtos.elevenlabs.txt) |
| 2 — ESP-IDF | [02-why-esp-idf.elevenlabs.txt](02-why-esp-idf.elevenlabs.txt) |
| 3 — OTA updates | [03-bluetooth-ota-updates.elevenlabs.txt](03-bluetooth-ota-updates.elevenlabs.txt) |
| 4 — Reliability | [04-reliability-hardening.elevenlabs.txt](04-reliability-hardening.elevenlabs.txt) |
| 5 — Evolution | [05-controller-evolution.elevenlabs.txt](05-controller-evolution.elevenlabs.txt) |
| 6 — CAN bus | [06-can-bus.elevenlabs.txt](06-can-bus.elevenlabs.txt) |

## The markup (tuned for Eleven v3)

Per ElevenLabs' [best-practices guide](https://elevenlabs.io/docs/overview/capabilities/text-to-speech/best-practices):

- **`[audio tags]`** in square brackets are **v3 delivery cues** — they are *not
  spoken*, they steer how the line is read. I used them **sparingly** (1–2 per
  script) at natural beats: `[curious]`, `[excited]`, `[whispers]`, `[sighs]`.
  Match them to your own voice — **delete any that feel off**; they're seasoning,
  not structure.
- **`…` (ellipsis)** and **`—` (em dash)** = pauses / pacing. v3 has **no
  `<break>` tag support**, so these are how you get a beat. The scripts already
  lean on em dashes the way you'd actually talk.
- **CAPS** = emphasis on a key word (e.g. *HIGHEST*, *PHYSICALLY*, *FIRST*,
  *NEVER*). Used lightly — too much reads as shouting.

ElevenLabs warns that **overusing tags causes instability**, so I kept them
minimal. If a long script drifts in tone, generate it a paragraph (or two) at a
time and stitch the audio — that's normal for longer reads.

## If you use a v2 model instead of v3

v2 does **not** understand the square-bracket tags — it will *read them out loud*.
So if you pick a v2 voice/model:

1. **Delete every `[tag]`** from the script.
2. (Optional) For precise pauses, insert break tags where the `…`/`—` are:
   `<break time="0.4s" />` (v2 supports up to `3s`; don't use many — too many
   `<break>`s also cause instability).

## Pronunciation cheat-sheet

TTS often mangles acronyms and product names. If a word comes out wrong,
replace it in the script with the say-as spelling, or use v3 IPA (`/…/`):

| Written | Say it as |
|---|---|
| EICAS | "EYE-kass" |
| NimBLE | "nimble" (one word) |
| LVGL | spell it: "L · V · G · L" |
| ESP-IDF | "E-S-P · I-D-F" |
| ESP32-S3 | "E-S-P thirty-two · S-three" |
| TWAI | "TWA-eye" (or just say "the CAN controller") |
| BLE | "B-L-E" |
| CAN bus | "can bus" (rhymes with *can*) — **not** "C-A-N" |
| ESC | "E-S-C" |
| BMS | "B-M-S" |
| RPM | "R-P-M" |
| v7.0 / v8.0 | "version seven point oh" / "version eight point oh" |
| v6.0 | "version six point oh" |
| config.openppg.com / ota.openppg.com | "config dot open-P-P-G dot com" / "O-T-A dot open-P-P-G dot com" |
| A/B update | "A-B update" |

Tip: a quick way to lock pronunciation in v3 is inline IPA, e.g. write
`EICAS (/ˈaɪkæs/)` the first time, then plain `EICAS` after.

---
*These are rough-cut drafts. The words are yours verbatim; the tags/CAPS/pauses
are suggestions — tweak to taste before you render.*
