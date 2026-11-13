# Production Rust controller lab

This native executable shares the firmware's `controller-core`, monitors,
`controller-esc`, DroneCAN transport, ANT parser and `controller-ui`. Both sides
of ESC CAN run the pinned libcanard engine; frames are queued at 1 Mbit/s and BMS
frames at 250 kbit/s. CRC/drop/bus-off fault injection occurs on those bytes.
The Rust LVGL screen renders from the decoded telemetry, not a canned image.

```sh
cd rust
cargo +stable test --locked -p controller-sim
cargo +stable run --locked -p controller-sim -- \
  --scenario ../simulator/scenarios/nominal-flight.csv --duration 10000 \
  --trace /tmp/flight.jsonl --framebuffer /tmp/flight.bmp
```

ESP-IDF/PlatformIO/Arduino are not required for this executable. The native C
compiler, libclang, pinned `components/lvgl` and `components/dronecan` sources are
required for LVGL bindings and libcanard. `--dark` selects the dark screen.

The CSV input/output schemas remain compatible with the browser lab. Additional
actions are `button` (`down`/`up`), `maintenance` (`on`/`off`), `role`
(`standalone`/`receiver`/`remote`), and `wireless_control` with
`session:sequence:raw:state:mode`. Wireless packets are encoded, CRC-validated and
passed to the production receiver state machine; RF propagation/encryption is
outside this functional simulation. State numbers are 0 disarmed, 1 armed,
2 cruise. Sequence/session freshness and the 150 ms failsafe are real production
logic, not independently rewritten simulator behavior.

`arm` and `cruise` are requests through production interlocks. In particular, a
boot-time arm before the 500 ms cooldown is rejected. `button` exercises the
actual click/hold gesture logic. Sample scenarios wait until 600 ms before arm.

Twenty integration tests cover control response, raw CAN latency/CRC/drop/
bus-off recovery, BMS startup grace, unsafe arm thresholds, maintenance exclusion,
button gesture, cruise grace, receiver stale-session rejection/failsafe,
ESC command-loss stop preceding watchdog recovery, telemetry injection,
invalid input and deterministic JSONL traces. Four of those raw-CAN tests run
read-all, single write/save/restart/verify, batched writes and the complete
256 KiB ESC image through production relay/flasher state machines. The full-image
test uses the exact shared C staging implementation shipped on S3, backed by a
NOR model, with one 4 KiB staging sector in RAM. Production Flasher reads the
staged image in 256-byte windows and sends actual DroneCAN frames. Separate
native staging tests inject power loss and retry writes across erase sectors. The synthetic
ESC peer enforces maintenance host node0x40. See `simulator/ARCHITECTURE.md`
for the separate instruction-level QEMU tier and fidelity boundaries.
