# Host CAN protocol probe

This isolated executable compiles the repository's cached, unmodified
`SINE ESC CAN` and `ANT BMS CAN` libraries on macOS. Only their hardware
transports are replaced:

- ESP-IDF TWAI calls use in-memory RX/TX queues.
- Adafruit MCP2515 calls consume an in-memory frame queue.
- Arduino `millis()`, `micros()`, `Serial`, `String`, and `SPIClass` have small
  host definitions.

The probe sends a real SINE throttle request, constructs and reassembles a
multi-frame DroneCAN telemetry broadcast, and parses representative ANT BMS
frames through the libraries' public APIs.

From the repository root:

```sh
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol
cmake --build build-host-can-protocol --parallel
./build-host-can-protocol/host_can_protocol_probe
```

This is a practical seam for scenario simulation. It validates real protocol
serialization and parsing, while bus arbitration/error timing still belongs in
a later transport model.
