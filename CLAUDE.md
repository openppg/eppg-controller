# Controller development

Production firmware on this branch is the Cargo-built Rust application under
`rust/apps/firmware-sp140`, targeting `xtensa-esp32s3-espidf` on ESP-IDF 5.5.3.
Arduino and PlatformIO are not part of the build. The preceding branch,
`codex/esp-idf-no-arduino`, contains the native C++ migration; `src/sp140` remains
an independent compatibility reference here and is excluded from the Rust image.

Use the pinned toolchains and environment in `rust/README.md`, then run:

```sh
bash rust/tools/build.sh
bash rust/tools/build.sh --qemu
```

The exporter checks OTA slot size, firmware marker, unchanged partition geometry,
and absence of Arduino/legacy C++ application objects. Archive matching ELF/map
files. App-only OTA and full recovery images are distinct artifacts.

Host regressions are documented in `tools/host-native-tests/README.md`; build with
CMake and run CTest. `./simulator/verify.sh` runs all firmware/host gates.
`tools/host-screenshot-offline` compares production LVGL 9.6 screens with existing
references and fails on missing/different references. QEMU instructions are in README.

Production controller state, monitoring, byte protocols, ESC sessions and UI live
in `rust/crates/`. The native simulator uses those same crates. Host verification:

```sh
cd rust
unset LIBCLANG_PATH
cargo +1.100.0 test --locked --workspace --exclude firmware-sp140
cargo +1.100.0 clippy --locked --workspace --exclude firmware-sp140 --all-targets -- -D warnings
```

Use system libclang for native builds and espup's libclang for S3 builds. Format
included UI source files explicitly: cargo fmt does not discover include! files.
The existing 34 screenshot fixtures are parity gates; do not automatically bless
changed or missing references. The C++ CMake tests and pinned protocol oracles
remain separate regression references. Browser/QEMU instructions are in README.

Preserve TWAI ESC at 1 Mbps, MCP2515 BMS at 250 kbps, the 50 Hz control deadline,
wire bytes/UUIDs and NVS representations. The control task owns ESC/CAN; other
workers enqueue bounded typed requests. RF and update workers must not extend
stale throttle commands or block control. Set thread stacks with Builder explicitly;
plain std::thread::spawn overrides the configured IDF stack with a 3 KiB default.
Native/QEMU success does not establish RF or physical hardware qualification.
