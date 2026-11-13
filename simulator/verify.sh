#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
cd "$project_root"
host_only=0
rust=0
qemu=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --host-only) host_only=1 ;;
    --rust) rust=1 ;;
    --qemu) qemu=1 ;;
    *) echo "Usage: $0 [--host-only] [--rust] [--qemu]" >&2; exit 2 ;;
  esac
  shift
done
jobs="${EPPG_BUILD_JOBS:-4}"
if [[ "$host_only" = 0 ]]; then
  if [[ "$rust" = 1 ]]; then
    bash rust/tools/build.sh
  else
    IDF_COMPONENT_MANAGER=0 idf.py -B build-idf build
    python3 tools/check_native_release.py build-idf
  fi
fi
cmake -S tools/host-native-tests -B build-host-native-tests -DCMAKE_BUILD_TYPE=Release
cmake --build build-host-native-tests --parallel "$jobs"
ctest --test-dir build-host-native-tests --output-on-failure
./simulator/run.sh --native --test-only
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol -DCMAKE_BUILD_TYPE=Release
cmake --build build-host-can-protocol --parallel "$jobs"
./build-host-can-protocol/host_can_protocol_probe
cmake -S tools/host-screenshot-offline -B build-host-screenshots -DCMAKE_BUILD_TYPE=Release
cmake --build build-host-screenshots --parallel "$jobs"
ctest --test-dir build-host-screenshots --output-on-failure
if [[ "$rust" = 1 ]]; then
  (
    cd rust
    # espup's libclang is for Xtensa; native bindgen uses the host installation.
    unset LIBCLANG_PATH
    cargo +stable test --locked --workspace --exclude firmware-sp140
    cargo +stable clippy --locked --workspace --exclude firmware-sp140 --all-targets -- -D warnings
    cargo +stable build --locked -p controller-sim
  )
  EPPG_SIM_BACKEND=rust node --test simulator/ui/server.test.mjs
else
  EPPG_SIM_BACKEND=native EPPG_SIMULATOR_BINARY="$project_root/build-simulator/eppg-sim" \
    node --test simulator/ui/server.test.mjs
fi
if [[ "$qemu" = 1 ]]; then
  if [[ "$rust" = 1 ]]; then
    bash rust/tools/build.sh --qemu
    qemu_image="$project_root/rust/target/qemu/xtensa-esp32s3-espidf/release/images/qemu-flash.bin"
    python3 simulator/tools/qemu_smoke.py --image "$qemu_image" --control-proof \
      --expect 'Rust controller boot complete' --expect 'pwm_01us=16000' \
      --report rust/target/qemu/smoke.json
    python3 simulator/tools/qemu_smoke.py --image "$qemu_image" \
      --send-after 'Rust controller boot complete' --send '{"emulator":{"stage_test":true}}' \
      --expect 'EPPG_QEMU_STAGE size=262144 verified=1 scratch=4096' \
      --report rust/target/qemu/staging.json
  else
    ./simulator/tools/run_qemu.sh --build --smoke
  fi
fi
python3 -m unittest discover -s simulator/tools -p test_qemu_smoke.py
echo "All requested build/emulator checks passed."
