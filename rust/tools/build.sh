#!/usr/bin/env bash
# Requires espup's environment and an activated, pinned ESP-IDF 5.5.3 SDK.
set -euo pipefail
cd "$(dirname "$0")/.."
export CARGO_WORKSPACE_DIR="$PWD"
profile=physical
args=(--locked --release -Zbuild-std=std,panic_abort -p firmware-sp140 --target xtensa-esp32s3-espidf)
if [[ "${1:-}" == --qemu ]]; then
  profile=qemu
  export EPPG_QEMU=1
  export ESP_IDF_SDKCONFIG_DEFAULTS='sdkconfig.defaults;sdkconfig.qemu.defaults'
  export CARGO_TARGET_DIR="$PWD/target/qemu"
  args+=(--features qemu)
else
  export ESP_IDF_SDKCONFIG_DEFAULTS=sdkconfig.defaults
  unset EPPG_QEMU
fi
cargo +esp build "${args[@]}"
python3 tools/export_image.py "${CARGO_TARGET_DIR:-$PWD/target}/xtensa-esp32s3-espidf/release" --profile "$profile"
