#!/usr/bin/env bash
set -euo pipefail
apt-get update
apt-get install -y --no-install-recommends cmake ninja-build g++ clang libclang-dev libgtest-dev libusb-1.0-0-dev curl pkg-config
if ! command -v rustup >/dev/null 2>&1; then
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/eppg-rustup-init.sh
  sh /tmp/eppg-rustup-init.sh -y --default-toolchain 1.100.0 --profile minimal
fi
. "$HOME/.cargo/env"
rustup toolchain install 1.100.0 --profile minimal --component rustfmt --component clippy
cargo +1.100.0 install --locked espup --version 0.19.0
espup install --toolchain-version 1.97.0.0 --targets esp32s3 --std --export-file "$HOME/export-esp.sh"
cargo +1.100.0 install --locked ldproxy --version 0.3.5
. /opt/esp/idf/export.sh
printf '%s\n' 'Rust / ESP-IDF 5.5.3 toolchains installed.' \
  'Firmware: source /opt/esp/idf/export.sh && source ~/export-esp.sh && bash rust/tools/build.sh' \
  'Host tests: unset LIBCLANG_PATH; cd rust; cargo +1.100.0 test --locked --workspace --exclude firmware-sp140' \
  'Browser lab: ./simulator/run.sh --rust --test-only; cd simulator/ui; npm ci; npm start'
