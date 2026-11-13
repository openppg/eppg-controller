#!/usr/bin/env bash
set -euo pipefail
apt-get update
apt-get install -y --no-install-recommends cmake ninja-build g++ libgtest-dev libusb-1.0-0-dev
. /opt/esp/idf/export.sh
python -m pip install cpplint
printf '%s\n' 'ESP-IDF 5.5.3 environment ready.' \
  'Build: IDF_COMPONENT_MANAGER=0 idf.py -B build-idf build' \
  'Validate: python3 tools/check_native_release.py build-idf' \
  'Run all gates: GTEST_DIR=/usr/src/googletest ./simulator/verify.sh'
