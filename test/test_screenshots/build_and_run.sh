#!/bin/bash
# Build and run the LVGL screenshot tests
# Usage: ./test/test_screenshots/build_and_run.sh [--update-references]
#
# Prerequisites: cmake, g++ (or clang++)
# LVGL and GoogleTest are fetched automatically if not cached.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build-screenshot"

echo "=== LVGL Screenshot Tests ==="
echo "Project root: $PROJECT_ROOT"
echo "Build dir:    $BUILD_DIR"

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake (FetchContent will download LVGL and GoogleTest if needed)
if [ ! -f "Makefile" ] && [ ! -f "build.ninja" ]; then
  echo ""
  echo "--- Configuring CMake ---"
  cmake "$SCRIPT_DIR" \
    -DCMAKE_BUILD_TYPE=Debug \
    ${LVGL_DIR:+-DLVGL_DIR="$LVGL_DIR"} \
    ${GTEST_DIR:+-DGTEST_DIR="$GTEST_DIR"}
fi

# Build
echo ""
echo "--- Building ---"
cmake --build . --parallel "$(nproc 2>/dev/null || echo 4)"

# Run tests from project root (paths are relative)
echo ""
echo "--- Running tests ---"
cd "$PROJECT_ROOT"
mkdir -p test/test_screenshots/output
mkdir -p test/test_screenshots/reference

if [ "$1" = "--update-references" ]; then
  echo "Updating reference screenshots..."
  rm -rf test/test_screenshots/reference/*.bmp
fi

"$BUILD_DIR/screenshot_tests" "$@"

echo ""
echo "--- Screenshots ---"
echo "Output:    test/test_screenshots/output/"
echo "Reference: test/test_screenshots/reference/"
echo ""
ls -la test/test_screenshots/output/*.bmp 2>/dev/null | awk '{print $NF, $5}'
