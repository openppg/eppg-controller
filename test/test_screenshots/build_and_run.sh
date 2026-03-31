#!/bin/bash
# Build and run the LVGL screenshot tests
# Usage:
#   ./test/test_screenshots/build_and_run.sh [--local] [--update-references]
#
# --local  Use Ninja (if installed), ccache (if installed), and a portable CPU
#          count for parallel builds. If the build dir was configured without
#          --local, it is cleared once so the generator can switch to Ninja.
#          You can also set EPPG_SCREENSHOT_LOCAL=1 instead of the flag.
#
# Prerequisites: cmake, g++ (or clang++), optional: ninja, ccache
# LVGL and GoogleTest are fetched automatically if not cached.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build-screenshot"

LOCAL_BUILD=0
if [ "${EPPG_SCREENSHOT_LOCAL:-}" = 1 ]; then
  LOCAL_BUILD=1
fi

PASS_TO_TEST=()
for arg in "$@"; do
  case "$arg" in
    --local) LOCAL_BUILD=1 ;;
    *) PASS_TO_TEST+=("$arg") ;;
  esac
done

UPDATE_REFS=0
for arg in "${PASS_TO_TEST[@]}"; do
  if [ "$arg" = "--update-references" ]; then
    UPDATE_REFS=1
    break
  fi
done

# Parallel jobs: full portable count only with --local; CI keeps nproc || 4 (Linux).
build_parallel_jobs() {
  if [ "$LOCAL_BUILD" != 1 ]; then
    nproc 2>/dev/null || echo 4
    return
  fi
  if [ -n "${EPPG_SCREENSHOT_BUILD_JOBS:-}" ]; then
    echo "$EPPG_SCREENSHOT_BUILD_JOBS"
    return
  fi
  nproc 2>/dev/null && return
  getconf _NPROCESSORS_ONLN 2>/dev/null && return
  sysctl -n hw.ncpu 2>/dev/null && return
  echo 4
}

CMAKE_GEN_ARGS=()
CMAKE_CCACHE_ARGS=()
if [ "$LOCAL_BUILD" = 1 ]; then
  if command -v ninja >/dev/null 2>&1; then
    CMAKE_GEN_ARGS=(-G Ninja)
  else
    echo "Warning: --local: 'ninja' not found; using CMake's default generator." >&2
  fi
  if command -v ccache >/dev/null 2>&1; then
    CMAKE_CCACHE_ARGS=(
      -DCMAKE_C_COMPILER_LAUNCHER=ccache
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    )
  else
    echo "Warning: --local: 'ccache' not found; building without compiler cache." >&2
  fi
fi

echo "=== LVGL Screenshot Tests ==="
echo "Project root: $PROJECT_ROOT"
echo "Build dir:    $BUILD_DIR"
if [ "$LOCAL_BUILD" = 1 ]; then
  ninja_status="default"
  [ "${#CMAKE_GEN_ARGS[@]}" -gt 0 ] && ninja_status="Ninja"
  ccache_status="off"
  [ "${#CMAKE_CCACHE_ARGS[@]}" -gt 0 ] && ccache_status="on"
  echo "Local fast path: generator=${ninja_status}, ccache=${ccache_status}, jobs=$(build_parallel_jobs)"
fi

mkdir -p "$BUILD_DIR"

# If the cached generator or build type no longer matches, clear the build dir once.
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
  cached_gen=$(grep '^CMAKE_GENERATOR:INTERNAL=' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
  cached_type=$(grep '^CMAKE_BUILD_TYPE:STRING=' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
  want_gen="Unix Makefiles"
  [ "${#CMAKE_GEN_ARGS[@]}" -gt 0 ] && want_gen="Ninja"
  if [ "$cached_gen" != "$want_gen" ] || [ "$cached_type" != "$BUILD_TYPE" ]; then
    echo "Clearing $BUILD_DIR (generator/build type changed: ${cached_gen}/${cached_type} -> ${want_gen}/${BUILD_TYPE})..."
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
  fi
fi

cd "$BUILD_DIR"

echo ""
echo "--- Configuring CMake ---"
# --local uses RelWithDebInfo: optimized binary (5-10x faster renderer) with debug symbols.
# CI keeps Debug for faithful failure diagnostics.
BUILD_TYPE="Debug"
[ "$LOCAL_BUILD" = 1 ] && BUILD_TYPE="RelWithDebInfo"
# shellcheck disable=SC2086
cmake "$SCRIPT_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  "${CMAKE_GEN_ARGS[@]}" \
  "${CMAKE_CCACHE_ARGS[@]}" \
  ${LVGL_DIR:+-DLVGL_DIR="$LVGL_DIR"} \
  ${GTEST_DIR:+-DGTEST_DIR="$GTEST_DIR"}

echo ""
echo "--- Building ---"
cmake --build . --parallel "$(build_parallel_jobs)"

echo ""
echo "--- Running tests ---"
cd "$PROJECT_ROOT"
mkdir -p test/test_screenshots/output
mkdir -p test/test_screenshots/reference

if [ "$UPDATE_REFS" = 1 ]; then
  echo "Updating reference screenshots..."
  rm -rf test/test_screenshots/reference/*.bmp
fi

"$BUILD_DIR/screenshot_tests" "${PASS_TO_TEST[@]}"

echo ""
echo "--- Screenshots ---"
echo "Output:    test/test_screenshots/output/"
echo "Reference: test/test_screenshots/reference/"
echo ""
ls -la test/test_screenshots/output/*.bmp 2>/dev/null | awk '{print $NF, $5}'
