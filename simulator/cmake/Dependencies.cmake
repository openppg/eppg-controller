# Shared offline dependency discovery for all host validation targets.
# Explicit paths make clean worktrees usable without installing PlatformIO.
set(EPPG_FIRMWARE_DEPS "$ENV{EPPG_FIRMWARE_DEPS}" CACHE PATH
    "Root containing the pinned controller protocol/reference libraries")
if(NOT EPPG_FIRMWARE_DEPS)
  foreach(candidate
      "${PROJECT_ROOT}/vendor"
      "${PROJECT_ROOT}/components/vendor"
      "${PROJECT_ROOT}/.pio/libdeps/OpenPPG-CESP32S3-CAN-SP140")
    if(EXISTS "${candidate}/SINE ESC CAN/src/SineEsc.cpp")
      set(EPPG_FIRMWARE_DEPS "${candidate}")
      break()
    endif()
  endforeach()
endif()
if(EXISTS "${PROJECT_ROOT}/components/dronecan/src/canard.c")
  set(_codec_default "${PROJECT_ROOT}/components/dronecan/src")
else()
  set(_codec_default "${EPPG_FIRMWARE_DEPS}/SINE ESC CAN/src")
endif()
set(DRONECAN_DIR "${_codec_default}" CACHE PATH "Pinned plain-C libcanard/generated codec directory")
set(SINE_DIR "${PROJECT_ROOT}/simulator/reference-can/sine" CACHE PATH
    "Pinned original SINE C++ reference wrapper directory")
set(BMS_DIR "${PROJECT_ROOT}/simulator/reference-can/ant" CACHE PATH
    "Pinned original ANT BMS C++ reference parser directory")

function(eppg_require_can_references)
  if(NOT EXISTS "${SINE_DIR}/SineEsc.cpp" OR NOT EXISTS "${BMS_DIR}/BMS_CAN.cpp" OR NOT EXISTS "${DRONECAN_DIR}/canard.c")
    message(FATAL_ERROR
      "Pinned CAN reference sources are missing. Set EPPG_FIRMWARE_DEPS to a "
      "directory containing 'SINE ESC CAN' and 'ANT BMS CAN', or set SINE_DIR "
      "and BMS_DIR individually. See simulator/README.md.")
  endif()
endfunction()
