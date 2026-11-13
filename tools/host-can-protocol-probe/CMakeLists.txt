cmake_minimum_required(VERSION 3.16)
project(eppg_host_can_protocol_probe LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(DEPS "${PROJECT_ROOT}/.pio/libdeps/OpenPPG-CESP32S3-CAN-SP140")
set(SINE_DIR "${DEPS}/SINE ESC CAN/src")
set(BMS_DIR "${DEPS}/ANT BMS CAN")

if(NOT EXISTS "${SINE_DIR}/SineEsc.cpp" OR NOT EXISTS "${BMS_DIR}/BMS_CAN.cpp")
  message(FATAL_ERROR "Firmware libraries are missing; run the normal PlatformIO firmware build first")
endif()

file(GLOB SINE_GENERATED_C CONFIGURE_DEPENDS "${SINE_DIR}/*.c")

add_executable(host_can_protocol_probe
  probe.cpp
  fake_can.cpp
  "${SINE_DIR}/CanardAdapter.cpp"
  "${SINE_DIR}/SineEsc.cpp"
  ${SINE_GENERATED_C}
  "${BMS_DIR}/BMS_CAN.cpp"
)

# Stubs intentionally come first. They replace only Arduino time/serial,
# ESP-IDF TWAI, and MCP2515 transport surfaces; the protocol code is real.
target_include_directories(host_can_protocol_probe PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/stubs"
  "${SINE_DIR}"
  "${BMS_DIR}"
)

target_compile_options(host_can_protocol_probe PRIVATE -Wall -Wextra)

