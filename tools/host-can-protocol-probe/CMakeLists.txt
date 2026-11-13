cmake_minimum_required(VERSION 3.16)
project(eppg_host_can_protocol_probe LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${PROJECT_ROOT}/simulator/cmake/Dependencies.cmake")
eppg_require_can_references()
file(GLOB SINE_GENERATED_C CONFIGURE_DEPENDS "${DRONECAN_DIR}/*.c")

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
  "${DRONECAN_DIR}"
  "${BMS_DIR}"
)

target_compile_options(host_can_protocol_probe PRIVATE -Wall -Wextra)

