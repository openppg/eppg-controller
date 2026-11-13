cmake_minimum_required(VERSION 3.16)
project(eppg_controller_simulator LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(FIRMWARE_DEPS "${PROJECT_ROOT}/.pio/libdeps/OpenPPG-CESP32S3-CAN-SP140")
set(SINE_DIR "${FIRMWARE_DEPS}/SINE ESC CAN/src")
set(BMS_DIR "${FIRMWARE_DEPS}/ANT BMS CAN")
if(NOT EXISTS "${SINE_DIR}/SineEsc.cpp" OR NOT EXISTS "${BMS_DIR}/BMS_CAN.cpp")
  message(FATAL_ERROR
    "Firmware CAN libraries are missing; run 'pio run -e OpenPPG-CESP32S3-CAN-SP140' first")
endif()
file(GLOB SINE_GENERATED_C CONFIGURE_DEPENDS "${SINE_DIR}/*.c")

add_library(eppg_sim_core
  controller_sim.cpp
  hardware/virtual_hardware.cpp
  "${PROJECT_ROOT}/src/sp140/throttle.cpp"
  "${SINE_DIR}/CanardAdapter.cpp"
  "${SINE_DIR}/SineEsc.cpp"
  ${SINE_GENERATED_C}
  "${BMS_DIR}/BMS_CAN.cpp"
)
target_include_directories(eppg_sim_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/hardware"
  "${SINE_DIR}"
  "${BMS_DIR}"
  "${PROJECT_ROOT}/inc"
)
target_compile_options(eppg_sim_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(eppg-sim main.cpp)
target_link_libraries(eppg-sim PRIVATE eppg_sim_core)

add_executable(eppg-sim-tests tests.cpp)
target_link_libraries(eppg-sim-tests PRIVATE eppg_sim_core)

enable_testing()
add_test(NAME controller_simulator COMMAND eppg-sim-tests)
