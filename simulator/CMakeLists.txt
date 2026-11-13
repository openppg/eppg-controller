cmake_minimum_required(VERSION 3.16)
project(eppg_controller_simulator LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")
eppg_require_can_references()
file(GLOB SINE_GENERATED_C CONFIGURE_DEPENDS "${DRONECAN_DIR}/*.c")

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
  "${DRONECAN_DIR}"
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

add_library(eppg_device_models devices/peripherals.cpp)
target_include_directories(eppg_device_models PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/devices")
target_compile_options(eppg_device_models PRIVATE -Wall -Wextra -Wpedantic)
add_executable(eppg-device-tests devices/tests.cpp)
target_link_libraries(eppg-device-tests PRIVATE eppg_device_models)
add_test(NAME register_device_models COMMAND eppg-device-tests)
