# Only the low-level C ABI IDF peripheral calls are retained from the IDF branch.
# No C++ controller, services, UI, protocol orchestration or startup is linked.
if(DEFINED ENV{EPPG_COMPONENTS})
  get_filename_component(EPPG_SOURCE_ROOT "$ENV{EPPG_COMPONENTS}/.." ABSOLUTE)
else()
  get_filename_component(EPPG_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
endif()
idf_component_register(SRCS "${EPPG_SOURCE_ROOT}/components/controller_platform/peripheral.cpp" "bmp_bridge.c" "${EPPG_SOURCE_ROOT}/components/controller_platform/staging.c" "${EPPG_SOURCE_ROOT}/components/image_staging/image_staging.c" "${EPPG_SOURCE_ROOT}/components/bmp3/bmp3.c"
 INCLUDE_DIRS "${EPPG_SOURCE_ROOT}/inc" "${EPPG_SOURCE_ROOT}/components/bmp3" "${EPPG_SOURCE_ROOT}/components/image_staging" REQUIRES driver esp_adc esp_timer esp_system esp_partition freertos log json)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
if("$ENV{EPPG_QEMU}" STREQUAL "1")
 file(GLOB QEMU_CODEC_SOURCES "${EPPG_SOURCE_ROOT}/components/dronecan/src/sine*.c")
 target_sources(${COMPONENT_LIB} PRIVATE ${QEMU_CODEC_SOURCES} "${EPPG_SOURCE_ROOT}/simulator/qemu_backend.cpp"
  "${EPPG_SOURCE_ROOT}/simulator/devices/peripherals.cpp")
 target_include_directories(${COMPONENT_LIB} PRIVATE "${EPPG_SOURCE_ROOT}/simulator" "${EPPG_SOURCE_ROOT}/components/dronecan/src")
 target_compile_definitions(${COMPONENT_LIB} PRIVATE EPPG_QEMU=1 EPPG_RUST=1)
 foreach(function driver_install start stop get_status_info reconfigure_alerts read_alerts receive transmit initiate_recovery driver_uninstall)
  target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=twai_${function}")
 endforeach()
endif()
