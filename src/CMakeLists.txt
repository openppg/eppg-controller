file(GLOB_RECURSE CONTROLLER_SOURCES "sp140/*.cpp")
idf_component_register(SRCS ${CONTROLLER_SOURCES}
  INCLUDE_DIRS "../inc" "../inc/sp140" "../inc/sp140/lvgl"
  REQUIRES controller_platform dronecan ant_bms lvgl bmp3 driver esp_wifi esp_event
    nvs_flash app_update esp_system esp_timer esp_hw_support esp_adc espcoredump json)
execute_process(COMMAND git rev-parse --short HEAD WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE EPPG_GIT_REV OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND git rev-list --count HEAD WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE EPPG_VERSION_BUILD OUTPUT_STRIP_TRAILING_WHITESPACE)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
  GIT_REV="${EPPG_GIT_REV}" VERSION_BUILD=${EPPG_VERSION_BUILD})
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u fw_version_marker")
if(EPPG_QEMU)
  target_sources(${COMPONENT_LIB} PRIVATE "${CMAKE_SOURCE_DIR}/simulator/qemu_backend.cpp"
    "${CMAKE_SOURCE_DIR}/simulator/devices/peripherals.cpp")
  target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_SOURCE_DIR}/simulator")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC EPPG_QEMU=1)
  foreach(function driver_install start stop get_status_info reconfigure_alerts
      read_alerts receive transmit initiate_recovery driver_uninstall)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=twai_${function}")
  endforeach()
endif()
