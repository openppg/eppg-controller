cmake_minimum_required(VERSION 3.16)
project(eppg_host_screenshot_offline LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(LVGL_DIR
    "${PROJECT_ROOT}/.pio/libdeps/native-screenshot/lvgl"
    CACHE PATH "Path to an LVGL 9.5.0 source tree")
set(GTEST_DIR
    "${PROJECT_ROOT}/.pio/libdeps/native-screenshot/googletest"
    CACHE PATH "Path to PlatformIO's GoogleTest source tree")

if(NOT EXISTS "${LVGL_DIR}/lvgl.h")
  message(FATAL_ERROR
    "LVGL cache not found at ${LVGL_DIR}. Run: pio pkg install -e native-screenshot")
endif()

if(NOT EXISTS "${GTEST_DIR}/googletest/src/gtest-all.cc")
  message(FATAL_ERROR
    "GoogleTest cache not found at ${GTEST_DIR}. Run: pio pkg install -e native-screenshot")
endif()

# PlatformIO's GoogleTest package intentionally omits its upstream CMake files,
# so compile the two canonical aggregation units directly.
add_library(gtest STATIC
  "${GTEST_DIR}/googletest/src/gtest-all.cc"
)
target_include_directories(gtest PUBLIC
  "${GTEST_DIR}/googletest"
  "${GTEST_DIR}/googletest/include"
)

add_library(gtest_main STATIC
  "${GTEST_DIR}/googletest/src/gtest_main.cc"
)
target_link_libraries(gtest_main PUBLIC gtest)

file(GLOB_RECURSE LVGL_SOURCES CONFIGURE_DEPENDS
  "${LVGL_DIR}/src/*.c"
)

add_library(lvgl STATIC ${LVGL_SOURCES})
target_include_directories(lvgl PUBLIC
  "${LVGL_DIR}"
  "${PROJECT_ROOT}/inc/sp140/lvgl"
)
target_compile_definitions(lvgl PUBLIC
  LV_CONF_INCLUDE_SIMPLE
  LV_LVGL_H_INCLUDE_SIMPLE
)
target_compile_options(lvgl PRIVATE -w)

set(UI_SOURCES
  "${PROJECT_ROOT}/src/sp140/lvgl/lvgl_splash.cpp"
  "${PROJECT_ROOT}/src/sp140/lvgl/lvgl_main_screen.cpp"
  "${PROJECT_ROOT}/src/sp140/lvgl/lvgl_updates.cpp"
  "${PROJECT_ROOT}/src/sp140/lvgl/lvgl_alerts.cpp"
)

add_executable(screenshot_tests
  ${UI_SOURCES}
  "${PROJECT_ROOT}/test/test_screenshots/emulator_display.cpp"
  "${PROJECT_ROOT}/test/test_screenshots/emulator_stubs.cpp"
  "${PROJECT_ROOT}/test/test_screenshots/test_screenshots.cpp"
)

target_include_directories(screenshot_tests PRIVATE
  "${PROJECT_ROOT}/test/screenshot_stubs"
  "${PROJECT_ROOT}/inc"
  "${PROJECT_ROOT}/inc/sp140/lvgl"
  "${LVGL_DIR}"
  "${PROJECT_ROOT}/test/test_screenshots"
)
target_compile_definitions(screenshot_tests PRIVATE
  LV_CONF_INCLUDE_SIMPLE
  LV_LVGL_H_INCLUDE_SIMPLE
  ARDUINO=1
  NATIVE_SCREENSHOT_TEST=1
)
set_source_files_properties(${UI_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
target_link_libraries(screenshot_tests PRIVATE lvgl gtest_main)
