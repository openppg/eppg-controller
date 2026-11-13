cmake_minimum_required(VERSION 3.16)
project(eppg_host_screenshot_offline LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(DEFAULT_PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(PROJECT_ROOT "${DEFAULT_PROJECT_ROOT}" CACHE PATH "Controller source tree to render")
set(HOST_TEST_ROOT "${DEFAULT_PROJECT_ROOT}" CACHE PATH "Source of host-only adapters")

set(LVGL_DIR "$ENV{LVGL_DIR}" CACHE PATH "Path to pinned LVGL 9.6.0 source")
set(GTEST_DIR "$ENV{GTEST_DIR}" CACHE PATH "Path to GoogleTest source")
if(NOT LVGL_DIR)
  foreach(candidate "${PROJECT_ROOT}/components/lvgl" "${PROJECT_ROOT}/.pio/libdeps/native-screenshot/lvgl")
    if(EXISTS "${candidate}/lvgl.h")
      set(LVGL_DIR "${candidate}")
      break()
    endif()
  endforeach()
endif()
if(NOT GTEST_DIR AND EXISTS "${PROJECT_ROOT}/.pio/libdeps/native-screenshot/googletest/googletest/src/gtest-all.cc")
  set(GTEST_DIR "${PROJECT_ROOT}/.pio/libdeps/native-screenshot/googletest")
endif()
if(NOT EXISTS "${LVGL_DIR}/lvgl.h")
  message(FATAL_ERROR "Set LVGL_DIR to the pinned LVGL 9.6.0 source or vendor components/lvgl")
endif()
if(NOT EXISTS "${GTEST_DIR}/googletest/src/gtest-all.cc")
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE)
  FetchContent_GetProperties(googletest)
  if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
  endif()
  set(GTEST_DIR "${googletest_SOURCE_DIR}")
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
  "${HOST_TEST_ROOT}/test/test_screenshots/emulator_display.cpp"
  "${HOST_TEST_ROOT}/test/test_screenshots/emulator_stubs.cpp"
  "${HOST_TEST_ROOT}/test/test_screenshots/test_screenshots.cpp"
)

target_include_directories(screenshot_tests PRIVATE
  "${HOST_TEST_ROOT}/test/screenshot_stubs"
  "${PROJECT_ROOT}/inc"
  "${PROJECT_ROOT}/inc/sp140/lvgl"
  "${LVGL_DIR}"
  "${HOST_TEST_ROOT}/test/test_screenshots"
)
target_compile_definitions(screenshot_tests PRIVATE
  LV_CONF_INCLUDE_SIMPLE
  LV_LVGL_H_INCLUDE_SIMPLE
  ARDUINO=1
  NATIVE_SCREENSHOT_TEST=1
)
set_source_files_properties(${UI_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
target_link_libraries(screenshot_tests PRIVATE lvgl gtest_main)

add_executable(framebuffer_renderer
  ${UI_SOURCES}
  "${HOST_TEST_ROOT}/test/test_screenshots/emulator_display.cpp"
  "${HOST_TEST_ROOT}/test/test_screenshots/emulator_stubs.cpp"
  "${HOST_TEST_ROOT}/test/test_screenshots/framebuffer_renderer.cpp")
target_include_directories(framebuffer_renderer PRIVATE
  "${HOST_TEST_ROOT}/test/screenshot_stubs" "${PROJECT_ROOT}/inc"
  "${PROJECT_ROOT}/inc/sp140/lvgl" "${LVGL_DIR}"
  "${HOST_TEST_ROOT}/test/test_screenshots")
target_compile_definitions(framebuffer_renderer PRIVATE
  LV_CONF_INCLUDE_SIMPLE LV_LVGL_H_INCLUDE_SIMPLE NATIVE_SCREENSHOT_TEST=1)
target_link_libraries(framebuffer_renderer PRIVATE lvgl)
if(EXISTS "${PROJECT_ROOT}/inc/platform/runtime.h")
  target_compile_definitions(screenshot_tests PRIVATE EPPG_NATIVE_IDF=1)
  target_compile_definitions(framebuffer_renderer PRIVATE EPPG_NATIVE_IDF=1)
endif()
enable_testing()
add_test(NAME screenshot_regressions COMMAND screenshot_tests)
set_tests_properties(screenshot_regressions PROPERTIES WORKING_DIRECTORY "${HOST_TEST_ROOT}")
