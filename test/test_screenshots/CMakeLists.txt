cmake_minimum_required(VERSION 3.14)
project(eppg_screenshot_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Paths
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(STUBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../screenshot_stubs)
set(LVGL_DIR "" CACHE PATH "Path to LVGL source (e.g., /tmp/lvgl)")
set(GTEST_DIR "" CACHE PATH "Path to GoogleTest source (e.g., /tmp/googletest)")

if(NOT LVGL_DIR OR NOT EXISTS ${LVGL_DIR}/CMakeLists.txt)
  # Try FetchContent as fallback
  include(FetchContent)
  FetchContent_Declare(
    lvgl
    GIT_REPOSITORY https://github.com/lvgl/lvgl.git
    GIT_TAG v8.4.0
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(lvgl)
  if(NOT lvgl_POPULATED)
    FetchContent_Populate(lvgl)
    set(LVGL_DIR ${lvgl_SOURCE_DIR})
  endif()
endif()

if(NOT GTEST_DIR OR NOT EXISTS ${GTEST_DIR}/CMakeLists.txt)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(googletest)
else()
  add_subdirectory(${GTEST_DIR} ${CMAKE_BINARY_DIR}/googletest)
endif()

# LVGL configuration: our lv_conf.h must be visible
# We compile LVGL sources directly rather than using its CMake to control flags precisely
file(GLOB_RECURSE LVGL_SOURCES
  ${LVGL_DIR}/src/*.c
)

add_library(lvgl STATIC ${LVGL_SOURCES})
target_include_directories(lvgl PUBLIC
  ${LVGL_DIR}
  ${PROJECT_ROOT}/inc/sp140/lvgl  # For lv_conf.h
)
target_compile_definitions(lvgl PUBLIC
  LV_CONF_INCLUDE_SIMPLE
  LV_LVGL_H_INCLUDE_SIMPLE
)
# Suppress warnings in LVGL source
target_compile_options(lvgl PRIVATE -w)

# Project UI source files (the actual code under test)
set(UI_SOURCES
  ${PROJECT_ROOT}/src/sp140/lvgl/lvgl_main_screen.cpp
  ${PROJECT_ROOT}/src/sp140/lvgl/lvgl_updates.cpp
  ${PROJECT_ROOT}/src/sp140/lvgl/lvgl_alerts.cpp
)

# Emulator sources
set(EMULATOR_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/emulator_display.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/emulator_stubs.cpp
)

# Test sources
set(TEST_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/test_screenshots.cpp
)

add_executable(screenshot_tests
  ${UI_SOURCES}
  ${EMULATOR_SOURCES}
  ${TEST_SOURCES}
)

target_include_directories(screenshot_tests PRIVATE
  ${STUBS_DIR}              # Stubs FIRST (Arduino.h, SPI.h, etc.)
  ${PROJECT_ROOT}/inc       # Project headers (sp140/structs.h, etc.)
  ${PROJECT_ROOT}/inc/sp140/lvgl  # lv_conf.h
  ${LVGL_DIR}               # LVGL headers
  ${CMAKE_CURRENT_SOURCE_DIR}  # emulator_display.h
)

target_compile_definitions(screenshot_tests PRIVATE
  LV_CONF_INCLUDE_SIMPLE
  LV_LVGL_H_INCLUDE_SIMPLE
  ARDUINO=1
  NATIVE_SCREENSHOT_TEST=1
)

# Force-include Arduino.h so project headers that rely on Arduino typedefs
# (e.g. `word` in structs.h) compile without modifying the core code.
target_compile_options(screenshot_tests PRIVATE
  -include ${STUBS_DIR}/Arduino.h
)

target_link_libraries(screenshot_tests PRIVATE
  lvgl
  gtest
  gtest_main
)

# Suppress warnings in UI sources (they may use Arduino idioms)
set_source_files_properties(${UI_SOURCES} PROPERTIES COMPILE_FLAGS "-w")
