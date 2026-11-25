cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench_study_stop_test)

# crate_limits.h, which serial_protocol.h includes (decision 56).
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/crate_limits.cmake)

# study_stop.c reads a byte source and decodes with serial_protocol.c; the
# analog stream is here so a test can stop a capture the way main.c does. The
# app's Kconfig is not loaded, so the two sizes it would define are given.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_compile_definitions(app PRIVATE
  CONFIG_EMBARCH_ANALOG_MAX_CHANNELS=2
  CONFIG_EMBARCH_ANALOG_STREAM_SLOTS=4
)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/study_stop.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/serial_protocol.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/eap_interp.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/analog_stream.c
  src/main.c
)
