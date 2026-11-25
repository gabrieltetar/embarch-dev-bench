cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench_analog_stream_test)

# analog_stream.c has no hardware in it: the test drives it with a fake
# backend and a fake sink. The app's Kconfig is not loaded here, so the two
# sizes it would define are given directly: two channels, to exercise the
# N-channel layout, and four slots, so a test can exhaust them.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_compile_definitions(app PRIVATE
  CONFIG_EMBARCH_ANALOG_MAX_CHANNELS=2
  CONFIG_EMBARCH_ANALOG_STREAM_SLOTS=4
)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/analog_stream.c
  src/main.c
)
