cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench_log_test)

# dev_bench_log.c is linkable in isolation on purpose (see dev_bench_log.h):
# it depends on the Zephyr log subsystem and on serial_protocol.h's
# DBM_MAX_LOG_LINE_LEN, never on main.c or the UART, so a test can install a
# capturing sink where main.c installs the real one.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/dev_bench_log.c
  src/main.c
  # Registers a log module under the app's own name, so the app-module floor
  # (design.md §3 decision 39) is tested against a real match.
  src/app_module.c
)
