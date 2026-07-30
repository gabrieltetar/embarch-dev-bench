# Shared application build (embarch-dev-bench/design.md §2) -- referenced,
# not copied, by every workspace under workspaces/*/app.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/serial_protocol.c
  src/study_ffi.c
)

# embarch-dev-bench/design.md §3 decision 16: ble_bridge_real.c (real Zephyr BT
# host calls) vs ble_bridge_stub.c (canned Outcomes, no BLE host) are separate
# files, not #ifdef branches inside one -- CONFIG_ARCH_POSIX is true exactly
# for native_sim among this project's targets, so it's what picks between them.
if(CONFIG_ARCH_POSIX)
  target_sources(app PRIVATE src/ble_bridge_stub.c)
else()
  target_sources(app PRIVATE src/ble_bridge_real.c)
endif()

# embarch-dev-bench/design.md §3 decision 18: HelloAck.firmware_version.
# Best-effort `git describe`; falls back to a fixed placeholder outside a git
# checkout (e.g. a source tarball) or without git installed.
execute_process(
  COMMAND git describe --always --dirty --abbrev=8
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE APP_FIRMWARE_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT APP_FIRMWARE_VERSION)
  set(APP_FIRMWARE_VERSION "unknown")
endif()
target_compile_definitions(app PRIVATE APP_FIRMWARE_VERSION="${APP_FIRMWARE_VERSION}")
