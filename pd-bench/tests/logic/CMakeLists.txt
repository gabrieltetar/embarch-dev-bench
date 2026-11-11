cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pd_bench_logic_test)

# pd_logic.c has no shell, kernel or device call, so it runs on native_sim
# without the USB-C stack, which has no TCPC emulator to run against.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pd_logic.c
  src/main.c
)
