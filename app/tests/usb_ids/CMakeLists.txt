cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench_usb_ids_test)

# usb_ids.c is pure arithmetic with no Zephyr dependency, so it links into a
# native_sim test on its own.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/usb_ids.c
  src/main.c
)
