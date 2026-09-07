cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench_scan_seen_mfg_test)

# scan_seen_mfg.c is linkable in isolation on purpose (see its own header
# comment): it needs Zephyr's printk/util helpers but nothing from the BT
# host, which is what lets this run under native_sim even though
# ble_bridge_real.c -- the file that actually calls it -- never builds there
# (app/CMakeLists.txt picks ble_bridge_stub.c instead, decision 16).
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/scan_seen_mfg.c
  src/main.c
)
