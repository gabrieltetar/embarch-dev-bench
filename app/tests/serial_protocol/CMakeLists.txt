cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench_serial_protocol_test)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/serial_protocol.c
  # embarch-study-designer/design.md §3 decisions 58-62: the `.eap`
  # interpreter. Pure C with no BLE and no Zephyr in it, which is what
  # lets this suite drive its semantics directly against the same worked
  # protocol that crate's own eap_worked_protocols.rs uses.
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/eap_interp.c
  src/main.c
)
