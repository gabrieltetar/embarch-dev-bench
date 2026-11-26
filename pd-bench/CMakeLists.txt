# PD bench: a scriptable USB Power Delivery partner (embarch-host-study-proposal.md,
# phase 1). Not the dev-bench wire: a Zephyr shell on the board's console.

cmake_minimum_required(VERSION 3.20.0)

# The expansion board is a shield of this app (boards/shields/): the SNK1M1
# sink unless -DSHIELD=x_nucleo_drp1m1 asks for the dual-role DRP1M1
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
if(NOT DEFINED SHIELD)
  set(SHIELD x_nucleo_snk1m1)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pd_bench)

target_sources(app PRIVATE src/main.c src/pd_logic.c)
target_sources_ifdef(CONFIG_USBC_CSM_DRP app PRIVATE src/tcpp03.c)

# What `pd hello` names: the version, and the commit this build's sources are at
set(PD_BENCH_VERSION 1.0.0)
execute_process(
  COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} describe --always --dirty --abbrev=12
  OUTPUT_VARIABLE PD_BENCH_COMMIT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT PD_BENCH_COMMIT)
  set(PD_BENCH_COMMIT unknown)
endif()
target_compile_definitions(app PRIVATE
  PD_BENCH_VERSION="${PD_BENCH_VERSION}"
  PD_BENCH_COMMIT="${PD_BENCH_COMMIT}")
