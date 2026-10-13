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

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_USBC_CSM_DRP app PRIVATE src/tcpp03.c)
