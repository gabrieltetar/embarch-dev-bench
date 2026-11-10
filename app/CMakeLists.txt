# Shared application build (embarch-dev-bench/design.md §2) -- referenced,
# not copied, by every workspace under workspaces/*/app.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/serial_protocol.c
  # embarch-study-designer/design.md §3 decisions 58-62: the `.eap` protocol
  # interpreter. Built for every board, including native_sim -- it is pure C
  # with no BLE and no Zephyr in it, which is what lets the ztest suite drive
  # the semantics directly (there is no real-vs-stub split to make here).
  src/eap_interp.c
  # embarch-dev-bench/design.md §3 decision 38: the Zephyr log backend that
  # forwards CONFIG_LOG output to Core as `LogLine`s. Built for every board,
  # including native_sim -- unlike ble_bridge there is no real-vs-stub
  # split here, since the file itself compiles to no-ops under CONFIG_LOG=n.
  src/dev_bench_log.c
  # decision 44: the advertiser census's Manufacturer Specific Data parsing and
  # rendering. Built for every board including native_sim, same reasoning as
  # eap_interp.c above -- it needs Zephyr's printk/util helpers but nothing
  # from the BT host, so a ztest can pin its offset arithmetic against a
  # synthetic payload on a board where ble_bridge_real.c itself never builds.
  src/scan_seen_mfg.c
  # dev-bench/007: the same failed-name census's bounded, no-partial-fragment
  # name-list append -- built for every board including native_sim, same
  # reasoning as scan_seen_mfg.c above, so a ztest can pin its truncation
  # behaviour even though ble_bridge_real.c itself never builds here.
  src/scan_seen_names.c
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
#
# Real bug found and fixed validating embarch-umbrella's stale-firmware
# doctor check (decision 25) against two real commits: `execute_process`
# only runs at CMake *configure* time, and CMake only reconfigures when one
# of its own tracked dependencies changes -- by default just CMakeLists.txt
# itself, not "the git commit checked out right now." An incremental
# `west build` after a plain `git commit` (no CMakeLists.txt edit) was
# silently re-linking with the *previous* configure's now-stale
# APP_FIRMWARE_VERSION baked in, for every translation unit, not just ones
# that happened to recompile -- exactly the false-negative decision 25's
# check exists to catch, self-inflicted by the build system itself. Fixed by
# making the checked-out commit itself a tracked configure dependency, so a
# commit (or branch switch) that changes it forces a reconfigure and a fresh
# `git describe`. Doesn't cover the plain-`--dirty` case (editing a tracked
# file without staging or committing touches neither of these) -- inherent
# to any git-describe-based scheme, not something CONFIGURE_DEPENDS can fix.
execute_process(
  COMMAND git rev-parse --git-dir
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE _embarch_git_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(_embarch_git_dir)
  if(NOT IS_ABSOLUTE "${_embarch_git_dir}")
    set(_embarch_git_dir "${CMAKE_CURRENT_SOURCE_DIR}/${_embarch_git_dir}")
  endif()
  if(EXISTS "${_embarch_git_dir}/HEAD")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_embarch_git_dir}/HEAD")
    file(READ "${_embarch_git_dir}/HEAD" _embarch_head_contents)
    if(_embarch_head_contents MATCHES "^ref: (.*)$")
      string(STRIP "${CMAKE_MATCH_1}" _embarch_head_ref)
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_embarch_git_dir}/${_embarch_head_ref}")
    endif()
  endif()
  if(EXISTS "${_embarch_git_dir}/index")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_embarch_git_dir}/index")
  endif()
endif()

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

# Where embarch-study-designer lives. No target links the crate; every target
# reads its wire schema version and its limits and wire enums out of the
# crate's source at configure time, because hand-mirroring them is how the
# schema number went stale twice (see below).
if(DEFINED ENV{EMBARCH_STUDY_DESIGNER_PATH})
  set(ESSD_DIR "$ENV{EMBARCH_STUDY_DESIGNER_PATH}")
else()
  # CMAKE_CURRENT_SOURCE_DIR here is workspaces/<name>/app, itself a
  # symlink to the real app/ (§2's tree). Resolve *that* symlink first, in
  # its own get_filename_component call -- CMake's REALPATH collapses ".."
  # textually before resolving symlinks, so appending "../../embarch-study-
  # designer" and resolving in one call never actually follows the
  # symlink: the ".." components cancel the still-unresolved "app" segment
  # lexically, landing in workspaces/embarch-study-designer (which doesn't
  # exist) instead of climbing out of the real app/'s real parent. Found
  # standing up workspaces/espressif (embarch-dev-bench/design.md's
  # ESP32-C5 interim decision) -- a real CMake-path-resolution bug, latent
  # in every other workspace too, just unnoticed because plain ABSOLUTE
  # (the original code here) happened to still be wrong in the same way
  # nobody hit until a workspace where the surrounding symlink chain
  # differed enough to surface it.
  get_filename_component(ESSD_APP_REALDIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
  get_filename_component(ESSD_DIR "${ESSD_APP_REALDIR}/../../embarch-study-designer" ABSOLUTE)
endif()
if(NOT EXISTS "${ESSD_DIR}/Cargo.toml")
  message(FATAL_ERROR "embarch-dev-bench: embarch-study-designer not found at ${ESSD_DIR} -- set EMBARCH_STUDY_DESIGNER_PATH if it isn't a sibling of this repo.")
endif()

# Every crate limit and `.eap` wire enum the headers size or decode by,
# derived from the crate rather than typed (decision 56).
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/crate_limits.cmake)

# The wire schema version `HelloAck` reports (decision 20), read out of the
# crate rather than mirrored by hand in C, for every target.
#
# It was a hand-written `#define`, and it went stale twice: found four bumps
# behind at v4 while implementing v9, fixed to 9, and found stale again at 9
# one bump later when decision 47 took the wire to 10. The comment written the
# first time already said nothing compares this number against the crate --
# which was true, and stayed true, and let the same defect recur immediately.
# Reading it from the crate's own source is what actually closes that, and the
# CONFIGURE_DEPENDS below is what makes an edit to that file re-run this
# instead of leaving a stale value baked in -- the same trap APP_FIRMWARE_VERSION
# hit above. Real boards used to get the number by cross-compiling the crate
# into a staticlib and calling into it (decision 8); that was the only live call
# into the staticlib, so the staticlib is gone and every board reads it here.
set(ESSD_SCHEMA_SRC "${ESSD_DIR}/src/schema_version.rs")
if(NOT EXISTS "${ESSD_SCHEMA_SRC}")
  message(FATAL_ERROR "embarch-dev-bench: ${ESSD_SCHEMA_SRC} not found -- the build reads DEV_BENCH_WIRE_SCHEMA_VERSION from it.")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ESSD_SCHEMA_SRC}")
file(READ "${ESSD_SCHEMA_SRC}" _essd_schema_src)
if(NOT _essd_schema_src MATCHES "pub const DEV_BENCH_WIRE_SCHEMA_VERSION: u32 = ([0-9]+)")
  message(FATAL_ERROR "embarch-dev-bench: could not read DEV_BENCH_WIRE_SCHEMA_VERSION out of ${ESSD_SCHEMA_SRC}.")
endif()
target_compile_definitions(app PRIVATE DEV_BENCH_WIRE_SCHEMA_VERSION=${CMAKE_MATCH_1}u)
message(STATUS "embarch-dev-bench: dev-bench wire schema version ${CMAKE_MATCH_1} (from the crate)")
