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
  # including native_sim -- unlike ble_bridge/study_ffi there is no real-vs-stub
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
# study_ffi.c follows the same split, same reasoning, as of decision 8: native_sim
# stays on the fixed-result stub permanently (a host-only sanity board doesn't need
# the real Rust decode/CRC logic to be useful), while a real board links study_ffi_real.c
# against the cross-compiled embarch-study-designer staticlib wired in below.
if(CONFIG_ARCH_POSIX)
  target_sources(app PRIVATE src/ble_bridge_stub.c src/study_ffi_stub.c)
else()
  target_sources(app PRIVATE src/ble_bridge_real.c src/study_ffi_real.c)
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

# Where embarch-study-designer lives. Resolved for *every* target, not just the
# ones that cross-compile it: the native_sim build links no crate but still has
# to report the crate's wire schema version, and hand-mirroring that number is
# how it went stale twice (see below).
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

# The stub's `study_ffi_schema_version()` answer, read out of the crate rather
# than mirrored by hand in C.
#
# It was a hand-written `#define`, and it went stale twice: found four bumps
# behind at v4 while implementing v9, fixed to 9, and found stale again at 9
# one bump later when decision 47 took the wire to 10. The comment written the
# first time already said nothing compares this number against the crate --
# which was true, and stayed true, and let the same defect recur immediately.
# Reading it from the crate's own source is what actually closes that, and the
# CONFIGURE_DEPENDS below is what makes an edit to that file re-run this
# instead of leaving a stale value baked in -- the same trap APP_FIRMWARE_VERSION
# hit above.
if(CONFIG_ARCH_POSIX)
  set(ESSD_SCHEMA_SRC "${ESSD_DIR}/src/schema_version.rs")
  if(NOT EXISTS "${ESSD_SCHEMA_SRC}")
    message(FATAL_ERROR "embarch-dev-bench: ${ESSD_SCHEMA_SRC} not found -- the native_sim build reads DEV_BENCH_WIRE_SCHEMA_VERSION from it.")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ESSD_SCHEMA_SRC}")
  file(READ "${ESSD_SCHEMA_SRC}" _essd_schema_src)
  if(NOT _essd_schema_src MATCHES "pub const DEV_BENCH_WIRE_SCHEMA_VERSION: u32 = ([0-9]+)")
    message(FATAL_ERROR "embarch-dev-bench: could not read DEV_BENCH_WIRE_SCHEMA_VERSION out of ${ESSD_SCHEMA_SRC}.")
  endif()
  target_compile_definitions(app PRIVATE STUDY_FFI_STUB_SCHEMA_VERSION=${CMAKE_MATCH_1})
  message(STATUS "embarch-dev-bench: dev-bench wire schema version ${CMAKE_MATCH_1} (from the crate)")
endif()

# embarch-dev-bench/design.md §3 decision 8: cross-compile
# embarch-study-designer's Rust FFI staticlib as part of `west build` itself,
# so there's exactly one build command a developer runs. Implemented as a
# plain `cargo rustc` invocation against the crate's path, not real
# west-manifest module plumbing (that workspace's manifest doesn't list it as
# a project) -- the crate lives as an ordinary sibling repo on disk (this
# suite's standard layout, DOC-PROTOCOL.md §2) and is edited locally far more
# often than it's pulled from a pinned upstream revision, so a path-based
# invocation is simpler for the same practical outcome decision 8 actually
# asked for. Skipped entirely for native_sim (CONFIG_ARCH_POSIX): that
# workspace keeps study_ffi.c on its decision-20 stub permanently, mirroring
# ble_bridge_stub.c's own reasoning above -- a host-only sanity board doesn't
# need the real decode/CRC logic to be useful.
if(NOT CONFIG_ARCH_POSIX)
  # Board -> Rust target triple. nRF54L15's cpuapp core is Cortex-M33
  # (Armv8-M Mainline) -> thumbv8m.main-none-eabi. Deliberately the *soft*-float
  # variant, not -eabihf: this app's Zephyr build has no CONFIG_FPU / hard-float
  # ABI enabled, and linking a hard-float-ABI staticlib (VFP-register calling
  # convention for any function touching an f32, e.g. Sample's own field --
  # not just this crate's actual FFI-exposed functions, which pass none)
  # against a soft-float app fails at link time ("uses VFP register
  # arguments") -- confirmed by hitting exactly that error against the real
  # board before settling on this. A second board/core later extends this
  # if/else, not replaces it (§4 open item).
  if(CONFIG_SOC_NRF54L15_CPUAPP)
    set(ESSD_RUST_TARGET "thumbv8m.main-none-eabi")
  elseif(CONFIG_SOC_ESP32C5_HPCORE)
    # ESP32-C5 (interim substitute dev-bench board, design.md's ESP32-C5
    # interim decision): single RISC-V HP core, RV32IMAC -- no hardware FPU,
    # no compressed-only variant assumed -- so the target is the plain
    # riscv32imac triple, not a narrower/wider one. Unlike the nRF54L15's
    # soft/hard-float split, there's only one RISC-V ABI variant Zephyr
    # builds this core with, so no analogous footgun was found here.
    set(ESSD_RUST_TARGET "riscv32imac-unknown-none-elf")
  else()
    message(FATAL_ERROR "embarch-dev-bench: no Rust target triple mapped for this board yet (decision 8) -- add one to app/CMakeLists.txt before building for it.")
  endif()

  set(ESSD_LIB "${ESSD_DIR}/target/${ESSD_RUST_TARGET}/release/libembarch_study_designer.a")

  # No CMake-tracked dependency list for this custom target -- `cargo` already
  # does its own incremental-rebuild tracking against the crate's real
  # sources, so this just re-invokes it on every build and lets cargo decide
  # whether there's anything to actually recompile.
  add_custom_target(embarch_study_designer_ffi ALL
    COMMAND cargo rustc --release --target ${ESSD_RUST_TARGET} --features ffi --lib --crate-type staticlib
    WORKING_DIRECTORY "${ESSD_DIR}"
    BYPRODUCTS "${ESSD_LIB}"
    COMMENT "Cross-compiling embarch-study-designer (${ESSD_RUST_TARGET}, --features ffi)"
    VERBATIM
  )
  add_dependencies(app embarch_study_designer_ffi)
  target_link_libraries(app PRIVATE "${ESSD_LIB}")
endif()
