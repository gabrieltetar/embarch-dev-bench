# Shared application build (embarch-dev-bench/design.md §2) -- referenced,
# not copied, by every workspace under workspaces/*/app.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarch_dev_bench)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/serial_protocol.c
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
