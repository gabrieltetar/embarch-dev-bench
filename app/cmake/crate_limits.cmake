# crate_limits.h: embarch-study-designer's limits and wire enums (link-level and `.eap`),
# derived from the crate's own source at configure time (decision 56).
#
# Included after find_package(Zephyr) by the app and by every test under
# app/tests/, since all of them compile headers that size arrays by these
# values. It generalises what the app's CMakeLists.txt already did for one
# number, DEV_BENCH_WIRE_SCHEMA_VERSION: a hand-typed copy of a crate constant
# went stale twice there, and every other copy carried the same "mirrors"
# comment with nothing comparing it.
#
# Uses ESSD_DIR when the includer has set it; otherwise resolves it the same
# way, from this file's real location (a workspace reaches app/ through a
# symlink, and REALPATH must follow it before climbing out).

if(NOT DEFINED ESSD_DIR)
  if(DEFINED ENV{EMBARCH_STUDY_DESIGNER_PATH})
    set(ESSD_DIR "$ENV{EMBARCH_STUDY_DESIGNER_PATH}")
  else()
    get_filename_component(_crate_limits_real "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
    get_filename_component(ESSD_DIR "${_crate_limits_real}/../../../embarch-study-designer" ABSOLUTE)
  endif()
endif()

set(_crate_limits_script "${CMAKE_CURRENT_LIST_DIR}/gen_crate_limits.py")
set(_crate_limits_inputs
  "${ESSD_DIR}/src/limits.rs"
  "${ESSD_DIR}/src/decoder.rs"
  "${ESSD_DIR}/src/eap.rs"
  "${ESSD_DIR}/src/protocol.rs"
  "${ESSD_DIR}/src/study.rs"
  "${ESSD_DIR}/src/ids.rs"
  "${ESSD_DIR}/src/gatt.rs"
  "${ESSD_DIR}/src/sample.rs"
  "${ESSD_DIR}/src/streams.rs"
)
foreach(_crate_limits_input ${_crate_limits_inputs})
  if(NOT EXISTS "${_crate_limits_input}")
    message(FATAL_ERROR "embarch-dev-bench: ${_crate_limits_input} not found -- crate_limits.h is derived from it. Set EMBARCH_STUDY_DESIGNER_PATH if embarch-study-designer isn't a sibling of this repo.")
  endif()
endforeach()
# An edit to any input, or to the generator, re-runs configure instead of
# leaving an old value compiled in.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${_crate_limits_inputs} "${_crate_limits_script}")

set(CRATE_LIMITS_DIR "${CMAKE_BINARY_DIR}/crate_limits")
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} "${_crate_limits_script}"
          --crate "${ESSD_DIR}" --out "${CRATE_LIMITS_DIR}/crate_limits.h"
  RESULT_VARIABLE _crate_limits_rc
  ERROR_VARIABLE _crate_limits_err
)
if(NOT _crate_limits_rc EQUAL 0)
  message(FATAL_ERROR "embarch-dev-bench: could not derive crate_limits.h from ${ESSD_DIR}:\n${_crate_limits_err}")
endif()
target_include_directories(app PRIVATE "${CRATE_LIMITS_DIR}")
message(STATUS "embarch-dev-bench: crate_limits.h derived from ${ESSD_DIR}")
