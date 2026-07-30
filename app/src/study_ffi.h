/* Interface to embarch-study-designer's Study decode/CRC-verify logic.
 *
 * embarch-dev-bench/design.md §3 decisions 19/20. This header's function
 * signatures mirror embarch-study-designer's `extern "C"` FFI surface
 * (embarch-study-designer/src/ffi.rs: `essd_schema_version`,
 * `essd_study_decode_and_verify`) so that swapping study_ffi.c's current stub
 * implementation for real calls into the cross-compiled Rust staticlib, once
 * decision 8's west-module wiring exists, needs no change to this header or
 * to any caller of it — only to study_ffi.c's own body.
 */
#ifndef EMBARCH_DEV_BENCH_STUDY_FFI_H_
#define EMBARCH_DEV_BENCH_STUDY_FFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* embarch-study-designer's STUDY_DESIGNER_SCHEMA_VERSION, for this firmware's
 * HelloAck (embarch-study-designer/design.md §3 decision 12). */
uint32_t study_ffi_schema_version(void);

/* Decodes a postcard-encoded Study from `input[0..input_len]` and reports
 * whether its steps_crc matches (embarch-study-designer/design.md §3 decision
 * 17) via `*out_crc_matches`. Returns 0 on success, a negative status
 * otherwise (null argument, or malformed input) — never crashes on malformed
 * input, matching essd_study_decode_and_verify's own panic-safety contract
 * (embarch-study-designer/design.md §3 decision 23). */
int study_ffi_decode_and_verify(const uint8_t *input, size_t input_len, bool *out_crc_matches);

#endif /* EMBARCH_DEV_BENCH_STUDY_FFI_H_ */
