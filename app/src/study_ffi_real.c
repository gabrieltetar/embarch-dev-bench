/* Real implementation, per embarch-dev-bench/design.md §3 decision 8: calls
 * into embarch-study-designer's cross-compiled `extern "C"` staticlib
 * (embarch-study-designer/src/ffi.rs) instead of decision 20's fixed-result
 * stub (study_ffi_stub.c, kept for native_sim only). No interface change from
 * that stub — study_ffi.h's signatures were already dictated by the Rust
 * crate's existing FFI surface, exactly as decision 20 anticipated.
 *
 * Linked against libembarch_study_designer.a (app/CMakeLists.txt's decision-8
 * cross-compile step) — CONFIG_ARCH_POSIX builds never compile this file at
 * all (see app/CMakeLists.txt), so `essd_*` are only ever unresolved symbols
 * for a build that also links that staticlib.
 */
#include "study_ffi.h"

/* embarch-study-designer's src/ffi.rs. Not declared in a shared header on
 * that crate's side (it's a Rust `#[no_mangle] extern "C"` surface, not a C
 * library with its own installed headers) — these declarations are this
 * firmware's own copy of that contract, same posture study_ffi.h's own doc
 * comment already takes for the stub side.
 */
extern uint32_t essd_schema_version(void);

/* EssdStatus (embarch-study-designer/src/ffi.rs): Ok = 0, NullPointer = -1,
 * DecodeError = -2 — `int` return matches `#[repr(i32)]` exactly. */
extern int essd_study_decode_and_verify(const uint8_t *input, size_t input_len, bool *out_crc_matches);

/* essd_study_decode_full's EssdStudy/EssdStep/EssdBleAdvertiseAction
 * (embarch-study-designer/src/ffi.rs) are #[repr(C)] with the exact same
 * field layout as struct study_ffi_study/study_ffi_step/study_ffi_ble_advertise_action
 * (study_ffi.h) — this cast is safe because both sides are kept in sync by
 * hand (same posture as this file's other declarations). EssdStatus (Ok = 0,
 * NullPointer = -1, DecodeError = -2, CrcMismatch = -3, UnsupportedAction =
 * -4) matches `int` / enum study_ffi_decode_status exactly. */
extern int essd_study_decode_full(const uint8_t *input, size_t input_len,
				   struct study_ffi_study *out_study);

uint32_t study_ffi_schema_version(void)
{
	return essd_schema_version();
}

int study_ffi_decode_and_verify(const uint8_t *input, size_t input_len, bool *out_crc_matches)
{
	return essd_study_decode_and_verify(input, input_len, out_crc_matches);
}

int study_ffi_decode_study(const uint8_t *input, size_t input_len, struct study_ffi_study *out_study)
{
	return essd_study_decode_full(input, input_len, out_study);
}
