/* Stub implementation, per embarch-dev-bench/design.md §3 decision 20: bring-up
 * of main.c/serial_protocol.c/ble_bridge_real.c doesn't need to wait on decision
 * 8's real cross-compiled embarch-study-designer staticlib. Returns fixed
 * results rather than actually decoding/verifying anything.
 *
 * Decision 8 has since landed (see study_ffi_real.c, the sibling this file no
 * longer feeds into) -- this file is kept on permanently for native_sim only
 * (app/CMakeLists.txt selects between the two on CONFIG_ARCH_POSIX, same split
 * as ble_bridge_real.c/ble_bridge_stub.c), since a host-only sanity board
 * doesn't need the real Rust decode/CRC logic to be useful.
 */
#include "study_ffi.h"

#include <string.h>

/* Mirrors embarch-study-designer's STUDY_DESIGNER_SCHEMA_VERSION by hand, since
 * native_sim never links the real staticlib (this file's own header comment) --
 * bump alongside that crate's own constant (currently 3, src/schema_version.rs)
 * whenever it changes. */
#define STUDY_FFI_STUB_SCHEMA_VERSION 3

uint32_t study_ffi_schema_version(void)
{
	return STUDY_FFI_STUB_SCHEMA_VERSION;
}

int study_ffi_decode_and_verify(const uint8_t *input, size_t input_len, bool *out_crc_matches)
{
	if (input == NULL || out_crc_matches == NULL) {
		return -1;
	}
	(void)input_len;
	*out_crc_matches = true;
	return 0;
}

/* Fixed-result stub, same posture as study_ffi_decode_and_verify above: this
 * decodes a `Study` (name + steps + validations + steps_crc), a different
 * postcard shape than the `DevBenchMessage::StudyStart` (steps + steps_crc
 * only) this firmware actually receives over the wire -- see study_ffi.h's
 * doc comment on struct study_ffi_study. Nothing in this firmware's real
 * dispatch path calls this on either build (main.c dispatches directly off
 * serial_protocol.c's own `struct dbm_study_start`, which already applies
 * the same CRC-mismatch/unsupported-action semantics natively in C), so
 * native_sim doesn't need a real postcard decoder here to stay useful --
 * this returns a fixed empty-but-valid decode rather than actually parsing
 * `input`, matching this file's existing fixed-result posture. */
int study_ffi_decode_study(const uint8_t *input, size_t input_len, struct study_ffi_study *out_study)
{
	if (input == NULL || out_study == NULL) {
		return STUDY_FFI_NULL_POINTER;
	}
	(void)input_len;
	memset(out_study, 0, sizeof(*out_study));
	return STUDY_FFI_OK;
}
