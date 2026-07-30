/* Stub implementation, per embarch-dev-bench/design.md §3 decision 20: bring-up
 * of main.c/serial_protocol.c/ble_bridge_real.c doesn't need to wait on decision
 * 8's real cross-compiled embarch-study-designer staticlib. Returns fixed
 * results rather than actually decoding/verifying anything. Swap this file's
 * body for real calls into the linked staticlib once decision 8 lands — study_ffi.h's
 * signatures are dictated by that staticlib's existing FFI surface already, so no
 * interface change is expected here.
 */
#include "study_ffi.h"

/* Mirrors embarch-study-designer's STUDY_DESIGNER_SCHEMA_VERSION at the time this
 * stub was written (src/schema_version.rs). Kept in sync by hand until decision 8
 * lets this firmware pull the real constant from the linked staticlib instead. */
#define STUDY_FFI_STUB_SCHEMA_VERSION 2

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
