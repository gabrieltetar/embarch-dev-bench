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

/* embarch-study-designer's DEV_BENCH_WIRE_SCHEMA_VERSION -- the **wire**
 * constant specifically, as of that crate's 2026-08-25 split of one schema
 * version into two (design.md §3 decision 12's amendment). This number is what
 * a `HelloAck` reports, and a host-side-only reshape must not move what
 * firmware claims about itself; the host constant has no business being
 * mirrored here at all, since dev-bench is not a party to that hop.
 *
 * Supplied by app/CMakeLists.txt, which reads it out of the crate's own
 * source. It used to be a hand-written number here and it went stale twice --
 * found four bumps behind at v4 while implementing v9, then stale again one
 * bump later. Nothing could compare it against the crate, because by
 * construction native_sim links no crate to compare it to, and the comment
 * saying so did not stop the second recurrence. Reading it at configure time
 * is what does.
 *
 * This does NOT close the wider gap that comment named: only a build linking
 * the real staticlib exercises `essd_schema_version` (study_ffi_real.c) for
 * real. What it closes is the number being wrong, which was the part that
 * actually kept happening.
 */
#ifndef STUDY_FFI_STUB_SCHEMA_VERSION
#error "STUDY_FFI_STUB_SCHEMA_VERSION must come from app/CMakeLists.txt"
#endif

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
 * decodes a `Study` (name + steps + steps_crc), a different
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
