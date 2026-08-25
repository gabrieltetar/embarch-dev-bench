/* Interface to embarch-study-designer's Study decode/CRC-verify logic.
 *
 * embarch-dev-bench/design.md §3 decisions 8/19/20. This header's function
 * signatures mirror embarch-study-designer's `extern "C"` FFI surface
 * (embarch-study-designer/src/ffi.rs: `essd_schema_version`,
 * `essd_study_decode_and_verify`) so that study_ffi_stub.c (native_sim,
 * decision 20's fixed results) and study_ffi_real.c (a real board, decision
 * 8's cross-compiled staticlib) are interchangeable behind one interface —
 * app/CMakeLists.txt picks which one actually compiles, same split as
 * ble_bridge_real.c/ble_bridge_stub.c.
 */
#ifndef EMBARCH_DEV_BENCH_STUDY_FFI_H_
#define EMBARCH_DEV_BENCH_STUDY_FFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* embarch-study-designer's DEV_BENCH_WIRE_SCHEMA_VERSION, for this firmware's
 * HelloAck (embarch-study-designer/design.md §3 decision 12). */
uint32_t study_ffi_schema_version(void);

/* Decodes a postcard-encoded Study from `input[0..input_len]` and reports
 * whether its steps_crc matches (embarch-study-designer/design.md §3 decision
 * 17) via `*out_crc_matches`. Returns 0 on success, a negative status
 * otherwise (null argument, or malformed input) — never crashes on malformed
 * input, matching essd_study_decode_and_verify's own panic-safety contract
 * (embarch-study-designer/design.md §3 decision 23). */
int study_ffi_decode_and_verify(const uint8_t *input, size_t input_len, bool *out_crc_matches);

/* Mirrors ffi.rs's EssdBleAdvertiseAction/EssdStep/EssdStudy/essd_study_decode_full
 * -- see that function's doc comment for the exact contract (BleAdvertise-only,
 * CRC-mismatch and unsupported-action both fail whole, no partial write).
 *
 * NOTE: this decodes a postcard-encoded `Study` (embarch-study-designer's
 * `study::Study`: name + steps + validations + steps_crc), NOT the leaner
 * `DevBenchMessage::StudyStart` (steps + steps_crc only) this firmware
 * actually receives over the wire (serial_protocol.h) -- those are two
 * different postcard shapes, not interchangeable byte-for-byte. main.c's
 * real dispatch loop (embarch-dev-bench/design.md §3 decision 21) therefore
 * does NOT call this function: serial_protocol.c already decodes
 * `StudyStart` directly into `struct dbm_study_start` (BleAdvertise-only,
 * with the same CRC-mismatch/unsupported-action semantics applied natively
 * in C alongside that decode) without needing a second FFI round-trip. This
 * decode surface is kept as validated, additive infrastructure matching
 * `essd_study_decode_and_verify`'s existing `Study`-shaped precedent, for a
 * future caller that does have a real `Study` blob to decode (see main.c's
 * own comment for the full reasoning). */
#define STUDY_FFI_MAX_LOCAL_NAME_LEN 26  /* mirrors limits::MAX_LOCAL_NAME_LEN */
#define STUDY_FFI_MAX_NAME_LEN 32        /* mirrors limits::MAX_NAME_LEN */
#define STUDY_FFI_MAX_STUDY_NAME_LEN 64  /* mirrors limits::MAX_STUDY_NAME_LEN */
#define STUDY_FFI_MAX_STEPS_PER_STUDY 64 /* mirrors limits::MAX_STEPS_PER_STUDY */

struct study_ffi_ble_advertise_action {
	uint8_t local_name[STUDY_FFI_MAX_LOCAL_NAME_LEN];
	uint8_t local_name_len;
	bool has_local_name;
	uint16_t adv_interval_ms;
};

struct study_ffi_step {
	uint8_t name[STUDY_FFI_MAX_NAME_LEN];
	uint8_t name_len;
	uint32_t timeout_ms;
	bool continue_on_fail;
	struct study_ffi_ble_advertise_action action;
};

struct study_ffi_study {
	uint8_t name[STUDY_FFI_MAX_STUDY_NAME_LEN];
	uint8_t name_len;
	struct study_ffi_step steps[STUDY_FFI_MAX_STEPS_PER_STUDY];
	uint32_t steps_len;
};

/* EssdStatus (embarch-study-designer/src/ffi.rs), append-only: Ok = 0,
 * NullPointer = -1, DecodeError = -2, CrcMismatch = -3, UnsupportedAction = -4. */
enum study_ffi_decode_status {
	STUDY_FFI_OK = 0,
	STUDY_FFI_NULL_POINTER = -1,
	STUDY_FFI_DECODE_ERROR = -2,
	STUDY_FFI_CRC_MISMATCH = -3,
	STUDY_FFI_UNSUPPORTED_ACTION = -4,
};

int study_ffi_decode_study(const uint8_t *input, size_t input_len, struct study_ffi_study *out_study);

#endif /* EMBARCH_DEV_BENCH_STUDY_FFI_H_ */
