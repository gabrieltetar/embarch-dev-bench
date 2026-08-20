/* DevBenchMessage wire protocol: COBS framing + postcard-compatible encoding.
 *
 * embarch-dev-bench/design.md §2, §3 decisions 7/10/12/20. Mirrors
 * embarch-study-designer's `protocol::DevBenchMessage` field-for-field and
 * byte-for-byte (postcard varint/fixint rules) so this hand-written C
 * implementation and the Rust crate's own round-trip tests both describe the
 * identical wire format. `embarch-study-designer`'s FFI surface doesn't cover
 * DevBenchMessage encode/decode (only `Study` decode+CRC, see study_ffi.h) —
 * that's this file's job.
 *
 * Pure encode/decode logic only, no I/O — keeps this unit-testable on any
 * host (see app/tests/serial_protocol) independent of a real UART.
 */
#ifndef EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_
#define EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Mirrors embarch-study-designer's limits::MAX_FIRMWARE_VERSION_LEN / MAX_LOG_LINE_LEN
 * / MAX_LOCAL_NAME_LEN / MAX_NAME_LEN / MAX_STEPS_PER_STUDY / MAX_FAIL_REASON_LEN /
 * MAX_PAYLOAD_LEN (embarch-study-designer/src/limits.rs) — kept in sync by hand until
 * decision 8's west-module wiring lets this firmware pull the constants directly from
 * that crate. */
#define DBM_MAX_FIRMWARE_VERSION_LEN 32
#define DBM_MAX_LOG_LINE_LEN 128
#define DBM_MAX_LOCAL_NAME_LEN 26
#define DBM_MAX_NAME_LEN 32
#define DBM_MAX_STEPS_PER_STUDY 64
#define DBM_MAX_FAIL_REASON_LEN 64
#define DBM_MAX_PAYLOAD_LEN 512

/* Largest single postcard-encoded (pre-COBS) DevBenchMessage this firmware sends/receives.
 * StudyStart with a full MAX_STEPS_PER_STUDY of BleAdvertise-only steps dominates every
 * other variant (LogLine's 128 bytes, StepResult's captured_data up to
 * DBM_MAX_PAYLOAD_LEN, included alongside for safety): per-step worst case is roughly
 * tag(1) + name len+bytes(1+DBM_MAX_NAME_LEN) + timeout_ms varint(5) +
 * continue_on_fail(1) + action tag(1) + has_local_name+local_name
 * len+bytes(1+1+DBM_MAX_LOCAL_NAME_LEN) + adv_interval_ms varint(3), generously
 * rounded up to 128 bytes/step; plus StudyStart's own steps_len/steps_crc varints and
 * a StepResult's captured_data headroom. */
#define DBM_MAX_STUDY_START_LEN (8 + (DBM_MAX_STEPS_PER_STUDY * 128) + 8)
#define DBM_MAX_STEP_RESULT_LEN (16 + DBM_MAX_NAME_LEN + DBM_MAX_FAIL_REASON_LEN + DBM_MAX_PAYLOAD_LEN)
#define DBM_MAX_RAW_LEN (DBM_MAX_STUDY_START_LEN > DBM_MAX_STEP_RESULT_LEN ? DBM_MAX_STUDY_START_LEN \
										  : DBM_MAX_STEP_RESULT_LEN)
/* COBS worst case adds one overhead byte per 254 payload bytes, plus a leading code byte
 * and a trailing 0x00 delimiter. */
#define DBM_MAX_FRAME_LEN (DBM_MAX_RAW_LEN + (DBM_MAX_RAW_LEN / 254) + 2)

/* Append-only (embarch-study-designer/design.md §3 decision 10) — matches
 * `DevBenchMessage`'s variant order exactly; postcard encodes this as the
 * enum's varint discriminant, so the order here must never change.
 * DBM_TAG_STREAM_CHUNK_BATCH = 9 (`StreamChunkBatch`) is deliberately not
 * implemented yet — not needed until real power/waveform sampling exists. */
enum dbm_tag {
	DBM_TAG_HELLO = 0,
	DBM_TAG_HELLO_ACK = 1,
	DBM_TAG_STREAM_START = 2,
	DBM_TAG_STREAM_CHUNK = 3,
	DBM_TAG_STREAM_END = 4,
	DBM_TAG_LOG_LINE = 5,
	DBM_TAG_STUDY_START = 6,
	DBM_TAG_STEP_RESULT = 7,
	DBM_TAG_STUDY_DONE = 8,
	/* DBM_TAG_STREAM_CHUNK_BATCH = 9, -- not implemented, see above */
};

enum dbm_stream_channel {
	DBM_CHANNEL_POWER = 0,
	DBM_CHANNEL_SENSOR_WAVEFORM = 1,
};

/* Mirrors embarch-study-designer's `Unit` (src/sample.rs). Append-only, same
 * wire-compatibility rule as `dbm_tag` above. */
enum dbm_unit {
	DBM_UNIT_MILLIAMPS = 0,
	DBM_UNIT_VOLTS = 1,
	DBM_UNIT_MILLIWATTS = 2,
	DBM_UNIT_RAW = 3,
};

/* `Hello` lost `steps_crc` (moved to `StudyStart` — embarch-study-designer
 * schema v3, embarch-study-designer/design.md §3 decisions 24/27). */
struct dbm_hello {
	uint32_t schema_version;
	uint64_t host_utc_ms;
};

struct dbm_hello_ack {
	uint32_t schema_version;
	bool compatible;
	/* NUL-terminated; wire form has no NUL, `+1` is this struct's own headroom. */
	char firmware_version[DBM_MAX_FIRMWARE_VERSION_LEN + 1];
};

struct dbm_stream_start_end {
	uint32_t step_index;
	enum dbm_stream_channel channel;
};

/* Mirrors embarch-study-designer's `Sample` (src/sample.rs); gained `unit`/`channel_id`
 * in schema v3 (design.md §3 decision 27). */
struct dbm_sample {
	uint64_t rx_utc_ms;
	float value;
	enum dbm_unit unit;
	uint8_t channel_id;
};

struct dbm_log_line {
	char text[DBM_MAX_LOG_LINE_LEN + 1];
};

/* Mirrors the FFI-side EssdStep/EssdBleAdvertiseAction shape 1:1 -- see
 * study_ffi.h. Only BleAdvertise is representable here for now (decision 21's
 * initial scope) -- a step whose Action isn't BleAdvertise fails the whole
 * StudyStart decode (see dbm_decode_frame's doc comment).
 */
struct dbm_ble_advertise_action {
	char local_name[DBM_MAX_LOCAL_NAME_LEN + 1]; /* NUL-terminated */
	bool has_local_name;
	uint16_t adv_interval_ms;
};

struct dbm_step {
	char name[DBM_MAX_NAME_LEN + 1];
	uint32_t timeout_ms;
	bool continue_on_fail;
	struct dbm_ble_advertise_action action;
};

struct dbm_study_start {
	uint32_t steps_len;
	struct dbm_step steps[DBM_MAX_STEPS_PER_STUDY];
	uint32_t steps_crc;
	/* Not part of the wire format -- set by dbm_decode_frame itself
	 * (design.md §3 decision 17/19): whether the just-decoded `steps`
	 * recompute to `steps_crc`. `false` on a genuine mismatch; also `false`
	 * (with `steps_len` left at 0) when decoding stopped early because a
	 * step's action wasn't BleAdvertise -- see dbm_decode_frame's own doc
	 * comment for why CRC can't be computed in that case. */
	bool steps_crc_valid;
	/* Set when a step's Action isn't BleAdvertise (decision 21's initial
	 * scope) -- decode still returns 0 (a well-formed StudyStart arrived),
	 * but the caller must not dispatch it. */
	bool has_unsupported_action;
};

struct dbm_outcome {
	uint8_t tag; /* 0=Pass, 1=Fail, 2=TimedOut */
	char fail_reason[DBM_MAX_FAIL_REASON_LEN + 1]; /* valid only if tag == 1 */
};

struct dbm_step_result_payload {
	char step_name[DBM_MAX_NAME_LEN + 1];
	struct dbm_outcome outcome;
	bool has_captured_data;
	uint8_t captured_data[DBM_MAX_PAYLOAD_LEN];
	uint32_t captured_data_len;
	/* power_samples_ref/waveform_ref: this firmware never sets these
	 * (decision 21's scope has no power/waveform capture yet) -- always
	 * encoded as None (0x00) on the wire; decode skips over a Some if one
	 * were ever received (a length-prefixed string), no C-side field to
	 * hold it. */
};

struct dbm_step_result {
	uint32_t step_index;
	struct dbm_step_result_payload result;
};

struct dbm_study_done {
	bool completed;
};

struct dev_bench_message {
	enum dbm_tag tag;
	union {
		struct dbm_hello hello;
		struct dbm_hello_ack hello_ack;
		struct dbm_stream_start_end stream_start;
		struct dbm_sample stream_chunk;
		struct dbm_stream_start_end stream_end;
		struct dbm_log_line log_line;
		struct dbm_study_start study_start;
		struct dbm_step_result step_result;
		struct dbm_study_done study_done;
	};
};

/* Encodes `msg` as postcard bytes wrapped in a COBS frame, including the
 * trailing 0x00 delimiter. Returns the frame length (> 0) on success, or a
 * negative value if `out_cap` is too small for the encoded frame. */
int dbm_encode_frame(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap);

/* Decodes one COBS-stuffed frame into `out`. `in`/`in_len` must be exactly one
 * frame's stuffed bytes, WITHOUT the trailing 0x00 delimiter — the caller (the
 * UART RX loop, which scans for the 0x00 frame boundary) strips it first, so
 * this function stays testable without any I/O involved. Returns 0 on
 * success; a negative value on a malformed frame, unknown tag, truncated
 * message, or a string field too long for its buffer.
 *
 * For DBM_TAG_STUDY_START specifically (embarch-study-designer/design.md §3
 * decisions 17/19, embarch-dev-bench/design.md §3 decision 21): each step's
 * Action must be BleAdvertise (decision 21's initial scope) — hitting any
 * other action kind stops decoding that step immediately and returns 0 with
 * `out->study_start.has_unsupported_action` set (`steps_len` reflects only
 * the steps successfully decoded before that point, `steps_crc_valid` is
 * left `false` since the CRC can't be computed without decoding every step's
 * raw bytes). Otherwise `steps_crc_valid` reports whether the CRC-32 (ISO-HDLC)
 * computed over the raw wire bytes of the decoded `steps` matches the
 * `steps_crc` field also received on the wire — independently reproducing
 * embarch-study-designer's own `steps_crc()` (src/crc.rs), which hashes each
 * step's postcard encoding concatenated in order; since postcard's encoding
 * is canonical, hashing each step's original as-received bytes is bit-for-bit
 * identical to hashing a fresh re-encoding of the same decoded value, so this
 * needs no FFI round-trip into embarch-study-designer to get the identical
 * answer (see study_ffi.h's own note on why study_ffi_decode_study --
 * decoding a `Study`, not a `StudyStart` -- doesn't fit this call site). */
int dbm_decode_frame(const uint8_t *in, size_t in_len, struct dev_bench_message *out);

#endif /* EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_ */
