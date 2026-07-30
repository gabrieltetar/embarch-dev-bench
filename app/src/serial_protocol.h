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
 * (embarch-study-designer/src/limits.rs) — kept in sync by hand until decision 8's
 * west-module wiring lets this firmware pull the constants directly from that crate. */
#define DBM_MAX_FIRMWARE_VERSION_LEN 32
#define DBM_MAX_LOG_LINE_LEN 128

/* Largest single postcard-encoded (pre-COBS) DevBenchMessage this firmware sends/receives
 * (LogLine's 128-byte text dominates), plus its tag/length overhead. */
#define DBM_MAX_RAW_LEN (DBM_MAX_LOG_LINE_LEN + 8)
/* COBS worst case adds one overhead byte per 254 payload bytes, plus a leading code byte
 * and a trailing 0x00 delimiter. */
#define DBM_MAX_FRAME_LEN (DBM_MAX_RAW_LEN + (DBM_MAX_RAW_LEN / 254) + 2)

/* Append-only (embarch-study-designer/design.md §3 decision 10) — matches
 * `DevBenchMessage`'s variant order exactly; postcard encodes this as the
 * enum's varint discriminant, so the order here must never change. */
enum dbm_tag {
	DBM_TAG_HELLO = 0,
	DBM_TAG_HELLO_ACK = 1,
	DBM_TAG_STREAM_START = 2,
	DBM_TAG_STREAM_CHUNK = 3,
	DBM_TAG_STREAM_END = 4,
	DBM_TAG_LOG_LINE = 5,
};

enum dbm_stream_channel {
	DBM_CHANNEL_POWER = 0,
	DBM_CHANNEL_SENSOR_WAVEFORM = 1,
};

struct dbm_hello {
	uint32_t schema_version;
	uint64_t host_utc_ms;
	uint32_t steps_crc;
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

/* Mirrors embarch-study-designer's `Sample` (src/sample.rs). */
struct dbm_sample {
	uint64_t rx_utc_ms;
	float value;
};

struct dbm_log_line {
	char text[DBM_MAX_LOG_LINE_LEN + 1];
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
 * message, or a string field too long for its buffer. */
int dbm_decode_frame(const uint8_t *in, size_t in_len, struct dev_bench_message *out);

#endif /* EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_ */
