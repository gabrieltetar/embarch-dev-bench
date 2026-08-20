#include "serial_protocol.h"

#include <string.h>

/* ---- postcard-compatible primitives -------------------------------------
 *
 * postcard encodes unsigned integers (including enum discriminants, which
 * serde treats as u32) as ULEB128 varints, `bool` as a single 0/1 byte,
 * `f32`/`f64` as raw little-endian bytes (no varint), and `&str`/`String` as
 * a ULEB128 byte-length prefix followed by raw UTF-8 bytes. This assumes a
 * little-endian target for float encoding, true for both this firmware's
 * targets (Cortex-M33 in Zephyr's default configuration, and native_sim's
 * host x86/x86_64).
 */

static size_t pc_write_varint(uint64_t value, uint8_t *out)
{
	size_t n = 0;

	while (value >= 0x80) {
		out[n++] = (uint8_t)((value & 0x7F) | 0x80);
		value >>= 7;
	}
	out[n++] = (uint8_t)(value & 0x7F);
	return n;
}

static int pc_read_varint(const uint8_t *in, size_t in_len, size_t *pos, uint64_t *out)
{
	uint64_t result = 0;
	int shift = 0;

	while (1) {
		if (*pos >= in_len) {
			return -1;
		}
		uint8_t byte = in[(*pos)++];

		result |= (uint64_t)(byte & 0x7F) << shift;
		if (!(byte & 0x80)) {
			break;
		}
		shift += 7;
		if (shift >= 64) {
			return -1;
		}
	}
	*out = result;
	return 0;
}

static int pc_write_bytes(const uint8_t *bytes, size_t len, uint8_t *out, size_t out_cap,
			   size_t *pos)
{
	uint8_t len_buf[10];
	size_t len_n = pc_write_varint((uint64_t)len, len_buf);

	if (*pos + len_n + len > out_cap) {
		return -1;
	}
	memcpy(out + *pos, len_buf, len_n);
	*pos += len_n;
	memcpy(out + *pos, bytes, len);
	*pos += len;
	return 0;
}

/* Reads a length-prefixed string into `out` (capacity `out_cap`, including
 * room for the NUL this function appends). Rejects a wire string that
 * wouldn't fit rather than truncating it silently. */
static int pc_read_str(const uint8_t *in, size_t in_len, size_t *pos, char *out, size_t out_cap)
{
	uint64_t len;

	if (pc_read_varint(in, in_len, pos, &len) != 0) {
		return -1;
	}
	if (len >= out_cap || *pos + len > in_len) {
		return -1;
	}
	memcpy(out, in + *pos, (size_t)len);
	out[len] = '\0';
	*pos += (size_t)len;
	return 0;
}

static size_t pc_write_f32(float value, uint8_t *out)
{
	uint8_t bytes[4];

	memcpy(bytes, &value, 4);
	memcpy(out, bytes, 4);
	return 4;
}

static int pc_read_f32(const uint8_t *in, size_t in_len, size_t *pos, float *out)
{
	if (*pos + 4 > in_len) {
		return -1;
	}
	memcpy(out, in + *pos, 4);
	*pos += 4;
	return 0;
}

/* Reads (and discards) a length-prefixed sequence of `elem_size`-byte fixed
 * elements -- postcard's `Vec<T,N>`/`&str`/`String` encoding for any `T`
 * with a compile-time-known size (`elem_size = 1` covers a `String`'s raw
 * UTF-8 bytes, `elem_size = 16` covers `Vec<Uuid,4>`'s fixed 16-byte
 * elements). Used where this firmware doesn't need the field's contents but
 * must still walk past it correctly to decode whatever comes next. */
static int pc_skip_len_prefixed(const uint8_t *in, size_t in_len, size_t *pos, size_t elem_size)
{
	uint64_t len;

	if (pc_read_varint(in, in_len, pos, &len) != 0) {
		return -1;
	}
	size_t n = (size_t)len * elem_size;

	if (*pos + n > in_len) {
		return -1;
	}
	*pos += n;
	return 0;
}

/* CRC-32 (ISO-HDLC / the common "CRC-32" used by zip/gzip/PNG/Ethernet;
 * poly 0xEDB88320 reflected, init/xorout 0xFFFFFFFF) -- matches the `crc`
 * crate's `CRC_32_ISO_HDLC` embarch-study-designer's `steps_crc()` (src/crc.rs)
 * uses. Bitwise, not table-driven: this only ever runs once per received
 * `StudyStart`, not a hot path worth the table's static footprint. */
static uint32_t dbm_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			uint32_t mask = -(crc & 1u);

			crc = (crc >> 1) ^ (0xEDB88320u & mask);
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

/* ---- COBS -----------------------------------------------------------------
 *
 * Standard Consistent Overhead Byte Stuffing (embarch-study-designer/design.md
 * §3 decision 10). `cobs_encode` does not append the trailing 0x00 frame
 * delimiter itself — `dbm_encode_frame` does, once, after calling this.
 */

static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < length) {
		if (input[read_index] == 0) {
			output[code_index] = code;
			code = 1;
			code_index = write_index++;
			read_index++;
		} else {
			output[write_index++] = input[read_index++];
			code++;
			if (code == 0xFF) {
				output[code_index] = code;
				code = 1;
				code_index = write_index++;
			}
		}
	}
	output[code_index] = code;
	return write_index;
}

/* Returns the decoded length, or 0 on a malformed frame or an output that
 * wouldn't fit in `output_cap` -- bounds-checked on every write, since `input`
 * is untrusted (bytes off a serial link, possibly corrupted or crafted) and
 * must never be able to overflow the fixed-size decode buffer regardless of
 * what `length` claims. */
static size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t output_cap)
{
	size_t read_index = 0;
	size_t write_index = 0;

	while (read_index < length) {
		uint8_t code = input[read_index];

		if (code == 0 || (read_index + code > length && code != 1)) {
			return 0;
		}
		read_index++;
		for (uint8_t i = 1; i < code; i++) {
			if (write_index >= output_cap) {
				return 0;
			}
			output[write_index++] = input[read_index++];
		}
		if (code != 0xFF && read_index != length) {
			if (write_index >= output_cap) {
				return 0;
			}
			output[write_index++] = 0;
		}
	}
	return write_index;
}

/* ---- DevBenchMessage encode/decode ---------------------------------------- */

static int encode_body(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap,
			size_t *pos)
{
	uint8_t varint_buf[10];

#define WRITE_VARINT(value)                                                                       \
	do {                                                                                       \
		size_t n = pc_write_varint((uint64_t)(value), varint_buf);                        \
		if (*pos + n > out_cap) {                                                         \
			return -1;                                                                \
		}                                                                                  \
		memcpy(out + *pos, varint_buf, n);                                                \
		*pos += n;                                                                        \
	} while (0)

	WRITE_VARINT(msg->tag);

	switch (msg->tag) {
	case DBM_TAG_HELLO:
		WRITE_VARINT(msg->hello.schema_version);
		WRITE_VARINT(msg->hello.host_utc_ms);
		return 0;
	case DBM_TAG_HELLO_ACK:
		WRITE_VARINT(msg->hello_ack.schema_version);
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->hello_ack.compatible ? 1 : 0;
		return pc_write_bytes((const uint8_t *)msg->hello_ack.firmware_version,
				       strlen(msg->hello_ack.firmware_version), out, out_cap, pos);
	case DBM_TAG_STREAM_START:
		WRITE_VARINT(msg->stream_start.step_index);
		WRITE_VARINT(msg->stream_start.channel);
		return 0;
	case DBM_TAG_STREAM_CHUNK:
		WRITE_VARINT(msg->stream_chunk.rx_utc_ms);
		if (*pos + 4 > out_cap) {
			return -1;
		}
		*pos += pc_write_f32(msg->stream_chunk.value, out + *pos);
		WRITE_VARINT(msg->stream_chunk.unit);
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->stream_chunk.channel_id; /* u8: raw byte, not varint */
		return 0;
	case DBM_TAG_STREAM_END:
		WRITE_VARINT(msg->stream_end.step_index);
		WRITE_VARINT(msg->stream_end.channel);
		return 0;
	case DBM_TAG_LOG_LINE:
		return pc_write_bytes((const uint8_t *)msg->log_line.text,
				       strlen(msg->log_line.text), out, out_cap, pos);
	case DBM_TAG_STUDY_START:
		/* dev-bench never sends StudyStart in practice (Core is the only
		 * sender) -- this encode path exists for this file's own
		 * round-trip tests. `service_uuids`/`power_sample` aren't stored
		 * on `struct dbm_step` (decision 21's scope), so they're always
		 * encoded empty/None here. */
		if (msg->study_start.steps_len > DBM_MAX_STEPS_PER_STUDY) {
			return -1; /* would read past struct dbm_study_start.steps[]'s bound */
		}
		WRITE_VARINT(msg->study_start.steps_len);
		for (uint32_t i = 0; i < msg->study_start.steps_len; i++) {
			const struct dbm_step *step = &msg->study_start.steps[i];

			if (pc_write_bytes((const uint8_t *)step->name, strlen(step->name), out,
					    out_cap, pos) != 0) {
				return -1;
			}
			WRITE_VARINT(0); /* Action::BleAdvertise tag */
			if (*pos + 1 > out_cap) {
				return -1;
			}
			out[(*pos)++] = step->action.has_local_name ? 1 : 0;
			if (step->action.has_local_name &&
			    pc_write_bytes((const uint8_t *)step->action.local_name,
					    strlen(step->action.local_name), out, out_cap, pos) != 0) {
				return -1;
			}
			WRITE_VARINT(0); /* service_uuids: Vec<Uuid,4>, always empty */
			WRITE_VARINT(step->action.adv_interval_ms);
			WRITE_VARINT(step->timeout_ms);
			if (*pos + 1 > out_cap) {
				return -1;
			}
			out[(*pos)++] = 0; /* power_sample: Option<PowerSampleWindow>, always None */
			if (*pos + 1 > out_cap) {
				return -1;
			}
			out[(*pos)++] = step->continue_on_fail ? 1 : 0;
		}
		WRITE_VARINT(msg->study_start.steps_crc);
		return 0;
	case DBM_TAG_STEP_RESULT: {
		const struct dbm_step_result_payload *r = &msg->step_result.result;

		WRITE_VARINT(msg->step_result.step_index);
		if (pc_write_bytes((const uint8_t *)r->step_name, strlen(r->step_name), out, out_cap,
				    pos) != 0) {
			return -1;
		}
		WRITE_VARINT(r->outcome.tag);
		if (r->outcome.tag == 1 &&
		    pc_write_bytes((const uint8_t *)r->outcome.fail_reason,
				    strlen(r->outcome.fail_reason), out, out_cap, pos) != 0) {
			return -1;
		}
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = r->has_captured_data ? 1 : 0;
		if (r->has_captured_data &&
		    pc_write_bytes(r->captured_data, r->captured_data_len, out, out_cap, pos) != 0) {
			return -1;
		}
		/* power_samples_ref/waveform_ref: Option<String>, always None
		 * (this firmware has no power/waveform capture yet, decision 21). */
		if (*pos + 2 > out_cap) {
			return -1;
		}
		out[(*pos)++] = 0;
		out[(*pos)++] = 0;
		return 0;
	}
	case DBM_TAG_STUDY_DONE:
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->study_done.completed ? 1 : 0;
		return 0;
	default:
		return -1;
	}

#undef WRITE_VARINT
}

int dbm_encode_frame(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap)
{
	/* `static`, not a stack-local array: DBM_MAX_RAW_LEN now scales with
	 * DBM_MAX_STEPS_PER_STUDY (StudyStart's own worst case), too large for
	 * a small embedded call stack, especially with dbm_decode_frame's own
	 * same-size scratch buffer potentially live in a caller's frame at the
	 * same time (e.g. main.c's send_message). Safe because this firmware's
	 * serial link is driven from a single thread, one message at a time
	 * (main.c's own RX loop) -- same posture as receive_message's static
	 * rx_buf in main.c. */
	static uint8_t raw[DBM_MAX_RAW_LEN];
	size_t raw_len = 0;

	if (encode_body(msg, raw, sizeof(raw), &raw_len) != 0) {
		return -1;
	}

	/* cobs_encode's worst case output is raw_len + ceil(raw_len / 254) + 1. */
	size_t cobs_cap = raw_len + (raw_len / 254) + 2;

	if (cobs_cap + 1 > out_cap) {
		return -1;
	}
	size_t cobs_len = cobs_encode(raw, raw_len, out);

	out[cobs_len] = 0x00;
	return (int)(cobs_len + 1);
}

static int decode_body(const uint8_t *raw, size_t raw_len, struct dev_bench_message *msg)
{
	size_t pos = 0;
	uint64_t tag;

	if (pc_read_varint(raw, raw_len, &pos, &tag) != 0) {
		return -1;
	}
	msg->tag = (enum dbm_tag)tag;

	uint64_t tmp;

	switch (msg->tag) {
	case DBM_TAG_HELLO:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello.schema_version = (uint32_t)tmp;
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello.host_utc_ms = tmp;
		return 0;
	case DBM_TAG_HELLO_ACK:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello_ack.schema_version = (uint32_t)tmp;
		if (pos >= raw_len) {
			return -1;
		}
		msg->hello_ack.compatible = raw[pos++] != 0;
		return pc_read_str(raw, raw_len, &pos, msg->hello_ack.firmware_version,
				    sizeof(msg->hello_ack.firmware_version));
	case DBM_TAG_STREAM_START:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->stream_start.step_index = (uint32_t)tmp;
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->stream_start.channel = (enum dbm_stream_channel)tmp;
		return 0;
	case DBM_TAG_STREAM_CHUNK:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->stream_chunk.rx_utc_ms = tmp;
		if (pc_read_f32(raw, raw_len, &pos, &msg->stream_chunk.value) != 0) {
			return -1;
		}
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->stream_chunk.unit = (enum dbm_unit)tmp;
		if (pos >= raw_len) {
			return -1;
		}
		msg->stream_chunk.channel_id = raw[pos++]; /* u8: raw byte, not varint */
		return 0;
	case DBM_TAG_STREAM_END:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->stream_end.step_index = (uint32_t)tmp;
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->stream_end.channel = (enum dbm_stream_channel)tmp;
		return 0;
	case DBM_TAG_LOG_LINE:
		return pc_read_str(raw, raw_len, &pos, msg->log_line.text,
				    sizeof(msg->log_line.text));
	case DBM_TAG_STUDY_START: {
		struct dbm_study_start *ss = &msg->study_start;

		memset(ss, 0, sizeof(*ss));

		uint64_t steps_len;

		if (pc_read_varint(raw, raw_len, &pos, &steps_len) != 0) {
			return -1;
		}
		if (steps_len > DBM_MAX_STEPS_PER_STUDY) {
			return -1;
		}

		size_t steps_start_pos = pos;
		uint32_t decoded = 0;
		bool unsupported = false;

		for (uint32_t i = 0; i < steps_len; i++) {
			struct dbm_step *step = &ss->steps[i];

			if (pc_read_str(raw, raw_len, &pos, step->name, sizeof(step->name)) != 0) {
				return -1;
			}

			uint64_t action_tag;

			if (pc_read_varint(raw, raw_len, &pos, &action_tag) != 0) {
				return -1;
			}
			if (action_tag != 0 /* Action::BleAdvertise */) {
				/* Decision 21's initial scope: this decode surface only
				 * understands BleAdvertise. Stop here -- everything after
				 * this action tag (this step's remaining fields, any
				 * further steps, steps_crc) has an unknown shape we can't
				 * safely walk past, so the whole StudyStart is rejected
				 * (see this function's own doc comment in the header). */
				unsupported = true;
				break;
			}

			if (pos >= raw_len) {
				return -1;
			}
			bool has_local_name = raw[pos++] != 0;

			if (has_local_name) {
				if (pc_read_str(raw, raw_len, &pos, step->action.local_name,
						 sizeof(step->action.local_name)) != 0) {
					return -1;
				}
			} else {
				step->action.local_name[0] = '\0';
			}
			step->action.has_local_name = has_local_name;

			/* service_uuids: Vec<Uuid,4> -- not stored, decision 21's scope. */
			if (pc_skip_len_prefixed(raw, raw_len, &pos, 16) != 0) {
				return -1;
			}

			uint64_t adv_interval_ms;

			if (pc_read_varint(raw, raw_len, &pos, &adv_interval_ms) != 0) {
				return -1;
			}
			step->action.adv_interval_ms = (uint16_t)adv_interval_ms;

			uint64_t timeout_ms;

			if (pc_read_varint(raw, raw_len, &pos, &timeout_ms) != 0) {
				return -1;
			}
			step->timeout_ms = (uint32_t)timeout_ms;

			/* power_sample: Option<PowerSampleWindow> -- not stored. */
			if (pos >= raw_len) {
				return -1;
			}
			if (raw[pos++] != 0) {
				uint64_t sample_rate_hz;

				if (pc_read_varint(raw, raw_len, &pos, &sample_rate_hz) != 0) {
					return -1;
				}
			}

			if (pos >= raw_len) {
				return -1;
			}
			step->continue_on_fail = raw[pos++] != 0;

			decoded++;
		}

		ss->steps_len = decoded;
		ss->has_unsupported_action = unsupported;

		if (unsupported) {
			ss->steps_crc_valid = false;
			return 0;
		}

		size_t steps_end_pos = pos;
		uint64_t steps_crc;

		if (pc_read_varint(raw, raw_len, &pos, &steps_crc) != 0) {
			return -1;
		}
		ss->steps_crc = (uint32_t)steps_crc;
		ss->steps_crc_valid =
			dbm_crc32(raw + steps_start_pos, steps_end_pos - steps_start_pos) == ss->steps_crc;
		return 0;
	}
	case DBM_TAG_STEP_RESULT: {
		struct dbm_step_result *sr = &msg->step_result;

		memset(sr, 0, sizeof(*sr));

		uint64_t step_index;

		if (pc_read_varint(raw, raw_len, &pos, &step_index) != 0) {
			return -1;
		}
		sr->step_index = (uint32_t)step_index;

		if (pc_read_str(raw, raw_len, &pos, sr->result.step_name,
				 sizeof(sr->result.step_name)) != 0) {
			return -1;
		}

		uint64_t outcome_tag;

		if (pc_read_varint(raw, raw_len, &pos, &outcome_tag) != 0) {
			return -1;
		}
		if (outcome_tag > 2) {
			return -1;
		}
		sr->result.outcome.tag = (uint8_t)outcome_tag;
		if (outcome_tag == 1 &&
		    pc_read_str(raw, raw_len, &pos, sr->result.outcome.fail_reason,
				sizeof(sr->result.outcome.fail_reason)) != 0) {
			return -1;
		}

		if (pos >= raw_len) {
			return -1;
		}
		bool has_captured_data = raw[pos++] != 0;

		sr->result.has_captured_data = has_captured_data;
		if (has_captured_data) {
			uint64_t captured_len;

			if (pc_read_varint(raw, raw_len, &pos, &captured_len) != 0) {
				return -1;
			}
			if (captured_len > sizeof(sr->result.captured_data) ||
			    pos + captured_len > raw_len) {
				return -1;
			}
			memcpy(sr->result.captured_data, raw + pos, (size_t)captured_len);
			pos += (size_t)captured_len;
			sr->result.captured_data_len = (uint32_t)captured_len;
		}

		/* power_samples_ref/waveform_ref: Option<String> -- this firmware
		 * always encodes None, but decode must still be able to skip a Some
		 * if one were ever received (see struct dbm_step_result_payload's
		 * own doc comment). */
		for (int i = 0; i < 2; i++) {
			if (pos >= raw_len) {
				return -1;
			}
			if (raw[pos++] != 0 && pc_skip_len_prefixed(raw, raw_len, &pos, 1) != 0) {
				return -1;
			}
		}
		return 0;
	}
	case DBM_TAG_STUDY_DONE:
		if (pos >= raw_len) {
			return -1;
		}
		msg->study_done.completed = raw[pos++] != 0;
		return 0;
	default:
		return -1;
	}
}

int dbm_decode_frame(const uint8_t *in, size_t in_len, struct dev_bench_message *out)
{
	/* `static`, not stack-local -- see dbm_encode_frame's own comment above. */
	static uint8_t raw[DBM_MAX_RAW_LEN];

	if (in_len == 0 || in_len > DBM_MAX_FRAME_LEN) {
		return -1;
	}
	size_t raw_len = cobs_decode(in, in_len, raw, sizeof(raw));

	if (raw_len == 0 || raw_len > sizeof(raw)) {
		return -1;
	}
	return decode_body(raw, raw_len, out);
}
