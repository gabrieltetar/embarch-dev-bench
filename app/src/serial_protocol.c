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
		WRITE_VARINT(msg->hello.steps_crc);
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
		return 0;
	case DBM_TAG_STREAM_END:
		WRITE_VARINT(msg->stream_end.step_index);
		WRITE_VARINT(msg->stream_end.channel);
		return 0;
	case DBM_TAG_LOG_LINE:
		return pc_write_bytes((const uint8_t *)msg->log_line.text,
				       strlen(msg->log_line.text), out, out_cap, pos);
	default:
		return -1;
	}

#undef WRITE_VARINT
}

int dbm_encode_frame(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap)
{
	uint8_t raw[DBM_MAX_RAW_LEN];
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
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello.steps_crc = (uint32_t)tmp;
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
		return pc_read_f32(raw, raw_len, &pos, &msg->stream_chunk.value);
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
	default:
		return -1;
	}
}

int dbm_decode_frame(const uint8_t *in, size_t in_len, struct dev_bench_message *out)
{
	uint8_t raw[DBM_MAX_RAW_LEN];

	if (in_len == 0 || in_len > DBM_MAX_FRAME_LEN) {
		return -1;
	}
	size_t raw_len = cobs_decode(in, in_len, raw, sizeof(raw));

	if (raw_len == 0 || raw_len > sizeof(raw)) {
		return -1;
	}
	return decode_body(raw, raw_len, out);
}
