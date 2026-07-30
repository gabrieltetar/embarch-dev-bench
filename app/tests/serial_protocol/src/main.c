/* Unit tests for serial_protocol.c's hand-written COBS+postcard implementation.
 *
 * embarch-dev-bench/design.md §3 decision 16: this is NOT re-deriving whether
 * COBS/postcard as a *format* is sound -- embarch-study-designer's own
 * round-trip tests already cover that for the Rust side. This C
 * implementation is new, independently hand-written code describing the same
 * wire format, and needs its own tests to catch bugs in that translation.
 * Runs on native_sim (`west build -b native_sim app/tests/serial_protocol`).
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "serial_protocol.h"

ZTEST_SUITE(serial_protocol, NULL, NULL, NULL, NULL, NULL);

static int round_trip(const struct dev_bench_message *msg, struct dev_bench_message *out)
{
	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(msg, frame, sizeof(frame));

	zassert_true(frame_len > 0, "encode failed");
	zassert_equal(frame[frame_len - 1], 0x00, "frame missing trailing delimiter");
	return dbm_decode_frame(frame, (size_t)(frame_len - 1), out);
}

ZTEST(serial_protocol, test_hello_round_trip)
{
	struct dev_bench_message msg = {
		.tag = DBM_TAG_HELLO,
		.hello = {.schema_version = 2, .host_utc_ms = 1753000000000ULL, .steps_crc = 0xDEADBEEF},
	};
	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_HELLO, "wrong tag");
	zassert_equal(decoded.hello.schema_version, 2, "schema_version mismatch");
	zassert_equal(decoded.hello.host_utc_ms, 1753000000000ULL, "host_utc_ms mismatch");
	zassert_equal(decoded.hello.steps_crc, 0xDEADBEEF, "steps_crc mismatch");
}

ZTEST(serial_protocol, test_hello_ack_round_trip)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_HELLO_ACK};

	msg.hello_ack.schema_version = 2;
	msg.hello_ack.compatible = true;
	strcpy(msg.hello_ack.firmware_version, "nrf54l15dk-g1a2b3c");

	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_HELLO_ACK, "wrong tag");
	zassert_equal(decoded.hello_ack.schema_version, 2, "schema_version mismatch");
	zassert_true(decoded.hello_ack.compatible, "compatible mismatch");
	zassert_str_equal(decoded.hello_ack.firmware_version, "nrf54l15dk-g1a2b3c",
			   "firmware_version mismatch");
}

ZTEST(serial_protocol, test_stream_start_and_end_round_trip)
{
	struct dev_bench_message start = {
		.tag = DBM_TAG_STREAM_START,
		.stream_start = {.step_index = 7, .channel = DBM_CHANNEL_SENSOR_WAVEFORM},
	};
	struct dev_bench_message decoded;

	zassert_equal(round_trip(&start, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_STREAM_START, "wrong tag");
	zassert_equal(decoded.stream_start.step_index, 7, "step_index mismatch");
	zassert_equal(decoded.stream_start.channel, DBM_CHANNEL_SENSOR_WAVEFORM, "channel mismatch");

	struct dev_bench_message end = {
		.tag = DBM_TAG_STREAM_END,
		.stream_end = {.step_index = 7, .channel = DBM_CHANNEL_POWER},
	};

	zassert_equal(round_trip(&end, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_STREAM_END, "wrong tag");
	zassert_equal(decoded.stream_end.channel, DBM_CHANNEL_POWER, "channel mismatch");
}

ZTEST(serial_protocol, test_stream_chunk_round_trip)
{
	struct dev_bench_message msg = {
		.tag = DBM_TAG_STREAM_CHUNK,
		.stream_chunk = {.rx_utc_ms = 42, .value = 3.3f},
	};
	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_STREAM_CHUNK, "wrong tag");
	zassert_equal(decoded.stream_chunk.rx_utc_ms, 42, "rx_utc_ms mismatch");
	zassert_within(decoded.stream_chunk.value, 3.3f, 0.0001f, "value mismatch");
}

ZTEST(serial_protocol, test_log_line_round_trip)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_LOG_LINE};

	strcpy(msg.log_line.text, "ble: connected");

	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_LOG_LINE, "wrong tag");
	zassert_str_equal(decoded.log_line.text, "ble: connected", "text mismatch");
}

ZTEST(serial_protocol, test_decode_rejects_garbage)
{
	uint8_t garbage[] = {0xFF, 0xFF, 0xFF, 0xFF};
	struct dev_bench_message decoded;

	zassert_true(dbm_decode_frame(garbage, sizeof(garbage), &decoded) != 0,
		     "garbage should not decode successfully");
}

ZTEST(serial_protocol, test_decode_rejects_empty_input)
{
	struct dev_bench_message decoded;

	zassert_true(dbm_decode_frame(NULL, 0, &decoded) != 0, "empty input should be rejected");
}

ZTEST(serial_protocol, test_encode_rejects_oversized_log_line)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_LOG_LINE};

	memset(msg.log_line.text, 'x', sizeof(msg.log_line.text) - 1);
	msg.log_line.text[sizeof(msg.log_line.text) - 1] = '\0';

	uint8_t frame[4]; /* deliberately too small */

	zassert_true(dbm_encode_frame(&msg, frame, sizeof(frame)) < 0,
		     "encode into an undersized buffer should fail, not overflow it");
}
