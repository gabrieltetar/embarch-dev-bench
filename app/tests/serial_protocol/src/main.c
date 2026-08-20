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
		.hello = {.schema_version = 2, .host_utc_ms = 1753000000000ULL},
	};
	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_HELLO, "wrong tag");
	zassert_equal(decoded.hello.schema_version, 2, "schema_version mismatch");
	zassert_equal(decoded.hello.host_utc_ms, 1753000000000ULL, "host_utc_ms mismatch");
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
		.stream_chunk = {.rx_utc_ms = 42,
				 .value = 3.3f,
				 .unit = DBM_UNIT_MILLIAMPS,
				 .channel_id = 5},
	};
	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_STREAM_CHUNK, "wrong tag");
	zassert_equal(decoded.stream_chunk.rx_utc_ms, 42, "rx_utc_ms mismatch");
	zassert_within(decoded.stream_chunk.value, 3.3f, 0.0001f, "value mismatch");
	zassert_equal(decoded.stream_chunk.unit, DBM_UNIT_MILLIAMPS, "unit mismatch");
	zassert_equal(decoded.stream_chunk.channel_id, 5, "channel_id mismatch");
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

/* dev_bench_message's union is large now that DBM_TAG_STUDY_START embeds a full
 * DBM_MAX_STEPS_PER_STUDY-sized struct dbm_step array -- `static`, not a stack
 * local, to fit this test suite's own thread stack (see app/prj.conf's/this
 * test's own prj.conf CONFIG_ZTEST_STACK_SIZE bump). */
static struct dev_bench_message study_start_msg;
static struct dev_bench_message study_start_decoded;

ZTEST(serial_protocol, test_study_start_round_trip_one_step)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 1;
	strcpy(study_start_msg.study_start.steps[0].name, "advertise");
	study_start_msg.study_start.steps[0].timeout_ms = 5000;
	study_start_msg.study_start.steps[0].continue_on_fail = false;
	study_start_msg.study_start.steps[0].action.has_local_name = true;
	strcpy(study_start_msg.study_start.steps[0].action.local_name, "embarch-dev-bench");
	study_start_msg.study_start.steps[0].action.adv_interval_ms = 100;
	/* The real steps_crc embarch-study-designer's own steps_crc() computes
	 * for this exact single-step content (name "advertise",
	 * BleAdvertise{local_name: Some("embarch-dev-bench"), service_uuids: [],
	 * adv_interval_ms: 100}, timeout_ms 5000, power_sample: None,
	 * continue_on_fail: false) -- confirmed against the real crate, not
	 * guessed, so this test actually proves this file's CRC-32 matches that
	 * crate's, not just that this file agrees with itself. */
	study_start_msg.study_start.steps_crc = 0x889FAF61;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.tag, DBM_TAG_STUDY_START, "wrong tag");
	zassert_equal(study_start_decoded.study_start.steps_len, 1, "steps_len mismatch");
	zassert_false(study_start_decoded.study_start.has_unsupported_action,
		      "should not flag an unsupported action");
	zassert_str_equal(study_start_decoded.study_start.steps[0].name, "advertise", "name mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].timeout_ms, 5000, "timeout_ms mismatch");
	zassert_true(study_start_decoded.study_start.steps[0].action.has_local_name,
		     "has_local_name mismatch");
	zassert_str_equal(study_start_decoded.study_start.steps[0].action.local_name,
			   "embarch-dev-bench", "local_name mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action.adv_interval_ms, 100,
		      "adv_interval_ms mismatch");
	zassert_equal(study_start_decoded.study_start.steps_crc, 0x889FAF61, "steps_crc mismatch");
	zassert_true(study_start_decoded.study_start.steps_crc_valid, "steps_crc_valid should be true");

	/* Flipping a bit must be caught. */
	study_start_msg.study_start.steps_crc ^= 1;
	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_false(study_start_decoded.study_start.steps_crc_valid,
		      "corrupted steps_crc should not validate");
}

ZTEST(serial_protocol, test_study_start_round_trip_two_steps)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 2;

	strcpy(study_start_msg.study_start.steps[0].name, "advertise-1");
	study_start_msg.study_start.steps[0].timeout_ms = 5000;
	study_start_msg.study_start.steps[0].continue_on_fail = false;
	study_start_msg.study_start.steps[0].action.has_local_name = true;
	strcpy(study_start_msg.study_start.steps[0].action.local_name, "dev-bench");
	study_start_msg.study_start.steps[0].action.adv_interval_ms = 100;

	strcpy(study_start_msg.study_start.steps[1].name, "advertise-2");
	study_start_msg.study_start.steps[1].timeout_ms = 2000;
	study_start_msg.study_start.steps[1].continue_on_fail = true;
	study_start_msg.study_start.steps[1].action.has_local_name = false;
	study_start_msg.study_start.steps[1].action.adv_interval_ms = 250;

	/* Real steps_crc for this exact two-step content, confirmed against
	 * embarch-study-designer's own steps_crc() -- see the one-step test's
	 * comment above. */
	study_start_msg.study_start.steps_crc = 0xAD7A131B;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.study_start.steps_len, 2, "steps_len mismatch");
	zassert_str_equal(study_start_decoded.study_start.steps[1].name, "advertise-2", "name mismatch");
	zassert_false(study_start_decoded.study_start.steps[1].action.has_local_name,
		      "has_local_name mismatch");
	zassert_true(study_start_decoded.study_start.steps[1].continue_on_fail,
		     "continue_on_fail mismatch");
	zassert_true(study_start_decoded.study_start.steps_crc_valid, "steps_crc_valid should be true");
}

ZTEST(serial_protocol, test_study_start_rejects_too_many_steps)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = DBM_MAX_STEPS_PER_STUDY + 1;

	uint8_t frame[DBM_MAX_FRAME_LEN];

	zassert_true(dbm_encode_frame(&study_start_msg, frame, sizeof(frame)) < 0,
		     "steps_len beyond struct dbm_step[]'s bound should be rejected, not read OOB");
}

ZTEST(serial_protocol, test_step_result_pass_round_trip)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_STEP_RESULT};

	msg.step_result.step_index = 0;
	strcpy(msg.step_result.result.step_name, "advertise");
	msg.step_result.result.outcome.tag = 0; /* Pass */
	msg.step_result.result.has_captured_data = false;

	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_STEP_RESULT, "wrong tag");
	zassert_equal(decoded.step_result.step_index, 0, "step_index mismatch");
	zassert_str_equal(decoded.step_result.result.step_name, "advertise", "step_name mismatch");
	zassert_equal(decoded.step_result.result.outcome.tag, 0, "outcome tag mismatch");
	zassert_false(decoded.step_result.result.has_captured_data, "has_captured_data mismatch");
}

ZTEST(serial_protocol, test_step_result_fail_round_trip)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_STEP_RESULT};

	msg.step_result.step_index = 2;
	strcpy(msg.step_result.result.step_name, "connect");
	msg.step_result.result.outcome.tag = 1; /* Fail */
	strcpy(msg.step_result.result.outcome.fail_reason, "no adv seen");
	msg.step_result.result.has_captured_data = true;
	msg.step_result.result.captured_data[0] = 0xAB;
	msg.step_result.result.captured_data[1] = 0xCD;
	msg.step_result.result.captured_data_len = 2;

	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.step_result.step_index, 2, "step_index mismatch");
	zassert_equal(decoded.step_result.result.outcome.tag, 1, "outcome tag mismatch");
	zassert_str_equal(decoded.step_result.result.outcome.fail_reason, "no adv seen",
			   "fail_reason mismatch");
	zassert_true(decoded.step_result.result.has_captured_data, "has_captured_data mismatch");
	zassert_equal(decoded.step_result.result.captured_data_len, 2, "captured_data_len mismatch");
	zassert_equal(decoded.step_result.result.captured_data[0], 0xAB, "captured_data[0] mismatch");
	zassert_equal(decoded.step_result.result.captured_data[1], 0xCD, "captured_data[1] mismatch");
}

ZTEST(serial_protocol, test_study_done_round_trip)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_STUDY_DONE, .study_done = {.completed = true}};
	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_STUDY_DONE, "wrong tag");
	zassert_true(decoded.study_done.completed, "completed mismatch");

	msg.study_done.completed = false;
	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_false(decoded.study_done.completed, "completed mismatch");
}
