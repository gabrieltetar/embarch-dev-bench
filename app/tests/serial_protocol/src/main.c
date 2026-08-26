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
	strcpy(msg.hello_ack.hardware_id, "aaaaaaaabbbbbbbb");

	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_equal(decoded.tag, DBM_TAG_HELLO_ACK, "wrong tag");
	zassert_equal(decoded.hello_ack.schema_version, 2, "schema_version mismatch");
	zassert_true(decoded.hello_ack.compatible, "compatible mismatch");
	zassert_str_equal(decoded.hello_ack.firmware_version, "nrf54l15dk-g1a2b3c",
			   "firmware_version mismatch");
	zassert_str_equal(decoded.hello_ack.hardware_id, "aaaaaaaabbbbbbbb",
			   "hardware_id mismatch");
}

/* A build with no hwinfo driver reports an empty hardware_id (main.c's
 * read_hardware_id). That has to survive the wire as a zero-length string
 * rather than as an absent field -- an absent field leaves nothing after it
 * walkable, and Core's own comparison is what decides an empty ID is
 * unusable, not the encoder dropping it. */
ZTEST(serial_protocol, test_hello_ack_with_no_hardware_id_round_trips)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_HELLO_ACK};

	msg.hello_ack.schema_version = 10;
	msg.hello_ack.compatible = true;
	strcpy(msg.hello_ack.firmware_version, "native_sim-g1a2b3c");
	msg.hello_ack.hardware_id[0] = '\0';

	struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_str_equal(decoded.hello_ack.firmware_version, "native_sim-g1a2b3c",
			   "firmware_version must still decode past an empty hardware_id");
	zassert_equal(decoded.hello_ack.hardware_id[0], '\0', "hardware_id should be empty");
}

/* `HelloAck`'s wire bytes, pinned across both languages for the first time at
 * schema v10 (embarch-study-designer/design.md §3 decision 47). Like
 * `StepResult` below, this frame predates decision 36's both-languages rule
 * and so was never covered -- and `StepResult`'s own history is the argument
 * for pinning it now: this file's encoder wrote two stale `Option` bytes for
 * a whole schema version while both suites stayed green, because each agreed
 * with itself.
 *
 * embarch-study-designer pins the identical pre-COBS body in
 * `hello_ack_matches_dev_bench_firmwares_own_hand_written_encoding`.
 */
ZTEST(serial_protocol, test_hello_ack_encodes_to_the_pinned_wire_bytes)
{
	/* body: 0x01 tag, 0x0a schema_version (varint), 0x01 compatible,
	 * 0x07 + "g1a2b3c", 0x10 + "aaaaaaaabbbbbbbb". No zero bytes, so COBS
	 * is one leading code byte (len + 1) plus the data plus the
	 * delimiter. */
	static const uint8_t expected[] = {
		0x1d, 0x01, 0x0a, 0x01, 0x07, 0x67, 0x31, 0x61, 0x32, 0x62, 0x33, 0x63,
		0x10, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x62, 0x62, 0x62,
		0x62, 0x62, 0x62, 0x62, 0x62, 0x00,
	};
	struct dev_bench_message msg = {.tag = DBM_TAG_HELLO_ACK};

	msg.hello_ack.schema_version = 10;
	msg.hello_ack.compatible = true;
	strcpy(msg.hello_ack.firmware_version, "g1a2b3c");
	strcpy(msg.hello_ack.hardware_id, "aaaaaaaabbbbbbbb");

	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

/* ---- Stream taps (embarch-study-designer schema v8, that doc's §3
 * decision 39) --------------------------------------------------------
 *
 * All three of these are pinned as literal COBS frames rather than
 * round-tripped through this file's own decoder, for the reason the GATT
 * transcript already was and now more so: there *is* no C decoder for them.
 * dev-bench only ever sends a StreamOpen/StreamChunkBatch/StreamClose, and
 * the only reader is embarch-core's Rust postcard decode. A C-side round
 * trip would prove this encoder self-consistent while saying nothing about
 * whether Rust agrees, which is the only thing that matters.
 *
 * embarch-study-designer pins the identical bytes (pre-COBS) from the Rust
 * side, in `stream_open_matches_dev_bench_firmwares_own_hand_written_encoding`
 * and its two siblings, and asserts postcard both decodes them into the
 * matching `DevBenchMessage` *and* re-encodes to the same bytes. Changing a
 * shape must break both tests, in both languages -- that pairing is the whole
 * point, and it found a real discrepancy the first time it ran.
 */

ZTEST(serial_protocol, test_stream_open_encodes_to_the_pinned_wire_bytes)
{
	/* body: 0x02 tag, 0x02 id -- no zero bytes, so COBS is one leading
	 * code byte (len + 1) plus the data plus the delimiter. */
	static const uint8_t expected[] = {0x03, 0x02, 0x02, 0x00};

	struct dev_bench_message msg = {
		.tag = DBM_TAG_STREAM_OPEN,
		.stream_open = {.id = 2},
	};
	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

ZTEST(serial_protocol, test_stream_close_encodes_to_the_pinned_wire_bytes)
{
	/* body: 0x04 tag, 0x02 id (raw u8), 0x05 dropped (varint). */
	static const uint8_t expected[] = {0x04, 0x04, 0x02, 0x05, 0x00};

	struct dev_bench_message msg = {
		.tag = DBM_TAG_STREAM_CLOSE,
		.stream_close = {.id = 0, .dropped = 0},
	};

	msg.stream_close.id = 2;
	msg.stream_close.dropped = 5;

	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

ZTEST(serial_protocol, test_stream_close_with_zero_fields_encodes_to_the_pinned_wire_bytes)
{
	/* Every byte of the body but the tag is zero (0x04, 0x00, 0x00), which
	 * is the interesting case for COBS: three overhead bytes and one
	 * literal. The Rust side pins the same body in
	 * `a_stream_close_with_zero_fields_still_round_trips`. */
	static const uint8_t expected[] = {0x02, 0x04, 0x01, 0x01, 0x00};

	struct dev_bench_message msg = {
		.tag = DBM_TAG_STREAM_CLOSE,
		.stream_close = {.id = 0, .dropped = 0},
	};
	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

ZTEST(serial_protocol, test_stream_chunk_batch_encodes_to_the_pinned_wire_bytes)
{
	/* body: 0x03 tag, 0x02 id (raw u8), 0x01 records_len (varint),
	 * then one record: 0x89 0x06 rx_utc_ms=777 (varint), 0x04 bytes_len,
	 * "ok\r\n". Ten bytes, none of them zero. */
	static const uint8_t expected[] = {0x0b, 0x03, 0x02, 0x01, 0x89, 0x06,
					    0x04, 0x6f, 0x6b, 0x0d, 0x0a, 0x00};

	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_STREAM_CHUNK_BATCH;
	msg.stream_chunk_batch.id = 2;
	msg.stream_chunk_batch.records_len = 1;
	msg.stream_chunk_batch.records[0].rx_utc_ms = 777;
	memcpy(msg.stream_chunk_batch.records[0].bytes, "ok\r\n", 4);
	msg.stream_chunk_batch.records[0].bytes_len = 4;

	static uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

ZTEST(serial_protocol, test_stream_chunk_batch_refuses_more_records_than_it_can_hold)
{
	/* A disclosed capacity limit, not a silent truncation -- same posture
	 * `dbm_decode_frame` already takes for an oversized `steps_len`. */
	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_STREAM_CHUNK_BATCH;
	msg.stream_chunk_batch.records_len = DBM_MAX_STREAM_RECORDS_PER_BATCH + 1;

	static uint8_t frame[DBM_MAX_FRAME_LEN];

	zassert_true(dbm_encode_frame(&msg, frame, sizeof(frame)) < 0,
		      "an over-capacity batch must fail, not read past records[]");
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
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_BLE_ADVERTISE;
	study_start_msg.study_start.steps[0].action.advertise.has_local_name = true;
	strcpy(study_start_msg.study_start.steps[0].action.advertise.local_name,
	       "embarch-dev-bench");
	study_start_msg.study_start.steps[0].action.advertise.adv_interval_ms = 100;
	/* The real steps_crc embarch-study-designer's own steps_crc() computes
	 * for this exact single-step content (name "advertise",
	 * BleAdvertise{local_name: Some("embarch-dev-bench"), service_uuids: [],
	 * adv_interval_ms: 100}, timeout_ms 5000, continue_on_fail: false,
	 * delay_before_ms: 0) -- confirmed against the real crate, not guessed,
	 * so this test actually proves this file's CRC-32 matches that crate's,
	 * not just that this file agrees with itself. Changed with schema v6
	 * (Step gained delay_before_ms) and again with v9 (Step *lost*
	 * power_sample), since every step's encoding feeds the digest; the
	 * crate-side counterpart that keeps this honest is
	 * embarch-study-designer/tests/firmware_test_vectors.rs. */
	study_start_msg.study_start.steps_crc = 0x889FAF61;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.tag, DBM_TAG_STUDY_START, "wrong tag");
	zassert_equal(study_start_decoded.study_start.steps_len, 1, "steps_len mismatch");
	zassert_false(study_start_decoded.study_start.has_unsupported_action,
		      "should not flag an unsupported action");
	zassert_str_equal(study_start_decoded.study_start.steps[0].name, "advertise", "name mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].timeout_ms, 5000, "timeout_ms mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action_tag, DBM_ACTION_BLE_ADVERTISE,
		      "action_tag mismatch");
	zassert_true(study_start_decoded.study_start.steps[0].action.advertise.has_local_name,
		     "has_local_name mismatch");
	zassert_str_equal(study_start_decoded.study_start.steps[0].action.advertise.local_name,
			   "embarch-dev-bench", "local_name mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action.advertise.adv_interval_ms, 100,
		      "adv_interval_ms mismatch");
	zassert_equal(study_start_decoded.study_start.steps_crc, 0x889FAF61, "steps_crc mismatch");
	zassert_true(study_start_decoded.study_start.steps_crc_valid, "steps_crc_valid should be true");
	/* Schema v9's sibling seal (design.md §3 decision 39's 2026-08-25
	 * amendment). This encoder writes no taps, so the seal here is the CRC
	 * of nothing, which really is 0 -- see the encoder's own comment. What
	 * this asserts is only that the two seals are *independent*: an empty
	 * tap list validates while `steps` carries real content. The walker and
	 * the CRC over a non-empty span are covered by
	 * test_decodes_cores_study_start_with_real_taps below. */
	zassert_equal(study_start_decoded.study_start.streams_len, 0, "streams_len mismatch");
	zassert_true(study_start_decoded.study_start.streams_crc_valid,
		     "streams_crc_valid should be true for an empty tap list");

	/* Flipping a bit must be caught -- and must not disturb the other
	 * seal's verdict, which is the whole reason there are two. */
	study_start_msg.study_start.steps_crc ^= 1;
	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_false(study_start_decoded.study_start.steps_crc_valid,
		      "corrupted steps_crc should not validate");
	zassert_true(study_start_decoded.study_start.streams_crc_valid,
		     "a corrupted steps_crc must not implicate streams_crc");
}

ZTEST(serial_protocol, test_study_start_round_trip_two_steps)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 2;

	strcpy(study_start_msg.study_start.steps[0].name, "advertise-1");
	study_start_msg.study_start.steps[0].timeout_ms = 5000;
	study_start_msg.study_start.steps[0].continue_on_fail = false;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_BLE_ADVERTISE;
	study_start_msg.study_start.steps[0].action.advertise.has_local_name = true;
	strcpy(study_start_msg.study_start.steps[0].action.advertise.local_name, "dev-bench");
	study_start_msg.study_start.steps[0].action.advertise.adv_interval_ms = 100;

	strcpy(study_start_msg.study_start.steps[1].name, "advertise-2");
	study_start_msg.study_start.steps[1].timeout_ms = 2000;
	study_start_msg.study_start.steps[1].continue_on_fail = true;
	study_start_msg.study_start.steps[1].action_tag = DBM_ACTION_BLE_ADVERTISE;
	study_start_msg.study_start.steps[1].action.advertise.has_local_name = false;
	study_start_msg.study_start.steps[1].action.advertise.adv_interval_ms = 250;

	/* Real steps_crc for this exact two-step content, confirmed against
	 * embarch-study-designer's own steps_crc() -- see the one-step test's
	 * comment above. */
	study_start_msg.study_start.steps_crc = 0xC36612CC;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.study_start.steps_len, 2, "steps_len mismatch");
	zassert_str_equal(study_start_decoded.study_start.steps[1].name, "advertise-2", "name mismatch");
	zassert_false(study_start_decoded.study_start.steps[1].action.advertise.has_local_name,
		      "has_local_name mismatch");
	zassert_true(study_start_decoded.study_start.steps[1].continue_on_fail,
		     "continue_on_fail mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].delay_before_ms, 0,
		      "delay_before_ms mismatch");
	zassert_equal(study_start_decoded.study_start.steps[1].delay_before_ms, 0,
		      "delay_before_ms mismatch");
	zassert_true(study_start_decoded.study_start.steps_crc_valid, "steps_crc_valid should be true");
}

/* Schema v6 (embarch-study-designer/design.md §3 decision 42): Step's
 * delay_before_ms is a *trailing* varint, which is exactly the shape that
 * round-trips convincingly while actually being misaligned -- a decoder that
 * forgot to read it, or read it one field early, still produces plausible
 * output for an all-zero study. So the two steps here deliberately disagree
 * about it, and the values are multi-byte varints (>= 128) rather than small
 * ones, so a length error can't hide. steps_crc isn't asserted valid: these
 * values aren't in the crate-confirmed CRCs pinned above, and round_trip()
 * proving encode/decode agree is the whole point of this test.
 */
ZTEST(serial_protocol, test_study_start_round_trip_delay_before_ms)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 2;

	strcpy(study_start_msg.study_start.steps[0].name, "settle");
	study_start_msg.study_start.steps[0].timeout_ms = 5000;
	study_start_msg.study_start.steps[0].delay_before_ms = 2500;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_GATT_MONITOR_START;

	strcpy(study_start_msg.study_start.steps[1].name, "stimulate");
	study_start_msg.study_start.steps[1].timeout_ms = 3000;
	study_start_msg.study_start.steps[1].delay_before_ms = 128;
	study_start_msg.study_start.steps[1].continue_on_fail = true;
	study_start_msg.study_start.steps[1].action_tag = DBM_ACTION_GATT_MONITOR_STOP;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.study_start.steps_len, 2, "steps_len mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].delay_before_ms, 2500,
		      "step 0 delay_before_ms mismatch");
	zassert_equal(study_start_decoded.study_start.steps[1].delay_before_ms, 128,
		      "step 1 delay_before_ms mismatch");
	/* The fields either side of it must survive too -- the failure mode
	 * this guards against shifts everything after the misread. */
	zassert_equal(study_start_decoded.study_start.steps[0].timeout_ms, 5000,
		      "step 0 timeout_ms mismatch");
	zassert_str_equal(study_start_decoded.study_start.steps[1].name, "stimulate",
			   "step 1 name mismatch");
	zassert_true(study_start_decoded.study_start.steps[1].continue_on_fail,
		     "step 1 continue_on_fail mismatch");
	zassert_equal(study_start_decoded.study_start.steps[1].action_tag,
		      DBM_ACTION_GATT_MONITOR_STOP, "step 1 action_tag mismatch");
}

/* The real bytes embarch-core puts on the wire, decoded by this file's own
 * decoder. Every other StudyStart test here round-trips through
 * dbm_encode_frame first, which means a decoder bug that this file's encoder
 * mirrors exactly passes them all while failing against Core -- precisely
 * the failure mode that produced a silent, message-less study timeout on
 * real hardware the first time the stimulate-and-capture path ran
 * (receive_message() returns non-zero and main.c's loop just `continue`s,
 * so a rejected frame looks identical to a dead link).
 *
 * These are postcard bytes for DevBenchMessage::StudyStart carrying the
 * four-step study, produced by
 * embarch-study-designer/tests/firmware_test_vectors.rs's
 * dump_study_start_wire_bytes -- run that with --nocapture to regenerate
 * after any wire change (last regenerated for schema v9). This is the *payload*, pre-COBS, which is what
 * dbm_decode_frame takes.
 */
static const uint8_t core_study_start_frame[] = {
	/* 0x06: DevBenchMessage's StudyStart variant index, i.e. the message tag
	 * -- the first byte of the payload, not part of the COBS framing. */
	0x06, 0x04, 0x07, 0x63, 0x6f, 0x6e, 0x6e, 0x65, 0x63, 0x74, 0x01, 0x00, 0x00, 0x01,
	0x0f, 0x45, 0x69, 0x67, 0x68, 0x74, 0x20, 0x53, 0x6c, 0x65, 0x65, 0x70, 0x20, 0x53,
	0x31, 0x31, 0xa0, 0x9c, 0x01, 0x00, 0x00, 0x0c, 0x6f, 0x70, 0x65, 0x6e, 0x2d, 0x63,
	0x61, 0x70, 0x74, 0x75, 0x72, 0x65, 0x05, 0xa0, 0x9c, 0x01, 0x00, 0x00, 0x09, 0x73,
	0x74, 0x69, 0x6d, 0x75, 0x6c, 0x61, 0x74, 0x65, 0x02, 0x6e, 0x40, 0x00, 0x01, 0xb5,
	0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e, 0x6e, 0x40, 0x00,
	0x02, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e, 0x01,
	0x10, 0x6b, 0x65, 0x72, 0x6e, 0x65, 0x6c, 0x20, 0x76, 0x65, 0x72, 0x73, 0x69, 0x6f,
	0x6e, 0x0d, 0x0a, 0x88, 0x27, 0x00, 0xe8, 0x07, 0x0d, 0x63, 0x6c, 0x6f, 0x73, 0x65,
	0x2d, 0x63, 0x61, 0x70, 0x74, 0x75, 0x72, 0x65, 0x06, 0x88, 0x27, 0x00, 0xc0, 0x3e,
	0xe6, 0xb6, 0xe2, 0x95, 0x05,
	/* Schema v9's two trailing fields on StudyStart: `streams` -- an empty
	 * `Vec<StreamTap, _>`, one zero-length varint -- followed by
	 * `streams_crc` (design.md §3 decision 39's 2026-08-25 amendment).
	 *
	 * `streams_crc` is 0 here, and that is the *correct* value rather than
	 * an unset one: CRC-32/ISO-HDLC over zero bytes is 0, because its init
	 * and xorout are both 0xFFFFFFFF. A test whose only tap list is empty
	 * would therefore pass against a decoder that never computed the CRC at
	 * all -- which is why `core_study_start_with_taps_frame` below exists
	 * and carries three real taps. */
	0x00, 0x00,
};

/* An independent COBS encoder, deliberately not serial_protocol.c's own
 * (which is `static` anyway): the point of the test below is to feed the
 * decoder bytes it didn't produce, so borrowing the implementation's encoder
 * would reintroduce exactly the blind spot being closed. Standard COBS, no
 * trailing 0x00 delimiter -- dbm_decode_frame takes the frame without it. */
static size_t test_cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
	size_t code_index = 0;
	size_t write_index = 1;
	uint8_t code = 1;

	for (size_t read_index = 0; read_index < len; read_index++) {
		if (in[read_index] == 0) {
			out[code_index] = code;
			code = 1;
			code_index = write_index++;
		} else {
			out[write_index++] = in[read_index];
			if (++code == 0xFF) {
				out[code_index] = code;
				code = 1;
				code_index = write_index++;
			}
		}
	}
	out[code_index] = code;
	return write_index;
}

ZTEST(serial_protocol, test_decodes_cores_real_study_start_bytes)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];

	/* dbm_decode_frame expects a COBS-encoded frame, the same way
	 * receive_message() hands it one -- so the raw postcard payload above is
	 * COBS-encoded here first, exactly as Core's own transport does. */
	size_t framed_len = test_cobs_encode(core_study_start_frame,
					     sizeof(core_study_start_frame), framed);

	zassert_true(framed_len > 0, "COBS encode of Core's payload failed");

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "failed to decode the bytes embarch-core actually sends");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_equal(study_start_decoded.tag, DBM_TAG_STUDY_START, "wrong tag");
	zassert_equal(ss->steps_len, 4, "steps_len mismatch");
	zassert_false(ss->has_unsupported_action, "should recognize every action");
	zassert_true(ss->steps_crc_valid,
		     "steps_crc computed over Core's own bytes must validate");

	zassert_str_equal(ss->steps[0].name, "connect", "step 0 name");
	zassert_equal(ss->steps[0].action_tag, DBM_ACTION_BLE_CONNECT, "step 0 action");
	zassert_equal(ss->steps[0].timeout_ms, 20000, "step 0 timeout");
	zassert_equal(ss->steps[0].delay_before_ms, 0, "step 0 delay");
	/* Schema v7's trailing field on the BleConnect variant (design.md §3
	 * decision 43). It sits *before* the step's own timeout/delay on the
	 * wire, so getting its length wrong shifts everything after it -- which
	 * is exactly what the two assertions above would then catch. */
	zassert_true(ss->steps[0].action.connect.has_target_name, "step 0 has_target_name");
	zassert_str_equal(ss->steps[0].action.connect.target_name, "the client S11",
			   "step 0 target_name");

	zassert_str_equal(ss->steps[1].name, "open-capture", "step 1 name");
	zassert_equal(ss->steps[1].action_tag, DBM_ACTION_GATT_MONITOR_START, "step 1 action");

	zassert_str_equal(ss->steps[2].name, "stimulate", "step 2 name");
	zassert_equal(ss->steps[2].action_tag, DBM_ACTION_DATA_EXCHANGE, "step 2 action");
	zassert_equal(ss->steps[2].delay_before_ms, 1000, "step 2 delay");
	zassert_equal(ss->steps[2].action.data_exchange.operation.kind, DBM_GATT_OP_WRITE,
		      "step 2 operation");
	zassert_equal(ss->steps[2].action.data_exchange.operation.payload_len, 16,
		      "step 2 payload_len");
	zassert_mem_equal(ss->steps[2].action.data_exchange.operation.payload,
			  "kernel version\r\n", 16, "step 2 payload");
	/* NUS RX, from embarch-study-designer's vendor table (decision 41). */
	static const uint8_t nus_rx[16] = {0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93,
					   0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e};
	zassert_mem_equal(ss->steps[2].action.data_exchange.characteristic_uuid, nus_rx, 16,
			  "step 2 characteristic_uuid");

	zassert_str_equal(ss->steps[3].name, "close-capture", "step 3 name");
	zassert_equal(ss->steps[3].action_tag, DBM_ACTION_GATT_MONITOR_STOP, "step 3 action");
	zassert_equal(ss->steps[3].delay_before_ms, 8000, "step 3 delay");
}

/* The same shape as core_study_start_frame above, but carrying three real
 * stream taps -- because an empty `Vec<StreamTap>` proves nothing about
 * either half of schema v9's `streams_crc` check. CRC-32 over zero bytes is
 * genuinely 0, so a decoder that skipped the walk and the digest entirely
 * would pass every empty-tap test in this file.
 *
 * These bytes cannot come from this file's own encoder either: dev-bench
 * holds no taps, so `dbm_encode_frame` can only ever write an empty list.
 * They are produced by embarch-study-designer/tests/firmware_test_vectors.rs's
 * dump_study_start_with_taps_wire_bytes -- run that with --nocapture to
 * regenerate after any `StreamTap` change.
 *
 * The three taps deliberately span the variants whose *widths* differ, so a
 * walker that reads any one of them at the wrong size shifts where the
 * `streams` span ends and fails the CRC rather than passing plausibly:
 *
 *   - "waveform": a GattNotify source (two raw 16-byte UUIDs, no length
 *     prefix) + a Samples encoding (two enum varints and a raw u8
 *     channel_id) + a Steps scope (two varints).
 *   - "outpost": a Signal source (a length-prefixed name) + an OutpostTrace
 *     encoding (a five-byte varint manifest_crc) + a WholeStudy scope (no
 *     fields).
 *   - "power": a PowerFrontEnd source (a varint) + a Raw encoding (no
 *     fields) + a Steps scope.
 */
static const uint8_t core_study_start_with_taps_frame[] = {
	/* tag, then one BleAdvertise step, then steps_crc = 0x889FAF61 -- the
	 * same single step the one-step round-trip test above pins. */
	0x06, 0x01, 0x09, 0x61, 0x64, 0x76, 0x65, 0x72, 0x74, 0x69, 0x73, 0x65, 0x00, 0x01,
	0x11, 0x65, 0x6d, 0x62, 0x61, 0x72, 0x63, 0x68, 0x2d, 0x64, 0x65, 0x76, 0x2d, 0x62,
	0x65, 0x6e, 0x63, 0x68, 0x00, 0x64, 0x88, 0x27, 0x00, 0x00, 0xe1, 0xde, 0xfe, 0xc4,
	0x08,
	/* streams: 3 taps, then streams_crc = 0x7A66C565. */
	0x03, 0x00, 0x08, 0x77, 0x61, 0x76, 0x65, 0x66, 0x6f, 0x72, 0x6d, 0x00, 0x6e, 0x40,
	0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e,
	0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc,
	0xca, 0x9e, 0x02, 0x02, 0x03, 0x07, 0x01, 0x00, 0x00, 0x01, 0x07, 0x6f, 0x75, 0x74,
	0x70, 0x6f, 0x73, 0x74, 0x04, 0x0d, 0x6f, 0x75, 0x74, 0x70, 0x6f, 0x73, 0x74, 0x2d,
	0x74, 0x72, 0x61, 0x63, 0x65, 0x04, 0xef, 0xfd, 0xb6, 0xf5, 0x0d, 0x00, 0x02, 0x05,
	0x70, 0x6f, 0x77, 0x65, 0x72, 0x01, 0xe8, 0x07, 0x00, 0x01, 0x00, 0x00, 0xe5, 0x8a,
	0x9b, 0xd3, 0x07,
};

ZTEST(serial_protocol, test_decodes_cores_study_start_with_real_taps)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];

	size_t framed_len = test_cobs_encode(core_study_start_with_taps_frame,
					     sizeof(core_study_start_with_taps_frame), framed);

	zassert_true(framed_len > 0, "COBS encode of Core's payload failed");

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "failed to decode a StudyStart carrying real stream taps");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_equal(ss->steps_len, 1, "steps_len mismatch");
	zassert_true(ss->steps_crc_valid, "steps_crc should still validate");
	zassert_equal(ss->streams_len, 3, "streams_len mismatch");
	zassert_equal(ss->streams_crc, 0x7A66C565, "streams_crc mismatch");
	zassert_true(ss->streams_crc_valid,
		     "streams_crc computed over the crate's own tap bytes must validate");

	/* The taps are *stored* as of Milestone 7 Phase B item 3, not just
	 * walked past, so the fields this node acts on are pinned against the
	 * crate's own vector rather than only the span length being checked.
	 * This is where a reordered StreamSource/StreamScope variant would
	 * show up as a wrong tag instead of as plausible-looking garbage. */
	zassert_equal(ss->streams[0].id, 0, "tap 0 id");
	zassert_equal(ss->streams[0].source_tag, DBM_STREAM_SRC_GATT_NOTIFY, "tap 0 source");
	zassert_equal(ss->streams[0].scope_tag, DBM_STREAM_SCOPE_STEPS, "tap 0 scope");
	zassert_equal(ss->streams[0].scope_from, 0, "tap 0 scope.from");
	zassert_equal(ss->streams[0].scope_to, 0, "tap 0 scope.to");

	zassert_equal(ss->streams[1].id, 1, "tap 1 id");
	zassert_equal(ss->streams[1].source_tag, DBM_STREAM_SRC_SIGNAL, "tap 1 source");
	zassert_equal(ss->streams[1].scope_tag, DBM_STREAM_SCOPE_WHOLE_STUDY, "tap 1 scope");

	zassert_equal(ss->streams[2].id, 2, "tap 2 id");
	zassert_equal(ss->streams[2].source_tag, DBM_STREAM_SRC_POWER_FRONT_END, "tap 2 source");
	zassert_equal(ss->streams[2].scope_tag, DBM_STREAM_SCOPE_STEPS, "tap 2 scope");

	/* The Signal tap is Core's to open, not this node's -- dev-bench
	 * announcing it too would put two producers on one id. */
	zassert_true(dbm_stream_tap_is_ours(&ss->streams[0]), "a GattNotify tap is dev-bench's");
	zassert_false(dbm_stream_tap_is_ours(&ss->streams[1]), "a Signal tap is Core's");
	zassert_true(dbm_stream_tap_is_ours(&ss->streams[2]), "a PowerFrontEnd tap is dev-bench's");
}

ZTEST(serial_protocol, test_stream_scope_covers_the_inclusive_range_it_declares)
{
	/* Mirrors embarch-study-designer's own `scope_covers_the_inclusive_
	 * step_range_it_declares`. Both must agree or a window opens on a
	 * different step at each end of the link. */
	struct dbm_stream_tap whole = {.scope_tag = DBM_STREAM_SCOPE_WHOLE_STUDY};

	zassert_true(dbm_stream_tap_covers(&whole, 0), "WholeStudy covers the first step");
	zassert_true(dbm_stream_tap_covers(&whole, 0xFFFFFFFFu), "WholeStudy covers any step");

	struct dbm_stream_tap window = {
		.scope_tag = DBM_STREAM_SCOPE_STEPS,
		.scope_from = 1,
		.scope_to = 2,
	};

	zassert_false(dbm_stream_tap_covers(&window, 0), "before the window");
	zassert_true(dbm_stream_tap_covers(&window, 1), "at from");
	zassert_true(dbm_stream_tap_covers(&window, 2), "`to` is inclusive");
	zassert_false(dbm_stream_tap_covers(&window, 3), "past the window");

	/* One step past the last is what dispatch_study uses to close
	 * everything still open at the end of a run: no scope may cover it. */
	struct dbm_stream_tap single = {
		.scope_tag = DBM_STREAM_SCOPE_STEPS,
		.scope_from = 0,
		.scope_to = 0,
	};

	zassert_true(dbm_stream_tap_covers(&single, 0), "a single-step window is from == to");
	zassert_false(dbm_stream_tap_covers(&single, 1), "one past the last step closes it");
}

ZTEST(serial_protocol, test_a_corrupted_streams_crc_is_caught_without_implicating_steps)
{
	uint8_t payload[sizeof(core_study_start_with_taps_frame)];
	uint8_t framed[DBM_MAX_FRAME_LEN];

	memcpy(payload, core_study_start_with_taps_frame, sizeof(payload));
	/* Flip a bit inside the first tap's *name*, which is inside the span
	 * streams_crc covers but outside the span steps_crc does. */
	payload[46] ^= 0x01;

	size_t framed_len = test_cobs_encode(payload, sizeof(payload), framed);

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "a corrupted tap name should still decode structurally");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_false(ss->streams_crc_valid, "corrupted streams must not validate");
	zassert_true(ss->steps_crc_valid,
		     "a corrupted tap must not implicate steps_crc -- saying which half is "
		     "corrupt is why there are two seals rather than one widened one");
}

/* `StepResult`'s wire bytes, pinned across both languages for the first time
 * at schema v9. It was never covered before: decision 36's both-languages
 * rule applied to *new* records, and this one predates it -- so when
 * decision 39 (v8) retired `power_samples_ref`/`waveform_ref` from
 * `StepResult`, this file's encoder kept writing two `Option` bytes for them
 * and nothing noticed. Both suites stayed green because each agreed with
 * itself, which is the exact blind spot the pairing exists to close.
 *
 * embarch-study-designer pins the identical pre-COBS body in
 * `step_result_matches_dev_bench_firmwares_own_hand_written_encoding`.
 */
/* `static`, not a stack local: dev_bench_message's union embeds a full
 * StudyStart, the same reason study_start_msg above is static. */
static struct dev_bench_message pinned_step_result_msg;

ZTEST(serial_protocol, test_step_result_encodes_to_the_pinned_wire_bytes)
{
	static const uint8_t expected[] = {
		0x0d, 0x07, 0x01, 0x09, 0x61, 0x64, 0x76, 0x65, 0x72, 0x74, 0x69, 0x73,
		0x65, 0x07, 0x01, 0x04, 0xde, 0xad, 0xbe, 0xef, 0x01, 0x01, 0x00,
	};
	struct dbm_step_result_payload *r = &pinned_step_result_msg.step_result.result;

	memset(&pinned_step_result_msg, 0, sizeof(pinned_step_result_msg));
	pinned_step_result_msg.tag = DBM_TAG_STEP_RESULT;
	pinned_step_result_msg.step_result.step_index = 1;
	strcpy(r->step_name, "advertise");
	r->outcome.tag = 0; /* Pass */
	r->has_captured_data = true;
	r->captured_data[0] = 0xde;
	r->captured_data[1] = 0xad;
	r->captured_data[2] = 0xbe;
	r->captured_data[3] = 0xef;
	r->captured_data_len = 4;

	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&pinned_step_result_msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

/* design.md §3 decisions 31/32: every Action kind this crate defines must
 * round-trip through a StudyStart, not just BleAdvertise. These don't assert
 * steps_crc_valid -- the hardcoded steps_crc values above are real, crate-
 * confirmed CRCs for their own exact content, and computing a new one by
 * hand here isn't worth it just to decode-round-trip a single step's action
 * shape (round_trip() itself already proves encode/decode agree either
 * way). */
ZTEST(serial_protocol, test_study_start_ble_connect_action_round_trip)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 1;
	strcpy(study_start_msg.study_start.steps[0].name, "connect");
	study_start_msg.study_start.steps[0].timeout_ms = 5000;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_BLE_CONNECT;
	study_start_msg.study_start.steps[0].action.connect.role = 0; /* Central */
	study_start_msg.study_start.steps[0].action.connect.has_target_address = true;
	study_start_msg.study_start.steps[0].action.connect.target_address_kind = 1; /* Random */
	memcpy(study_start_msg.study_start.steps[0].action.connect.target_address,
	       (uint8_t[]){0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 6);

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.study_start.steps_len, 1, "steps_len mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action_tag, DBM_ACTION_BLE_CONNECT,
		      "action_tag mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action.connect.role, 0,
		      "role mismatch");
	zassert_true(study_start_decoded.study_start.steps[0].action.connect.has_target_address,
		     "has_target_address mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action.connect.target_address_kind, 1,
		      "target_address_kind mismatch");
	zassert_mem_equal(study_start_decoded.study_start.steps[0].action.connect.target_address,
			   ((uint8_t[]){0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}), 6,
			   "target_address mismatch");
}

ZTEST(serial_protocol, test_study_start_data_exchange_write_action_round_trip)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 1;
	strcpy(study_start_msg.study_start.steps[0].name, "write-cfg");
	study_start_msg.study_start.steps[0].timeout_ms = 2000;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_DATA_EXCHANGE;
	memset(study_start_msg.study_start.steps[0].action.data_exchange.service_uuid, 0x11, 16);
	memset(study_start_msg.study_start.steps[0].action.data_exchange.characteristic_uuid, 0x22,
	       16);
	study_start_msg.study_start.steps[0].action.data_exchange.operation.kind = DBM_GATT_OP_WRITE;
	study_start_msg.study_start.steps[0].action.data_exchange.operation.payload[0] = 0xDE;
	study_start_msg.study_start.steps[0].action.data_exchange.operation.payload[1] = 0xAD;
	study_start_msg.study_start.steps[0].action.data_exchange.operation.payload_len = 2;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	const struct dbm_data_exchange_action *de =
		&study_start_decoded.study_start.steps[0].action.data_exchange;

	zassert_equal(study_start_decoded.study_start.steps[0].action_tag, DBM_ACTION_DATA_EXCHANGE,
		      "action_tag mismatch");
	uint8_t expect_service[16], expect_char[16];

	memset(expect_service, 0x11, 16);
	memset(expect_char, 0x22, 16);
	zassert_mem_equal(de->service_uuid, expect_service, 16, "service_uuid mismatch");
	zassert_mem_equal(de->characteristic_uuid, expect_char, 16, "characteristic_uuid mismatch");
	zassert_equal(de->operation.kind, DBM_GATT_OP_WRITE, "operation kind mismatch");
	zassert_equal(de->operation.payload_len, 2, "payload_len mismatch");
	zassert_equal(de->operation.payload[0], 0xDE, "payload[0] mismatch");
	zassert_equal(de->operation.payload[1], 0xAD, "payload[1] mismatch");
}

ZTEST(serial_protocol, test_study_start_gatt_discover_and_monitor_all_round_trip)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 2;
	strcpy(study_start_msg.study_start.steps[0].name, "discover");
	study_start_msg.study_start.steps[0].timeout_ms = 3000;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_GATT_DISCOVER;
	strcpy(study_start_msg.study_start.steps[1].name, "monitor-all");
	study_start_msg.study_start.steps[1].timeout_ms = 10000;
	study_start_msg.study_start.steps[1].action_tag = DBM_ACTION_GATT_MONITOR_ALL;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.study_start.steps_len, 2, "steps_len mismatch");
	zassert_equal(study_start_decoded.study_start.steps[0].action_tag, DBM_ACTION_GATT_DISCOVER,
		      "step 0 action_tag mismatch");
	zassert_equal(study_start_decoded.study_start.steps[1].action_tag,
		      DBM_ACTION_GATT_MONITOR_ALL, "step 1 action_tag mismatch");
	zassert_false(study_start_decoded.study_start.has_unsupported_action,
		      "should not flag an unsupported action");
}

/* design.md §3 decisions 31/32: StepResult.gatt_services/gatt_activity, the
 * new fields GattDiscover/GattMonitorAll populate. */
static struct dev_bench_message gatt_step_result_msg;
static struct dev_bench_message gatt_step_result_decoded;

ZTEST(serial_protocol, test_step_result_gatt_services_round_trip)
{
	memset(&gatt_step_result_msg, 0, sizeof(gatt_step_result_msg));
	gatt_step_result_msg.tag = DBM_TAG_STEP_RESULT;
	gatt_step_result_msg.step_result.step_index = 1;
	strcpy(gatt_step_result_msg.step_result.result.step_name, "discover");
	gatt_step_result_msg.step_result.result.outcome.tag = 0; /* Pass */

	struct dbm_step_result_payload *r = &gatt_step_result_msg.step_result.result;

	r->has_gatt_services = true;
	r->gatt_services_len = 2;
	memset(r->gatt_services[0].uuid, 0x01, 16);
	r->gatt_services[0].characteristics_len = 1;
	memset(r->gatt_services[0].characteristics[0].uuid, 0x02, 16);
	r->gatt_services[0].characteristics[0].properties = 0x10; /* Notify */
	memset(r->gatt_services[1].uuid, 0x03, 16);
	r->gatt_services[1].characteristics_len = 0;

	zassert_equal(round_trip(&gatt_step_result_msg, &gatt_step_result_decoded), 0,
		      "decode failed");
	const struct dbm_step_result_payload *d = &gatt_step_result_decoded.step_result.result;

	zassert_true(d->has_gatt_services, "has_gatt_services mismatch");
	zassert_equal(d->gatt_services_len, 2, "gatt_services_len mismatch");
	zassert_equal(d->gatt_services[0].characteristics_len, 1,
		      "service 0 characteristics_len mismatch");
	zassert_equal(d->gatt_services[0].characteristics[0].properties, 0x10,
		      "service 0 characteristic 0 properties mismatch");
	zassert_equal(d->gatt_services[1].characteristics_len, 0,
		      "service 1 characteristics_len mismatch");
	zassert_false(d->has_gatt_activity, "has_gatt_activity should default false");
}

ZTEST(serial_protocol, test_step_result_gatt_activity_round_trip)
{
	memset(&gatt_step_result_msg, 0, sizeof(gatt_step_result_msg));
	gatt_step_result_msg.tag = DBM_TAG_STEP_RESULT;
	gatt_step_result_msg.step_result.step_index = 2;
	strcpy(gatt_step_result_msg.step_result.result.step_name, "monitor-all");
	gatt_step_result_msg.step_result.result.outcome.tag = 0; /* Pass */

	struct dbm_step_result_payload *r = &gatt_step_result_msg.step_result.result;

	r->has_gatt_activity = true;
	r->gatt_activity_len = 2;
	r->gatt_activity[0].rx_utc_ms = 1753000000123ULL;
	r->gatt_activity[0].characteristic_index = 3;
	r->gatt_activity[0].payload[0] = 0x01;
	r->gatt_activity[0].payload[1] = 0x02;
	r->gatt_activity[0].payload_len = 2;
	r->gatt_activity[1].rx_utc_ms = 1753000000456ULL;
	r->gatt_activity[1].characteristic_index = 7;
	r->gatt_activity[1].payload_len = 0;

	zassert_equal(round_trip(&gatt_step_result_msg, &gatt_step_result_decoded), 0,
		      "decode failed");
	const struct dbm_step_result_payload *d = &gatt_step_result_decoded.step_result.result;

	zassert_true(d->has_gatt_activity, "has_gatt_activity mismatch");
	zassert_equal(d->gatt_activity_len, 2, "gatt_activity_len mismatch");
	zassert_equal(d->gatt_activity[0].rx_utc_ms, 1753000000123ULL, "record 0 rx_utc_ms mismatch");
	zassert_equal(d->gatt_activity[0].characteristic_index, 3,
		      "record 0 characteristic_index mismatch");
	zassert_equal(d->gatt_activity[0].payload_len, 2, "record 0 payload_len mismatch");
	zassert_equal(d->gatt_activity[0].payload[1], 0x02, "record 0 payload[1] mismatch");
	zassert_equal(d->gatt_activity[1].characteristic_index, 7,
		      "record 1 characteristic_index mismatch");
	zassert_false(d->has_gatt_services, "has_gatt_services should stay false");
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

/* ---- GATT transcript (embarch-dev-bench/design.md §3 decision 36) -------- */

/* The exact bytes `dbm_encode_transcript_entry` must produce for one
 * `GattTranscriptEntry`.
 *
 * These used to be pinned as a whole DBM_TAG_GATT_TRANSCRIPT_RECORD frame.
 * That message is retired at schema v8 (embarch-study-designer/design.md §3
 * decision 39): the entry itself, its both-directions coverage, its uncapped
 * streaming and its `gatt.csv` columns are all unchanged, but it now rides as
 * the byte payload of a DBM_TAG_STREAM_CHUNK_BATCH record on a tap declared
 * `StreamEncoding::GattTranscript`. So what is pinned is the entry, which is
 * what actually crosses the wire; the bytes below are byte-for-byte the entry
 * half of the frame decision 36 originally pinned.
 *
 * embarch-study-designer pins the identical bytes in
 * `gatt_transcript_entry_matches_dev_bench_firmwares_own_hand_written_encoding`
 * and asserts postcard decodes and re-encodes them. Changing the entry's
 * shape must break both.
 */
static const uint8_t expected_transcript_entry[] = {
	0x89, 0x06, 0x01, 0x0b, 0x01, 0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3,
	0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e, 0x01, 0x6e, 0x40,
	0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc,
	0xca, 0x9e, 0x00, 0x04, 0x6f, 0x6b, 0x0d, 0x0a,
};

ZTEST(serial_protocol, test_gatt_transcript_entry_encodes_to_the_pinned_wire_bytes)
{
	static const uint8_t nus_service[16] = {0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93,
						0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e};
	static const uint8_t nus_tx[16] = {0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93,
					   0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e};

	static struct dbm_gatt_transcript_entry entry;

	memset(&entry, 0, sizeof(entry));
	entry.rx_utc_ms = 777;
	entry.direction = DBM_GATT_DIR_IN;
	entry.kind = DBM_GATT_EVT_NOTIFICATION;
	entry.has_service_uuid = true;
	memcpy(entry.service_uuid, nus_service, 16);
	entry.has_characteristic_uuid = true;
	memcpy(entry.characteristic_uuid, nus_tx, 16);
	entry.att_status = 0;
	memcpy(entry.payload, "ok\r\n", 4);
	entry.payload_len = 4;

	static uint8_t bytes[DBM_MAX_TRANSCRIPT_RECORD_LEN];
	int len = dbm_encode_transcript_entry(&entry, bytes, sizeof(bytes));

	zassert_equal(len, (int)sizeof(expected_transcript_entry), "entry length mismatch");
	zassert_mem_equal(bytes, expected_transcript_entry, sizeof(expected_transcript_entry),
			   "encoded entry differs from the pinned wire bytes");
}

ZTEST(serial_protocol, test_a_transcript_entry_rides_a_stream_record_unchanged)
{
	/* The whole message dev-bench will send for one transcript line once
	 * Phase B rewires main.c onto a tap: the pinned entry above, verbatim,
	 * as a generic record's payload. Pinned here as the frame; pinned on
	 * the Rust side, pre-COBS, in
	 * `a_transcript_entry_carried_as_a_stream_record_matches_the_pinned_frame`. */
	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_STREAM_CHUNK_BATCH;
	msg.stream_chunk_batch.id = 2;
	msg.stream_chunk_batch.records_len = 1;
	msg.stream_chunk_batch.records[0].rx_utc_ms = 777;
	memcpy(msg.stream_chunk_batch.records[0].bytes, expected_transcript_entry,
	       sizeof(expected_transcript_entry));
	msg.stream_chunk_batch.records[0].bytes_len = sizeof(expected_transcript_entry);

	static uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&msg, frame, sizeof(frame));

	/* body = tag, id, records_len, rx_utc_ms varint (2), bytes_len (1),
	 * then the 44-byte entry = 50 bytes; COBS adds a leading code byte and
	 * a trailing delimiter, and relocates the entry's own zero bytes. */
	zassert_equal(frame_len, 50 + 2, "frame length mismatch");
	zassert_equal(frame[frame_len - 1], 0x00, "frame missing trailing delimiter");
}

ZTEST(serial_protocol, test_gatt_transcript_entry_with_no_uuids_encodes_to_the_pinned_bytes)
{
	/* A connect/disconnect/discovery-started entry carries neither UUID --
	 * both Options must encode as a bare 0 discriminant with no bytes
	 * following, or every field after them shifts. */
	static struct dbm_gatt_transcript_entry entry;

	memset(&entry, 0, sizeof(entry));
	entry.rx_utc_ms = 5;
	entry.direction = DBM_GATT_DIR_LOCAL;
	entry.kind = DBM_GATT_EVT_DISCOVERY_STARTED;
	entry.att_status = 0;
	entry.payload_len = 0;

	/* rx_utc_ms, direction, kind, None, None, att_status, payload_len --
	 * seven single bytes. */
	static const uint8_t expected[] = {0x05, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00};

	static uint8_t bytes[DBM_MAX_TRANSCRIPT_RECORD_LEN];
	int len = dbm_encode_transcript_entry(&entry, bytes, sizeof(bytes));

	zassert_equal(len, (int)sizeof(expected), "entry length mismatch");
	zassert_mem_equal(bytes, expected, sizeof(expected), "encoded entry mismatch");
}

ZTEST(serial_protocol, test_study_start_gatt_monitor_start_and_stop_round_trip)
{
	/* design.md §3 decision 36's two new Action tags must decode as
	 * field-less, and must not trip `has_unsupported_action` -- the check
	 * that would otherwise make a whole Study abort before running. */
	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_STUDY_START;
	msg.study_start.steps_len = 2;
	strcpy(msg.study_start.steps[0].name, "open-window");
	msg.study_start.steps[0].timeout_ms = 10000;
	msg.study_start.steps[0].action_tag = DBM_ACTION_GATT_MONITOR_START;
	strcpy(msg.study_start.steps[1].name, "close-window");
	msg.study_start.steps[1].timeout_ms = 5000;
	msg.study_start.steps[1].action_tag = DBM_ACTION_GATT_MONITOR_STOP;

	static struct dev_bench_message decoded;

	zassert_equal(round_trip(&msg, &decoded), 0, "decode failed");
	zassert_false(decoded.study_start.has_unsupported_action,
		       "monitor start/stop reported as unsupported");
	zassert_equal(decoded.study_start.steps_len, 2, "steps_len mismatch");
	zassert_equal(decoded.study_start.steps[0].action_tag, DBM_ACTION_GATT_MONITOR_START,
		       "step 0 action tag mismatch");
	zassert_equal(decoded.study_start.steps[1].action_tag, DBM_ACTION_GATT_MONITOR_STOP,
		       "step 1 action tag mismatch");
	zassert_str_equal(decoded.study_start.steps[0].name, "open-window", "step 0 name mismatch");
	zassert_equal(decoded.study_start.steps[1].timeout_ms, 5000, "step 1 timeout mismatch");
}
