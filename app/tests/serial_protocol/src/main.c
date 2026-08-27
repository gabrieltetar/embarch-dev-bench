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

#include "eap_interp.h"
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
 * after any wire change (last regenerated for schema v13). This is the *payload*, pre-COBS, which is what
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
	/* dev_bench_log_level = DevBenchLogLevel::Debug (4) -- schema v13,
	 * design.md §3 decision 39. Deliberately not the default (Warn = 2):
	 * the vector generator picks a value the C struct would not contain by
	 * accident, so an off-by-one in walking the streams_crc that precedes
	 * it cannot pass. */
	0x04,
	/* protocols + protocols_crc -- schema v15
	 * (embarch-study-designer/design.md §3 decision 58), an empty list and,
	 * correspondingly, the CRC of nothing. Both zeros are *correct* values
	 * rather than unset ones, the same way the empty-`streams` pair is:
	 * CRC-32/ISO-HDLC over zero bytes is 0. A decoder that stopped at
	 * `dev_bench_log_level` would still pass every assertion in this test,
	 * which is why `core_study_start_with_protocol_frame` below carries a
	 * real manifest and a real seal. */
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

	/* Schema v13 (design.md §3 decision 39): the level the study asked for,
	 * decoded from bytes this firmware did not produce. The generator picks
	 * `Debug` rather than the `Warn` default precisely so a decoder that
	 * ignored this trailing byte -- which would still decode the frame and
	 * still pass every other assertion here -- fails this one. */
	zassert_equal(ss->dev_bench_log_level, DBM_LOG_LEVEL_DBG,
		      "dev_bench_log_level mismatch (got %u)",
		      (unsigned int)ss->dev_bench_log_level);

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
 *     encoding (no fields as of schema v11, where `manifest_crc` was removed
 *     -- it encoded a mechanism the firmware cannot produce; see that
 *     variant's own doc comment) + a WholeStudy scope (no fields).
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
	/* streams: 3 taps, then streams_crc = 0x91D195D2. */
	0x03, 0x00, 0x08, 0x77, 0x61, 0x76, 0x65, 0x66, 0x6f, 0x72, 0x6d, 0x00, 0x6e, 0x40,
	0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e,
	0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc,
	0xca, 0x9e, 0x02, 0x02, 0x03, 0x07, 0x01, 0x00, 0x00, 0x01, 0x07, 0x6f, 0x75, 0x74,
	0x70, 0x6f, 0x73, 0x74, 0x04, 0x0d, 0x6f, 0x75, 0x74, 0x70, 0x6f, 0x73, 0x74, 0x2d,
	0x74, 0x72, 0x61, 0x63, 0x65, 0x04, 0x00, 0x02, 0x05, 0x70, 0x6f, 0x77, 0x65, 0x72,
	0x01, 0xe8, 0x07, 0x00, 0x01, 0x00, 0x00, 0xd2, 0xab, 0xc6, 0x8e, 0x09,
	/* dev_bench_log_level = DevBenchLogLevel::Debug (4) -- schema v13,
	 * design.md §3 decision 39. Deliberately not the default (Warn = 2):
	 * the vector generator picks a value the C struct would not contain by
	 * accident, so an off-by-one in walking the streams_crc that precedes
	 * it cannot pass. */
	0x04,
	/* protocols + protocols_crc -- schema v15
	 * (embarch-study-designer/design.md §3 decision 58), an empty list and,
	 * correspondingly, the CRC of nothing. Both zeros are *correct* values
	 * rather than unset ones, the same way the empty-`streams` pair is:
	 * CRC-32/ISO-HDLC over zero bytes is 0. A decoder that stopped at
	 * `dev_bench_log_level` would still pass every assertion in this test,
	 * which is why `core_study_start_with_protocol_frame` below carries a
	 * real manifest and a real seal. */
	0x00, 0x00,
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
	zassert_equal(ss->streams_crc, 0x91D195D2, "streams_crc mismatch");
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

	/* Schema v13 (design.md §3 decision 39): the level the study asked for,
	 * decoded from bytes this firmware did not produce. The generator picks
	 * `Debug` rather than the `Warn` default precisely so a decoder that
	 * ignored this trailing byte -- which would still decode the frame and
	 * still pass every other assertion here -- fails this one. */
	zassert_equal(ss->dev_bench_log_level, DBM_LOG_LEVEL_DBG,
		      "dev_bench_log_level mismatch (got %u)",
		      (unsigned int)ss->dev_bench_log_level);

}

/* Schema v14's own cross-language pin (embarch-study-designer/design.md §3
 * decisions 52/53): a `StudyStart` whose first step is a
 * `GattMonitorSelectedStart` carrying two targets, whose second is a
 * field-less `GattMonitorStop`, and whose one tap is a `GattNotify` source
 * with a `Struct` encoding.
 *
 * Produced by embarch-study-designer/tests/firmware_test_vectors.rs's
 * dump_study_start_with_selective_monitor_wire_bytes -- run it with
 * --nocapture to regenerate. Three things here are unwalkable by a decoder
 * that predates v14, and each shifts everything after it:
 *
 *   - the target *sequence*: every monitor action before v14 was field-less,
 *     so a decoder treating this one the same way reads the target-count
 *     varint as the next step's name length and decodes a step list that
 *     still parses, into nonsense. The `GattMonitorStop` step after it is
 *     what catches that.
 *   - `StreamEncoding::Struct`'s one raw u8, inside the span `streams_crc`
 *     seals. Skipped at the wrong width, the CRC check fails -- loudly, at
 *     the handshake, rather than as a study that runs and captures into the
 *     wrong file. The decoder value is deliberately 1, not 0, so a decoder
 *     that skipped the byte entirely still shifts the span.
 *   - the kept `characteristic_uuid` on a GattNotify tap (decision 55), which
 *     nothing kept before v14 and which is what routes a notification to this
 *     tap's id at all.
 */
static const uint8_t core_study_start_selective_monitor_frame[] = {
	0x06, 0x02, 0x07, 0x6d, 0x6f, 0x6e, 0x69, 0x74, 0x6f, 0x72, 0x0a, 0x02,
	0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e,
	0x24, 0xdc, 0xca, 0x9e, 0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93,
	0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e, 0x6e, 0x40, 0x00, 0x01,
	0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e,
	0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e,
	0x24, 0xdc, 0xca, 0x9e, 0x88, 0x27, 0x00, 0x00, 0x04, 0x73, 0x74, 0x6f,
	0x70, 0x06, 0xe8, 0x07, 0x00, 0xfa, 0x01, 0xca, 0xda, 0xc4, 0xa6, 0x0c,
	0x01, 0x00, 0x06, 0x6e, 0x75, 0x73, 0x2d, 0x74, 0x78, 0x00, 0x6e, 0x40,
	0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc,
	0xca, 0x9e, 0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9,
	0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e, 0x05, 0x01, 0x00, 0xa7, 0xaf, 0xc4,
	0xec, 0x0d, 0x04,
	/* protocols + protocols_crc -- schema v15
	 * (embarch-study-designer/design.md §3 decision 58), an empty list and,
	 * correspondingly, the CRC of nothing. Both zeros are *correct* values
	 * rather than unset ones, the same way the empty-`streams` pair is:
	 * CRC-32/ISO-HDLC over zero bytes is 0. A decoder that stopped at
	 * `dev_bench_log_level` would still pass every assertion in this test,
	 * which is why `core_study_start_with_protocol_frame` below carries a
	 * real manifest and a real seal. */
	0x00, 0x00,
};

ZTEST(serial_protocol, test_decodes_cores_selective_monitor_study_start)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];

	size_t framed_len = test_cobs_encode(core_study_start_selective_monitor_frame,
					     sizeof(core_study_start_selective_monitor_frame),
					     framed);

	zassert_true(framed_len > 0, "COBS encode of Core's payload failed");

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "failed to decode a StudyStart carrying a selective monitor step");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_equal(ss->steps_len, 2, "steps_len mismatch");
	zassert_false(ss->has_unsupported_action, "v14's actions must be recognized");
	zassert_true(ss->steps_crc_valid,
		     "steps_crc computed over the crate's own bytes must validate -- a target "
		     "list walked at the wrong width is exactly what this catches");

	zassert_equal(ss->steps[0].action_tag, DBM_ACTION_GATT_MONITOR_SELECTED_START,
		      "step 0 action_tag mismatch");
	zassert_equal(ss->steps[0].action.monitor_selected.targets_len, 2,
		      "step 0 targets_len mismatch");
	/* Nordic UART Service: 6e400001-... service, 6e400003-... TX and
	 * 6e400002-... RX. Asserted on the bytes that differ between the two,
	 * so a decoder that read one target twice fails here. */
	zassert_equal(ss->steps[0].action.monitor_selected.targets[0].characteristic_uuid[3], 0x03,
		      "target 0 characteristic mismatch");
	zassert_equal(ss->steps[0].action.monitor_selected.targets[1].characteristic_uuid[3], 0x02,
		      "target 1 characteristic mismatch");
	zassert_equal(ss->steps[0].action.monitor_selected.targets[0].service_uuid[3], 0x01,
		      "target 0 service mismatch");

	/* The step after the variable-length one lands where the encoder put
	 * it -- the same role BleUnbond plays in the v12 vector. */
	zassert_equal(ss->steps[1].action_tag, DBM_ACTION_GATT_MONITOR_STOP,
		      "step 1 action_tag mismatch");
	zassert_equal(ss->steps[1].delay_before_ms, 250, "step 1 delay_before_ms mismatch");

	zassert_equal(ss->streams_len, 1, "streams_len mismatch");
	zassert_true(ss->streams_crc_valid,
		     "streams_crc must validate -- a Struct encoding walked at the wrong width "
		     "is what this catches");
	zassert_equal(ss->streams[0].source_tag, DBM_STREAM_SRC_GATT_NOTIFY, "tap 0 source");
	/* Decision 55: the characteristic is *kept*, not skipped, because this
	 * node routes notifications to this tap's id and a notification
	 * identifies itself by characteristic. */
	zassert_equal(ss->streams[0].characteristic_uuid[3], 0x03,
		      "tap 0 must keep the characteristic it routes");
	zassert_equal(ss->streams[0].characteristic_uuid[0], 0x6e, "tap 0 characteristic[0]");
	zassert_equal(ss->streams[0].scope_tag, DBM_STREAM_SCOPE_WHOLE_STUDY, "tap 0 scope");
}

ZTEST(serial_protocol, test_a_non_gatt_notify_tap_keeps_no_routing_uuid)
{
	/* A stale UUID left over from a previous study's tap at this index
	 * would route a notification into a tap that isn't a GattNotify one at
	 * all -- so the decoder clears it per tap rather than per study. The
	 * three-tap vector's Signal and PowerFrontEnd taps are the check. */
	uint8_t framed[DBM_MAX_FRAME_LEN];
	size_t framed_len = test_cobs_encode(core_study_start_with_taps_frame,
					     sizeof(core_study_start_with_taps_frame), framed);

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	/* Pre-poison every byte the decoder is supposed to overwrite. */
	memset(study_start_decoded.study_start.streams, 0xAA,
	       sizeof(study_start_decoded.study_start.streams));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0, "decode failed");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;
	static const uint8_t zeros[16] = {0};

	zassert_equal(ss->streams[0].source_tag, DBM_STREAM_SRC_GATT_NOTIFY, "tap 0 source");
	zassert_equal(ss->streams[0].characteristic_uuid[3], 0x03,
		      "a GattNotify tap keeps its characteristic");
	zassert_mem_equal(ss->streams[1].characteristic_uuid, zeros, 16,
			  "a Signal tap must carry no routing UUID");
	zassert_mem_equal(ss->streams[2].characteristic_uuid, zeros, 16,
			  "a PowerFrontEnd tap must carry no routing UUID");
}

ZTEST(serial_protocol, test_selective_monitor_round_trips_through_this_encoder)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 1;
	strcpy(study_start_msg.study_start.steps[0].name, "monitor");
	study_start_msg.study_start.steps[0].timeout_ms = 5000;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_GATT_MONITOR_SELECTED;
	study_start_msg.study_start.steps[0].action.monitor_selected.targets_len = 1;
	memset(study_start_msg.study_start.steps[0].action.monitor_selected.targets[0].service_uuid,
	       0x11, 16);
	memset(study_start_msg.study_start.steps[0]
		       .action.monitor_selected.targets[0]
		       .characteristic_uuid,
	       0x22, 16);

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");

	const struct dbm_step *step = &study_start_decoded.study_start.steps[0];

	zassert_equal(step->action_tag, DBM_ACTION_GATT_MONITOR_SELECTED, "action_tag mismatch");
	zassert_equal(step->action.monitor_selected.targets_len, 1, "targets_len mismatch");
	zassert_equal(step->action.monitor_selected.targets[0].service_uuid[0], 0x11,
		      "service_uuid mismatch");
	zassert_equal(step->action.monitor_selected.targets[0].characteristic_uuid[15], 0x22,
		      "characteristic_uuid mismatch");
}

ZTEST(serial_protocol, test_selective_monitor_beyond_the_target_cap_is_refused)
{
	/* A truncated subscription list would be a study that silently
	 * monitored a subset of what it named -- the silently-empty capture
	 * this whole family of decisions keeps being opened by. */
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 1;
	strcpy(study_start_msg.study_start.steps[0].name, "monitor");
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_GATT_MONITOR_SELECTED;
	study_start_msg.study_start.steps[0].action.monitor_selected.targets_len =
		DBM_MAX_MONITOR_TARGETS + 1;

	uint8_t frame[DBM_MAX_FRAME_LEN];

	zassert_true(dbm_encode_frame(&study_start_msg, frame, sizeof(frame)) < 0,
		     "targets_len beyond the array's bound must be rejected, not read OOB");
}

ZTEST(serial_protocol, test_stream_scope_covers_the_inclusive_range_it_declares)
{
	/* Mirrors embarch-study-designer's own `scope_covers_the_inclusive_
	 * step_range_it_declares`. Both must agree or a window opens on a
	 * different step at each end of the link. */
	struct dbm_stream_tap whole = {.scope_tag = DBM_STREAM_SCOPE_WHOLE_STUDY};

	zassert_true(dbm_stream_tap_covers(&whole, 0), "WholeStudy covers the first step");
	zassert_true(dbm_stream_tap_covers(&whole, 0xFFFFFFFFu), "WholeStudy covers any step");

	/* And *because* it covers any step, no step index closes it. This
	 * assertion is the reason `dispatch_study` ends a run with
	 * `close_all_taps()` rather than with a step index past the last one:
	 * that trick closes a `Steps` window and silently leaves every
	 * `WholeStudy` tap open, so its `StreamClose` -- and with it the
	 * `dropped` count Core turns into `StreamRef.truncated` -- was never
	 * sent. Measured on the bench: a `gatt` tap widened to `WholeStudy`
	 * reported `truncated: false` in a run whose own log line said 31
	 * entries had been dropped. */
	zassert_true(dbm_stream_tap_covers(&whole, 14),
		     "one past the last step does NOT close a WholeStudy tap -- close_all_taps does");

	struct dbm_stream_tap window = {
		.scope_tag = DBM_STREAM_SCOPE_STEPS,
		.scope_from = 1,
		.scope_to = 2,
	};

	zassert_false(dbm_stream_tap_covers(&window, 0), "before the window");
	zassert_true(dbm_stream_tap_covers(&window, 1), "at from");
	zassert_true(dbm_stream_tap_covers(&window, 2), "`to` is inclusive");
	zassert_false(dbm_stream_tap_covers(&window, 3), "past the window");

	/* One step past the last is what a `Steps` window's own scope stops
	 * covering -- which is all this predicate promises. Closing everything
	 * at the end of a run is `close_all_taps`'s job, not a step index's. */
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
		0x65, 0x07, 0x01, 0x04, 0xde, 0xad, 0xbe, 0xef, 0x01,
		/* Schema v12's trailing `security_level: Option<SecurityLevel>`
		 * (embarch-study-designer/design.md §3 decision 50), None here --
		 * one more COBS zero-run code byte. The populated case is pinned
		 * separately below; an all-None frame would pass against an
		 * encoder that wrote the Option byte but not the value.
		 *
		 * **One 0x01 shorter than it was at v13.** `gatt_activity`'s
		 * `None` byte sat between `gatt_services` and `security_level`
		 * and is retired at schema v14
		 * (embarch-study-designer/design.md §3 decision 54). An encoder
		 * that kept writing it would put `security_level` one byte late
		 * and Core would read the *activity* Option byte as the security
		 * level -- the same class of drift the retired
		 * `power_samples_ref`/`waveform_ref` bytes caused for a whole
		 * schema version, which is why this vector exists. */
		0x01,
		/* Schema v15's own trailing `protocol: Option<ProtocolOutcome>`
		 * (embarch-study-designer/design.md §3 decision 62), None here
		 * -- and None on every step but a `RunProtocol` one, which is
		 * every step this firmware has produced to date. One more COBS
		 * zero-run code byte, which is exactly what "appended, not
		 * inserted" is supposed to look like on the message this
		 * firmware sends most. The populated case is pinned separately
		 * below. */
		0x01,
		0x00,
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

/* design.md §3 decisions 31/32: StepResult.gatt_services, the field every
 * discovering action populates. */
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
}

/* `test_step_result_gatt_activity_round_trip` was here. Retired with the
 * field at schema v14 (embarch-study-designer/design.md §3 decision 54): a
 * capped in-memory copy of a capture the tap pipeline already streams to Core
 * uncapped. What a monitor step captured is now covered by the GattNotify tap
 * routing tests instead -- decision 55's own half.
 */

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


/* ---- schema v12: security (embarch-study-designer/design.md §3 decisions
 * 50/51) ------------------------------------------------------------------ */

/* The populated half of `StepResult.security_level`. The None case above
 * would pass against an encoder that wrote the Option byte and forgot the
 * value, which is a real shape of this exact bug -- decision 39's retired
 * refs survived a whole schema version as two bytes nothing read.
 *
 * Pre-COBS body pinned by embarch-study-designer's
 * dump_step_result_with_security_wire_bytes; this is that body COBS-encoded
 * with dbm_encode_frame's own trailing delimiter, which is what the wire
 * carries. */
ZTEST(serial_protocol, test_step_result_with_security_level_encodes_to_the_pinned_wire_bytes)
{
	static const uint8_t expected[] = {
		0x0a, 0x07, 0x01, 0x06, 0x73, 0x65, 0x63, 0x75, 0x72, 0x65,
		/* One 0x01 shorter than at v13, for the same reason the
		 * all-None vector above is: `gatt_activity`'s retired `None`
		 * byte sat immediately before `security_level`, so an encoder
		 * that kept writing it would put the level one byte late --
		 * and this is the vector where that shows up as a wrong
		 * *value* rather than only a wrong length. */
		0x01, 0x01, 0x03, 0x01, 0x03,
		/* Schema v15's trailing `protocol` Option, None. */
		0x01,
		0x00,
	};
	struct dbm_step_result_payload *r = &pinned_step_result_msg.step_result.result;

	memset(&pinned_step_result_msg, 0, sizeof(pinned_step_result_msg));
	pinned_step_result_msg.tag = DBM_TAG_STEP_RESULT;
	pinned_step_result_msg.step_result.step_index = 1;
	strcpy(r->step_name, "secure");
	r->outcome.tag = 0; /* Pass */
	r->has_security_level = true;
	r->security_level = DBM_SECURITY_L4;

	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&pinned_step_result_msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

ZTEST(serial_protocol, test_step_result_security_level_round_trips)
{
	struct dbm_step_result_payload *r = &pinned_step_result_msg.step_result.result;

	memset(&pinned_step_result_msg, 0, sizeof(pinned_step_result_msg));
	pinned_step_result_msg.tag = DBM_TAG_STEP_RESULT;
	pinned_step_result_msg.step_result.step_index = 3;
	strcpy(r->step_name, "discover");
	r->outcome.tag = 0;
	r->has_security_level = true;
	r->security_level = DBM_SECURITY_L1;

	zassert_equal(round_trip(&pinned_step_result_msg, &study_start_decoded), 0,
		      "decode failed");
	zassert_true(study_start_decoded.step_result.result.has_security_level,
		     "has_security_level lost");
	/* L1 specifically: it is the level no study may *request* and the one a
	 * study debugging a security-requiring DUT most needs reported, so it
	 * has to survive the wire like any other. */
	zassert_equal(study_start_decoded.step_result.result.security_level, DBM_SECURITY_L1,
		      "security_level mismatch");
}

ZTEST(serial_protocol, test_study_start_security_actions_round_trip)
{
	memset(&study_start_msg, 0, sizeof(study_start_msg));
	study_start_msg.tag = DBM_TAG_STUDY_START;
	study_start_msg.study_start.steps_len = 2;
	strcpy(study_start_msg.study_start.steps[0].name, "secure");
	study_start_msg.study_start.steps[0].timeout_ms = 10000;
	study_start_msg.study_start.steps[0].action_tag = DBM_ACTION_BLE_SECURITY;
	study_start_msg.study_start.steps[0].action.set_security.level = DBM_SECURITY_L4;
	strcpy(study_start_msg.study_start.steps[1].name, "drop-bond");
	study_start_msg.study_start.steps[1].timeout_ms = 5000;
	study_start_msg.study_start.steps[1].action_tag = DBM_ACTION_BLE_UNBOND;

	zassert_equal(round_trip(&study_start_msg, &study_start_decoded), 0, "decode failed");
	zassert_equal(study_start_decoded.study_start.steps_len, 2, "steps_len mismatch");
	zassert_false(study_start_decoded.study_start.has_unsupported_action,
		      "both v12 actions must be recognized");
	zassert_equal(study_start_decoded.study_start.steps[0].action_tag,
		      DBM_ACTION_BLE_SECURITY, "step 0 action_tag");
	zassert_equal(study_start_decoded.study_start.steps[0].action.set_security.level,
		      DBM_SECURITY_L4, "step 0 level");
	/* The field-less variant *after* the field-carrying one: if the level
	 * varint were walked at the wrong width, this tag lands somewhere else
	 * and this is what says so. */
	zassert_equal(study_start_decoded.study_start.steps[1].action_tag, DBM_ACTION_BLE_UNBOND,
		      "step 1 action_tag");
	zassert_equal(study_start_decoded.study_start.steps[1].timeout_ms, 5000,
		      "step 1 timeout -- a mis-walked level shifts everything after it");
}

ZTEST(serial_protocol, test_study_start_rejects_an_unknown_security_level)
{
	/* Hand-built bytes: the encoder cannot produce an out-of-range level
	 * (`struct dbm_ble_set_security_action.level` is only ever set from a
	 * decode that already checked it), so this frame has to be written
	 * directly. A security step that silently became a *weaker* one is the
	 * exact silent degradation decision 50 exists to refuse, which is why
	 * this is a whole-frame reject rather than a per-step "unsupported". */
	static const uint8_t body[] = {
		0x06,                                            /* StudyStart */
		0x01,                                            /* steps_len = 1 */
		0x06, 0x73, 0x65, 0x63, 0x75, 0x72, 0x65,        /* name "secure" */
		0x07,                                            /* BleSecurity */
		0x09,                                            /* level = 9 (no such level) */
		0x88, 0x27,                                      /* timeout_ms = 5000 */
		0x00,                                            /* continue_on_fail */
		0x00,                                            /* delay_before_ms */
		0x00, 0x00, 0x00, 0x00,                          /* steps_crc (irrelevant) */
		0x00, 0x00,                                      /* streams, streams_crc */
	};
	uint8_t framed[DBM_MAX_FRAME_LEN];
	size_t framed_len = test_cobs_encode(body, sizeof(body), framed);

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_not_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
			  "an unmappable security level must fail the whole decode");
}

/* Core's own bytes for a study that establishes security and then drops the
 * bond -- decision 36's both-languages rule applied to schema v12's two new
 * actions in the pass that adds them, rather than a version later (which is
 * how `StepResult`'s two stale bytes survived one).
 *
 * Produced by embarch-study-designer/tests/firmware_test_vectors.rs's
 * dump_study_start_with_security_wire_bytes -- run with --nocapture to
 * regenerate. Pre-COBS payload, exactly as dbm_decode_frame's caller sees it
 * after unframing.
 *
 * `BleSecurity` is the first Action variant since `BleConnect` to carry a
 * field, so this is the frame that proves the level varint is walked at the
 * right width by bytes this firmware did not produce. */
static const uint8_t core_study_start_with_security_frame[] = {
	0x06, 0x03,
	/* step 0: "connect", BleConnect, target_name "the client S11" */
	0x07, 0x63, 0x6f, 0x6e, 0x6e, 0x65, 0x63, 0x74, 0x01, 0x00, 0x00, 0x01, 0x0f, 0x45,
	0x69, 0x67, 0x68, 0x74, 0x20, 0x53, 0x6c, 0x65, 0x65, 0x70, 0x20, 0x53, 0x31, 0x31,
	0xa0, 0x9c, 0x01, 0x00, 0x00,
	/* step 1: "secure", BleSecurity { level: L4 }, timeout 10000 */
	0x06, 0x73, 0x65, 0x63, 0x75, 0x72, 0x65, 0x07, 0x03, 0x90, 0x4e, 0x00, 0x00,
	/* step 2: "drop-bond", BleUnbond, timeout 5000 */
	0x09, 0x64, 0x72, 0x6f, 0x70, 0x2d, 0x62, 0x6f, 0x6e, 0x64, 0x08, 0x88, 0x27, 0x00,
	0x00,
	/* steps_crc = 0xB0025B12, then an empty `streams` + its CRC of nothing. */
	0x92, 0xb6, 0x89, 0x80, 0x0b, 0x00, 0x00,
	/* dev_bench_log_level = DevBenchLogLevel::Debug (4) -- schema v13,
	 * design.md §3 decision 39. Deliberately not the default (Warn = 2):
	 * the vector generator picks a value the C struct would not contain by
	 * accident, so an off-by-one in walking the streams_crc that precedes
	 * it cannot pass. */
	0x04,
	/* protocols + protocols_crc -- schema v15
	 * (embarch-study-designer/design.md §3 decision 58): an empty list and
	 * the CRC of nothing, both correct values rather than unset ones. */
	0x00, 0x00,
};

ZTEST(serial_protocol, test_decodes_cores_real_security_study_start_bytes)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];
	size_t framed_len = test_cobs_encode(core_study_start_with_security_frame,
					     sizeof(core_study_start_with_security_frame), framed);

	zassert_true(framed_len > 0, "COBS encode of Core's payload failed");

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "failed to decode the bytes embarch-core actually sends");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_equal(ss->steps_len, 3, "steps_len mismatch");
	zassert_false(ss->has_unsupported_action, "should recognize every action");
	/* The real point: the CRC is computed over the raw `steps` span, so a
	 * level varint walked at the wrong width moves where that span ends
	 * and this fails rather than passing plausibly. */
	zassert_true(ss->steps_crc_valid,
		     "steps_crc computed over Core's own bytes must validate");
	zassert_equal(ss->steps[1].action_tag, DBM_ACTION_BLE_SECURITY, "step 1 action");
	zassert_equal(ss->steps[1].action.set_security.level, DBM_SECURITY_L4, "step 1 level");
	zassert_equal(ss->steps[1].timeout_ms, 10000, "step 1 timeout");
	zassert_str_equal(ss->steps[2].name, "drop-bond", "step 2 name");
	zassert_equal(ss->steps[2].action_tag, DBM_ACTION_BLE_UNBOND, "step 2 action");
	zassert_equal(ss->steps[2].timeout_ms, 5000, "step 2 timeout");

	/* Schema v13 (design.md §3 decision 39): the level the study asked for,
	 * decoded from bytes this firmware did not produce. The generator picks
	 * `Debug` rather than the `Warn` default precisely so a decoder that
	 * ignored this trailing byte -- which would still decode the frame and
	 * still pass every other assertion here -- fails this one. */
	zassert_equal(ss->dev_bench_log_level, DBM_LOG_LEVEL_DBG,
		      "dev_bench_log_level mismatch (got %u)",
		      (unsigned int)ss->dev_bench_log_level);

}

/* ---- `.eap` protocol manifests (embarch-study-designer/design.md §3
 *      decisions 58-62, §4.9) ----------------------------------------------
 *
 * Two kinds of test here, and the split is deliberate.
 *
 * The first is the usual decision-36 cross-language pin: a literal frame that
 * crate produced, decoded here and asserted field by field. Nothing in this
 * file can produce those bytes -- `dbm_encode_frame` writes an empty
 * `protocols` list on purpose, because the decoder discards every name in a
 * manifest and a re-encode would differ -- so a round-trip test would only
 * prove this implementation agrees with itself, which is exactly the blind
 * spot the pinning rule exists to close.
 *
 * The second drives `eap_interp.c` directly. Those tests are the C half of
 * `embarch-study-designer/tests/eap_worked_protocols.rs`: the same worked BDS
 * download, the same sequences, the same expected transitions. They exist
 * because §3 decision 60 put the **executor** on this side while leaving the
 * **specification** in that crate, and the only way that division is safe is
 * if both are exercised against the same cases.
 */

/* Core's own bytes for a `StudyStart` carrying the worked BDS batch-download
 * protocol and an `Action::RunProtocol` step naming it -- schema v15.
 *
 * Produced by embarch-study-designer/tests/firmware_test_vectors.rs's
 * dump_study_start_with_protocol_wire_bytes; run it with --nocapture to
 * regenerate. The protocol is the **real** worked one out of
 * tests/fixtures/bds_batch_download.eap rather than a shrunk stand-in,
 * because a decoder pinned against a purpose-built protocol would prove it
 * can walk a shape nobody authors. Between them these bytes exercise every
 * branch of the walker: three sources, a `select_if` frame *and* an
 * unguarded one, a byte span with no declared length, two session variables,
 * both write forms, a `remember` over `len(...)`, a guarded `goto`, a
 * self-transitioning `otherwise`, a `retry` timeout, a zero-retry stall
 * watchdog, and both terminal outcomes.
 */
static const uint8_t core_study_start_with_protocol_frame[] = {
	/* tag, then one step: "download", Action::RunProtocol { protocol: 0,
	 * entry_state: 0 }, timeout 30000, then steps_crc = 0x8BFD158E. */
	0x06, 0x01, 0x08, 0x64, 0x6f, 0x77, 0x6e, 0x6c, 0x6f, 0x61, 0x64, 0x0b, 0x00, 0x00,
	0xb0, 0xea, 0x01, 0x00, 0x00, 0x8e, 0xab, 0xf4, 0xdf, 0x08,
	/* streams: empty, streams_crc: the CRC of nothing. Then
	 * dev_bench_log_level = Debug (4). */
	0x00, 0x00, 0x04,
	/* protocols: one ProtocolDef. */
	0x01,
	/* name "bds_batch_download" */
	0x12, 0x62, 0x64, 0x73, 0x5f, 0x62, 0x61, 0x74, 0x63, 0x68, 0x5f, 0x64, 0x6f, 0x77,
	0x6e, 0x6c, 0x6f, 0x61, 0x64,
	/* sources: 3 -- "ctrl", "status", "data", each a discarded alias
	 * followed by two raw 16-byte UUIDs. The characteristic UUIDs are the
	 * 16-bit shorthands 0x0021/0x0022/0x0023 expanded through the Bluetooth
	 * Base UUID, which is what makes them differ only in one byte. */
	0x03,
	0x04, 0x63, 0x74, 0x72, 0x6c,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34,
	0x56, 0x78, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80,
	0x5f, 0x9b, 0x34, 0xfb,
	0x06, 0x73, 0x74, 0x61, 0x74, 0x75, 0x73,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34,
	0x56, 0x78, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80,
	0x5f, 0x9b, 0x34, 0xfb,
	0x04, 0x64, 0x61, 0x74, 0x61,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34,
	0x56, 0x78, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80,
	0x5f, 0x9b, 0x34, 0xfb,
	/* frames: 2. "progress" on source 1 with select_if { offset 0, eq [0x02] }
	 * and three scalars (u8 @0, u32be @1, u32be @5); "chunk" on source 2
	 * with no select_if, no scalars, and one span "payload" @0 with no
	 * declared length. */
	0x02,
	0x08, 0x70, 0x72, 0x6f, 0x67, 0x72, 0x65, 0x73, 0x73, 0x01, 0x01, 0x00, 0x01, 0x02,
	0x03,
	0x08, 0x6d, 0x73, 0x67, 0x5f, 0x74, 0x79, 0x70, 0x65, 0x00, 0x00,
	0x06, 0x6f, 0x66, 0x66, 0x73, 0x65, 0x74, 0x01, 0x07,
	0x05, 0x74, 0x6f, 0x74, 0x61, 0x6c, 0x05, 0x07,
	0x00,
	0x05, 0x63, 0x68, 0x75, 0x6e, 0x6b, 0x02, 0x00, 0x00, 0x01,
	0x07, 0x70, 0x61, 0x79, 0x6c, 0x6f, 0x61, 0x64, 0x00, 0x00,
	/* session: 2 -- "received" = 0, "expect_total" = 0 (zigzag). */
	0x02,
	0x08, 0x72, 0x65, 0x63, 0x65, 0x69, 0x76, 0x65, 0x64, 0x00,
	0x0c, 0x65, 0x78, 0x70, 0x65, 0x63, 0x74, 0x5f, 0x74, 0x6f, 0x74, 0x61, 0x6c, 0x00,
	/* states: 6. */
	0x06,
	/* 0 "start": on_enter write ctrl { u8 0x01 } with_response; on_event
	 * frame 0 { remember session[1] = frame.field[2]; otherwise goto 1 };
	 * on_timeout 2000ms retry 2 -> goto 5. */
	0x05, 0x73, 0x74, 0x61, 0x72, 0x74, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x02, 0x01,
	0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x02, 0x00, 0x01, 0x01, 0x01, 0xd0, 0x0f, 0x02,
	0x05,
	/* 1 "pumping": on_enter write ctrl { u8 0x02 } **without** response;
	 * on_event frame 1 { remember session[0] = session[0] + len(span 0);
	 * when session[0] >= session[1] -> goto 2; otherwise goto 1 };
	 * on_timeout 1500ms retry 0 -> goto 3. The self-transitioning
	 * `otherwise` is the flow-control ack, and the one thing this whole
	 * fixture exists to keep honest. */
	0x07, 0x70, 0x75, 0x6d, 0x70, 0x69, 0x6e, 0x67, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
	0x04, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x02, 0x00, 0x03, 0x00, 0x01, 0x02, 0x00,
	0x05, 0x02, 0x01, 0x02, 0x01, 0x01, 0x01, 0xdc, 0x0b, 0x00, 0x03,
	/* 2 "consuming": write ctrl { u8 0x03 } with_response; on_event frame 0
	 * -> goto 4; on_timeout 2000ms retry 1 -> goto 3. */
	0x09, 0x63, 0x6f, 0x6e, 0x73, 0x75, 0x6d, 0x69, 0x6e, 0x67, 0x00, 0x01, 0x00, 0x01,
	0x00, 0x00, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x04, 0x01, 0xd0, 0x0f, 0x01,
	0x03,
	/* 3 "aborting": write ctrl { u8 0x04 } with_response; on_event frame 0
	 * -> goto 5; on_timeout 2000ms retry 0 -> goto 5. */
	0x08, 0x61, 0x62, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67, 0x00, 0x01, 0x00, 0x01, 0x00,
	0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x05, 0x01, 0xd0, 0x0f, 0x00, 0x05,
	/* 4 "done" outcome: pass, 5 "failed" outcome: fail. */
	0x04, 0x64, 0x6f, 0x6e, 0x65, 0x01, 0x00,
	0x06, 0x66, 0x61, 0x69, 0x6c, 0x65, 0x64, 0x01, 0x01,
	/* protocols_crc = 0xEFF3E046. */
	0xc6, 0xc0, 0xcf, 0xff, 0x0e,
};

ZTEST(serial_protocol, test_decodes_cores_study_start_carrying_a_protocol)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];
	size_t framed_len = test_cobs_encode(core_study_start_with_protocol_frame,
					     sizeof(core_study_start_with_protocol_frame), framed);

	zassert_true(framed_len > 0, "COBS encode of Core's payload failed");

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "failed to decode a StudyStart carrying an .eap protocol");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_equal(ss->steps_len, 1, "steps_len mismatch");
	zassert_false(ss->has_unsupported_action, "DBM_ACTION_RUN_PROTOCOL must be recognized");
	zassert_true(ss->steps_crc_valid,
		     "steps_crc must validate -- a RunProtocol action walked as field-less "
		     "reads its two index bytes as the next field and is exactly what this "
		     "catches");
	zassert_equal(ss->steps[0].action_tag, DBM_ACTION_RUN_PROTOCOL, "step 0 action_tag");
	zassert_equal(ss->steps[0].action.run_protocol.protocol, 0, "step 0 protocol index");
	zassert_equal(ss->steps[0].action.run_protocol.entry_state, 0, "step 0 entry_state");
	zassert_equal(ss->steps[0].timeout_ms, 30000, "step 0 timeout");

	/* The third seal, checked independently of the other two -- which is
	 * the whole reason there are three siblings rather than one widened
	 * one: a mismatch says which of the three is corrupt. */
	zassert_true(ss->streams_crc_valid, "streams_crc (of nothing) must still validate");
	zassert_equal(ss->protocols_crc, 0xEFF3E046, "protocols_crc mismatch");
	zassert_true(ss->protocols_crc_valid,
		     "protocols_crc computed over the crate's own manifest bytes must validate");

	zassert_equal(ss->protocols_len, 1, "protocols_len mismatch");

	const struct eap_protocol_def *def = &ss->protocols[0];

	zassert_equal(def->sources_len, 3, "sources_len mismatch");
	zassert_equal(def->frames_len, 2, "frames_len mismatch");
	zassert_equal(def->session_len, 2, "session_len mismatch");
	zassert_equal(def->states_len, 6, "states_len mismatch");

	/* Sources: the three characteristic UUIDs differ in one byte, so a
	 * decoder that read one source three times passes every length
	 * assertion above and fails here. */
	zassert_equal(def->sources[0].characteristic_uuid[3], 0x21, "source 0 characteristic");
	zassert_equal(def->sources[1].characteristic_uuid[3], 0x22, "source 1 characteristic");
	zassert_equal(def->sources[2].characteristic_uuid[3], 0x23, "source 2 characteristic");

	/* Frame 0 dispatches on a magic byte; frame 1 does not, and a frame
	 * with no `select_if` matches any payload on its source. */
	zassert_equal(def->frames[0].source, 1, "frame 0 source");
	zassert_true(def->frames[0].has_select, "frame 0 has a select_if");
	zassert_equal(def->frames[0].select_offset, 0, "frame 0 select offset");
	zassert_equal(def->frames[0].select_len, 1, "frame 0 select length");
	zassert_equal(def->frames[0].select_eq[0], 0x02, "frame 0 select byte");
	zassert_equal(def->frames[0].fields_len, 3, "frame 0 fields_len");
	zassert_equal(def->frames[0].fields[2].offset, 5, "frame 0 field 2 offset");
	zassert_equal(def->frames[0].fields[2].ty, EAP_SCALAR_U32BE, "frame 0 field 2 type");
	zassert_equal(def->frames[0].spans_len, 0, "frame 0 spans_len");

	zassert_equal(def->frames[1].source, 2, "frame 1 source");
	zassert_false(def->frames[1].has_select, "frame 1 has no select_if");
	zassert_equal(def->frames[1].spans_len, 1, "frame 1 spans_len");
	zassert_false(def->frames[1].spans[0].has_len, "frame 1 span is 'the rest of the payload'");

	/* States: the one string a manifest keeps, and the shapes the machine
	 * runs on. */
	zassert_str_equal(def->states[0].name, "start", "state 0 name");
	zassert_equal(def->states[0].kind, EAP_STATE_ACTIVE, "state 0 kind");
	zassert_true(def->states[0].has_on_enter, "state 0 writes on entry");
	zassert_true(def->states[0].on_enter.with_response, "state 0's REQUEST_OLDEST is acked");
	zassert_equal(def->states[0].on_enter.fields_len, 1, "state 0 write field count");
	zassert_equal(def->states[0].on_enter.fields[0].value.kind, EAP_OP_LITERAL,
		      "state 0 write operand kind");
	zassert_equal((int)def->states[0].on_enter.fields[0].value.literal, 0x01,
		      "state 0 write opcode");
	zassert_true(def->states[0].has_on_timeout, "state 0 has a timeout");
	zassert_equal(def->states[0].on_timeout.after_ms, 2000, "state 0 timeout");
	zassert_equal(def->states[0].on_timeout.retry, 2, "state 0 retries");
	zassert_equal(def->states[0].on_timeout.goto_state, 5, "state 0 timeout target");

	/* `with_response` really is per-write, not per-protocol: NEXT_CHUNK is
	 * a Write Command and REQUEST_OLDEST is a Write Request. */
	zassert_str_equal(def->states[1].name, "pumping", "state 1 name");
	zassert_false(def->states[1].on_enter.with_response, "state 1's NEXT_CHUNK is unacked");
	zassert_equal(def->states[1].on_event_len, 1, "state 1 arm count");
	zassert_equal(def->states[1].on_event[0].frame, 1, "state 1 reacts to the chunk frame");
	zassert_equal(def->states[1].on_event[0].remember_len, 1, "state 1 remember count");
	zassert_equal(def->states[1].on_event[0].remember[0].value.kind, EAP_EXPR_ADD,
		      "state 1 accumulates rather than assigning");
	zassert_equal(def->states[1].on_event[0].remember[0].value.b.kind, EAP_OP_SPAN_LEN,
		      "state 1 counts the chunk's own length");
	zassert_equal(def->states[1].on_event[0].when_len, 1, "state 1 guard count");
	zassert_equal(def->states[1].on_event[0].when[0].cond.op, EAP_CMP_GE, "state 1 guard op");
	zassert_equal(def->states[1].on_event[0].when[0].goto_state, 2, "state 1 guard target");
	/* **The distinction row 66 of embarch-decision-reversals.md is about.**
	 * This arm has an `otherwise` pointing at its own state, which
	 * re-enters and re-sends the ack; an absent `otherwise` would consume
	 * every chunk correctly, ack none of them, and stall at the watchdog. */
	zassert_true(def->states[1].on_event[0].has_otherwise, "state 1 has an otherwise");
	zassert_equal(def->states[1].on_event[0].otherwise, 1, "state 1 otherwise self-transitions");

	zassert_str_equal(def->states[4].name, "done", "state 4 name");
	zassert_equal(def->states[4].kind, EAP_STATE_TERMINAL, "state 4 kind");
	zassert_equal(def->states[4].terminal, EAP_TERMINAL_PASS, "state 4 outcome");
	zassert_str_equal(def->states[5].name, "failed", "state 5 name");
	zassert_equal(def->states[5].terminal, EAP_TERMINAL_FAIL, "state 5 outcome");
}

ZTEST(serial_protocol, test_a_corrupt_protocol_span_fails_only_its_own_seal)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];
	uint8_t corrupt[sizeof(core_study_start_with_protocol_frame)];

	memcpy(corrupt, core_study_start_with_protocol_frame, sizeof(corrupt));
	/* Flip one bit of a state name, inside the protocols span and outside
	 * both other spans. The whole point of three sibling seals rather than
	 * one widened one is that this says *which* half of a Study arrived
	 * corrupt. */
	corrupt[sizeof(corrupt) - 20] ^= 0x01;

	size_t framed_len = test_cobs_encode(corrupt, sizeof(corrupt), framed);

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	zassert_equal(dbm_decode_frame(framed, framed_len, &study_start_decoded), 0,
		      "a corrupt protocol must still decode -- the seal reports it, the "
		      "decoder does not refuse it");

	const struct dbm_study_start *ss = &study_start_decoded.study_start;

	zassert_true(ss->steps_crc_valid, "steps_crc is unaffected by a corrupt protocol");
	zassert_true(ss->streams_crc_valid, "streams_crc is unaffected by a corrupt protocol");
	zassert_false(ss->protocols_crc_valid, "protocols_crc must not validate");
}

ZTEST(serial_protocol, test_step_result_with_a_protocol_outcome_encodes_to_the_pinned_wire_bytes)
{
	/* Schema v15's populated trailing field. The all-None vectors above
	 * pin the one appended `0x00`; this is the case that catches an encoder
	 * writing the Option byte and nothing after it.
	 *
	 * The step's own outcome is `TimedOut` while the protocol's is `Fail`
	 * -- deliberately different, because they are two facts and a vector
	 * where they agreed would pass against an encoder that filled one from
	 * the other. Pre-COBS body pinned by embarch-study-designer's
	 * dump_step_result_with_protocol_wire_bytes. */
	static const uint8_t expected[] = {
		0x0d, 0x07, 0x01, 0x08, 0x64, 0x6f, 0x77, 0x6e, 0x6c, 0x6f, 0x61, 0x64,
		0x02, 0x01, 0x01, 0x35, 0x01, 0x08, 0x61, 0x62, 0x6f, 0x72, 0x74, 0x69,
		0x6e, 0x67, 0x01, 0x28, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x63, 0x6f, 0x6c,
		0x20, 0x72, 0x65, 0x61, 0x63, 0x68, 0x65, 0x64, 0x20, 0x74, 0x65, 0x72,
		0x6d, 0x69, 0x6e, 0x61, 0x6c, 0x20, 0x73, 0x74, 0x61, 0x74, 0x65, 0x20,
		0x61, 0x62, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67, 0x00,
	};
	struct dbm_step_result_payload *r = &pinned_step_result_msg.step_result.result;

	memset(&pinned_step_result_msg, 0, sizeof(pinned_step_result_msg));
	pinned_step_result_msg.tag = DBM_TAG_STEP_RESULT;
	pinned_step_result_msg.step_result.step_index = 1;
	strcpy(r->step_name, "download");
	r->outcome.tag = 2; /* TimedOut */
	r->has_protocol = true;
	strcpy(r->protocol_final_state, "aborting");
	r->protocol_outcome.tag = 1; /* Fail */
	strcpy(r->protocol_outcome.fail_reason, "protocol reached terminal state aborting");

	uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(&pinned_step_result_msg, frame, sizeof(frame));

	zassert_equal(frame_len, (int)sizeof(expected), "frame length mismatch");
	zassert_mem_equal(frame, expected, sizeof(expected), "encoded frame mismatch");
}

/* ---- the interpreter itself (eap_interp.c) -----------------------------
 *
 * These are the C half of `embarch-study-designer/tests/eap_worked_protocols.rs`
 * -- the same worked BDS download, the same sequences, the same expected
 * transitions. §3 decision 60 put the executor on this side and left the
 * specification in that crate, and the only thing that makes that division
 * safe is both being driven through the same cases.
 *
 * The manifest under test is not hand-built: it is decoded out of the pinned
 * frame above, so these tests run against the same bytes Core would send.
 */

static struct dev_bench_message interp_msg;

static const struct eap_protocol_def *worked_protocol(void)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];
	size_t framed_len = test_cobs_encode(core_study_start_with_protocol_frame,
					     sizeof(core_study_start_with_protocol_frame), framed);

	memset(&interp_msg, 0, sizeof(interp_msg));
	zassert_equal(dbm_decode_frame(framed, framed_len, &interp_msg), 0,
		      "the worked protocol must decode");
	return &interp_msg.study_start.protocols[0];
}

/* A `progress` notification: type 0x02, then big-endian offset and total. */
static void progress_frame(uint8_t out[9], uint32_t offset, uint32_t total)
{
	out[0] = 0x02;
	out[1] = (uint8_t)(offset >> 24);
	out[2] = (uint8_t)(offset >> 16);
	out[3] = (uint8_t)(offset >> 8);
	out[4] = (uint8_t)offset;
	out[5] = (uint8_t)(total >> 24);
	out[6] = (uint8_t)(total >> 16);
	out[7] = (uint8_t)(total >> 8);
	out[8] = (uint8_t)total;
}

static void notify(struct eap_run *run, uint8_t source, const uint8_t *payload, size_t len,
		   struct eap_step *step)
{
	struct eap_event ev = {
		.kind = EAP_EVENT_NOTIFY, .source = source, .payload = payload, .payload_len = len};

	eap_run_on_event(run, &ev, step);
}

static void expire(struct eap_run *run, struct eap_step *step)
{
	struct eap_event ev = {.kind = EAP_EVENT_TIMEOUT};

	eap_run_on_event(run, &ev, step);
}

ZTEST(serial_protocol, test_bds_download_runs_to_pass_over_a_real_chunk_sequence)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;
	uint8_t progress[9];
	uint8_t chunk[200] = {0};

	zassert_equal(eap_run_start(&run, def, 0), 0, "run starts at `start`");
	eap_run_enter(&run, &step);

	/* start: REQUEST_OLDEST, acknowledged. */
	zassert_equal(step.kind, EAP_STEP_WRITE, "entering `start` writes");
	zassert_equal(step.source, 0, "REQUEST_OLDEST goes to ctrl");
	zassert_equal(step.payload_len, 1, "one-byte opcode");
	zassert_equal(step.payload[0], 0x01, "REQUEST_OLDEST");
	zassert_true(step.with_response, "REQUEST_OLDEST is acked");
	zassert_true(step.has_deadline, "`start` arms its own 2s deadline");
	zassert_equal(step.deadline_ms, 2000, "`start` deadline");

	/* The DUT answers on a *different* characteristic -- the whole reason
	 * nothing here transitions on a write's own ATT response. */
	progress_frame(progress, 0, 700);
	notify(&run, 1, progress, sizeof(progress), &step);

	zassert_equal(step.kind, EAP_STEP_WRITE, "the transition into `pumping` writes");
	zassert_equal(step.payload[0], 0x02, "NEXT_CHUNK");
	zassert_false(step.with_response, "NEXT_CHUNK is a Write Command");
	zassert_equal(eap_run_state(&run), 1, "now in `pumping`");

	/* 700 bytes in 200-byte chunks: three full, one short. Each of the
	 * first three self-transitions, which re-sends the ack. */
	for (int i = 0; i < 3; i++) {
		notify(&run, 2, chunk, 200, &step);
		zassert_equal(step.kind, EAP_STEP_WRITE, "chunk %d re-acks", i);
		zassert_equal(step.payload[0], 0x02, "chunk %d re-sends NEXT_CHUNK", i);
		zassert_equal(eap_run_state(&run), 1, "chunk %d stays in `pumping`", i);
	}

	notify(&run, 2, chunk, 100, &step);
	zassert_equal(eap_run_state(&run), 2, "700 received -> `consuming`");
	zassert_equal(step.kind, EAP_STEP_WRITE, "entering `consuming` writes");
	zassert_equal(step.payload[0], 0x03, "CONSUME_OLDEST");

	notify(&run, 1, progress, sizeof(progress), &step);
	zassert_equal(step.kind, EAP_STEP_DONE, "the run finished");
	zassert_equal(step.outcome.tag, EAP_OUTCOME_PASS, "`done` declares pass");
	zassert_str_equal(step.final_state, "done", "final state");
}

ZTEST(serial_protocol, test_retry_re_sends_the_on_enter_write_rather_than_waiting_longer)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;

	zassert_equal(eap_run_start(&run, def, 0), 0, NULL);
	eap_run_enter(&run, &step);
	zassert_equal(step.payload[0], 0x01, "the first REQUEST_OLDEST");

	/* `retry 2`: two expiries re-send, the third takes the goto. This is
	 * the behavior the whole `retry` field exists for -- a bench that
	 * merely waited longer would look identical for two expiries and
	 * differ only in what the DUT saw. */
	for (int i = 0; i < 2; i++) {
		expire(&run, &step);
		zassert_equal(step.kind, EAP_STEP_WRITE, "retry %d re-sends", i);
		zassert_equal(step.payload[0], 0x01, "retry %d re-sends REQUEST_OLDEST", i);
		zassert_equal(eap_run_state(&run), 0, "retry %d stays in `start`", i);
	}

	expire(&run, &step);
	zassert_equal(step.kind, EAP_STEP_DONE, "retries exhausted -> terminal");
	zassert_equal(step.outcome.tag, EAP_OUTCOME_FAIL, "`failed` declares fail");
	zassert_str_equal(step.final_state, "failed", "final state");
	/* Byte-for-byte the sentence `eap_interp.rs`'s own `fail_reason`
	 * builds, so a run reads identically whichever interpreter produced
	 * it. */
	zassert_str_equal(step.outcome.fail_reason, "protocol reached terminal state failed",
			  "fail_reason must match the Rust reference's wording");
}

ZTEST(serial_protocol, test_a_stalled_pump_takes_the_watchdog_to_aborting_and_then_fails)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;
	uint8_t progress[9];

	zassert_equal(eap_run_start(&run, def, 0), 0, NULL);
	eap_run_enter(&run, &step);
	progress_frame(progress, 0, 700);
	notify(&run, 1, progress, sizeof(progress), &step);
	zassert_equal(eap_run_state(&run), 1, "in `pumping`");

	/* `retry 0` on the stall watchdog: the first expiry takes the goto,
	 * which is what a watchdog wants and the opposite of `start`'s. */
	expire(&run, &step);
	zassert_equal(eap_run_state(&run), 3, "the stall watchdog goes straight to `aborting`");
	zassert_equal(step.kind, EAP_STEP_WRITE, "`aborting` writes ABORT");
	zassert_equal(step.payload[0], 0x04, "ABORT opcode");
	zassert_true(step.with_response, "ABORT is always acked -- the DUT must not stay wedged");

	notify(&run, 1, progress, sizeof(progress), &step);
	zassert_equal(step.kind, EAP_STEP_DONE, "aborting -> failed");
	zassert_str_equal(step.final_state, "failed", "final state");
}

ZTEST(serial_protocol, test_an_unrelated_notification_is_ignored_rather_than_failing_the_run)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;
	uint8_t not_progress[9] = {0x7f, 0, 0, 0, 0, 0, 0, 0, 0};

	zassert_equal(eap_run_start(&run, def, 0), 0, NULL);
	eap_run_enter(&run, &step);

	/* Wrong magic byte on the status characteristic: no frame selects, so
	 * nothing happens. A machine that failed on the first unrelated
	 * notification could not survive a real connection. */
	notify(&run, 1, not_progress, sizeof(not_progress), &step);
	zassert_equal(step.kind, EAP_STEP_WAIT, "an unmatched frame is ignored");
	zassert_equal(eap_run_state(&run), 0, "and does not move the machine");
	zassert_true(step.has_deadline, "the state's deadline keeps running");

	/* A notification on a source no frame reads is equally ignored. */
	notify(&run, 0, not_progress, sizeof(not_progress), &step);
	zassert_equal(step.kind, EAP_STEP_WAIT, "an unread source is ignored");
	zassert_equal(eap_run_state(&run), 0, "and does not move the machine");
}

ZTEST(serial_protocol, test_a_truncated_frame_is_never_zero_filled)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;
	/* `progress.total` lives at offset 5. This payload matches the magic
	 * byte and then stops short of it. */
	uint8_t truncated[5] = {0x02, 0, 0, 0, 1};

	zassert_equal(eap_run_start(&run, def, 0), 0, NULL);
	eap_run_enter(&run, &step);

	notify(&run, 1, truncated, sizeof(truncated), &step);

	/* The arm's `otherwise` is unconditional, so the machine does move --
	 * that is the manifest's decision. What the interpreter guarantees is
	 * the other half: `expect_total` keeps its **declared** initial value
	 * rather than a zero conjured out of bytes that never arrived. Mirrors
	 * the crate's own `a_truncated_frame_does_not_advance_the_machine`. */
	zassert_equal((int)eap_run_session(&run)[1], 0,
		      "the declared initial value, not a decoded one");
}

ZTEST(serial_protocol, test_a_short_payload_makes_a_guard_false_rather_than_true)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;
	uint8_t progress[9];

	zassert_equal(eap_run_start(&run, def, 0), 0, NULL);
	eap_run_enter(&run, &step);
	progress_frame(progress, 0, 700);
	notify(&run, 1, progress, sizeof(progress), &step);
	zassert_equal(eap_run_state(&run), 1, "in `pumping` with expect_total = 700");

	/* A zero-length chunk: `len(chunk.payload)` resolves to 0 (the span is
	 * "the rest of the payload" and there is none), so `received` stays 0
	 * and the guard is false. The machine self-transitions and asks again
	 * rather than declaring the download complete. */
	notify(&run, 2, progress, 0, &step);
	zassert_equal(eap_run_state(&run), 1, "a zero-length chunk does not finish the download");
	zassert_equal(step.kind, EAP_STEP_WRITE, "it re-acks instead");
	zassert_equal(step.payload[0], 0x02, "NEXT_CHUNK again");
}

ZTEST(serial_protocol, test_the_step_timeout_is_the_only_way_to_reach_timed_out)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;
	struct eap_step step;

	zassert_equal(eap_run_start(&run, def, 0), 0, NULL);
	eap_run_enter(&run, &step);

	/* `Outcome::TimedOut` is the one outcome no manifest can declare --
	 * `TerminalOutcome` has exactly `pass` and `fail` -- so the only
	 * producer of it is the step's own budget running out. */
	eap_run_abandon(&run, &step);
	zassert_equal(step.kind, EAP_STEP_DONE, "abandoning ends the run");
	zassert_equal(step.outcome.tag, EAP_OUTCOME_TIMED_OUT, "reported as TimedOut");
	zassert_str_equal(step.final_state, "start",
			  "against whatever state it was sitting in, which is the whole "
			  "diagnostic value of recording one");
}

ZTEST(serial_protocol, test_an_out_of_range_entry_state_is_refused)
{
	const struct eap_protocol_def *def = worked_protocol();
	struct eap_run run;

	/* Belt and braces -- `validate_protocol`, Core's pre-flight and
	 * main.c's own check all range-check this first. Refused here anyway
	 * rather than left to a raw array subscript, which is §3 decision 18's
	 * rule. */
	zassert_equal(eap_run_start(&run, def, def->states_len), -1,
		      "an entry state past the end must be refused");
}

/* ---- the disclosed capacity limits -------------------------------------
 *
 * Every cap this firmware sets below the crate's own is refused **outright
 * and by returning a failure**, never by truncating -- the posture
 * DBM_MAX_STEPS_PER_STUDY established. A truncated protocol is worse than a
 * refused one: a state machine missing an event arm still runs, and branches
 * wrongly while looking correct.
 */

/* A minimal well-formed `protocols` span: one protocol, one source, one
 * frame, no session variables, and `states_len` states whose first has
 * `arms` event arms. Returns the body length written.
 *
 * Hand-built rather than dumped from the crate on purpose: the crate cannot
 * *produce* a manifest past this firmware's own caps -- they are below its
 * ceilings, which is the whole point -- so the only way to test the refusal
 * is to write the bytes here. */
static size_t build_protocol_study_start(uint8_t *out, uint8_t protocols, uint8_t arms)
{
	size_t n = 0;

	out[n++] = 0x06; /* StudyStart */
	out[n++] = 0x00; /* steps: empty */
	out[n++] = 0x00; /* steps_crc = 0 (the CRC of nothing) */
	out[n++] = 0x00; /* streams: empty */
	out[n++] = 0x00; /* streams_crc = 0 */
	out[n++] = 0x00; /* dev_bench_log_level = Off */
	out[n++] = protocols;
	for (uint8_t p = 0; p < protocols; p++) {
		out[n++] = 0x01;
		out[n++] = 'p'; /* name */
		out[n++] = 0x01; /* sources: 1 */
		out[n++] = 0x01;
		out[n++] = 's'; /* alias */
		memset(out + n, 0, 32); /* service + characteristic UUIDs */
		n += 32;
		out[n++] = 0x01; /* frames: 1 */
		out[n++] = 0x01;
		out[n++] = 'f';  /* name */
		out[n++] = 0x00; /* source 0 */
		out[n++] = 0x00; /* select_if: None */
		out[n++] = 0x00; /* fields: none */
		out[n++] = 0x00; /* spans: none */
		out[n++] = 0x00; /* session: none */
		out[n++] = 0x01; /* states: 1 */
		out[n++] = 0x01;
		out[n++] = 'a';  /* name */
		out[n++] = 0x00; /* StateKind::Active */
		out[n++] = 0x00; /* on_enter: None */
		out[n++] = arms; /* on_event */
		for (uint8_t a = 0; a < arms; a++) {
			out[n++] = 0x00; /* frame 0 */
			out[n++] = 0x00; /* remember: none */
			out[n++] = 0x00; /* when: none */
			out[n++] = 0x00; /* otherwise: None */
		}
		out[n++] = 0x00; /* on_timeout: None */
	}
	out[n++] = 0x00; /* protocols_crc (deliberately wrong; unread on a reject) */
	return n;
}

static int decode_built(const uint8_t *body, size_t body_len)
{
	uint8_t framed[DBM_MAX_FRAME_LEN];
	size_t framed_len = test_cobs_encode(body, body_len, framed);

	memset(&study_start_decoded, 0, sizeof(study_start_decoded));
	return dbm_decode_frame(framed, framed_len, &study_start_decoded);
}

ZTEST(serial_protocol, test_a_protocol_within_every_cap_decodes)
{
	uint8_t body[256];
	size_t len = build_protocol_study_start(body, DBM_MAX_PROTOCOLS_PER_STUDY,
						EAP_MAX_EVENT_ARMS_PER_STATE);

	/* The control for the two refusals below: at exactly the caps, this
	 * shape decodes. Without it, a decoder that rejected everything would
	 * pass both of them. */
	zassert_equal(decode_built(body, len), 0, "a manifest at the caps must decode");
	zassert_equal(study_start_decoded.study_start.protocols_len,
		      DBM_MAX_PROTOCOLS_PER_STUDY, "protocols_len mismatch");
	zassert_equal(study_start_decoded.study_start.protocols[0].states[0].on_event_len,
		      EAP_MAX_EVENT_ARMS_PER_STATE, "on_event_len mismatch");
}

ZTEST(serial_protocol, test_more_protocols_than_this_bench_holds_is_refused)
{
	uint8_t body[512];
	size_t len = build_protocol_study_start(body, DBM_MAX_PROTOCOLS_PER_STUDY + 1,
						EAP_MAX_EVENT_ARMS_PER_STATE);

	zassert_not_equal(decode_built(body, len), 0,
			  "protocols_len beyond struct eap_protocol_def[]'s bound must be "
			  "rejected, not read out of bounds");
}

ZTEST(serial_protocol, test_more_event_arms_than_this_bench_holds_is_refused)
{
	uint8_t body[256];
	size_t len = build_protocol_study_start(body, 1, EAP_MAX_EVENT_ARMS_PER_STATE + 1);

	/* The one cap below the crate's own (4 there, 2 here -- see
	 * EAP_MAX_EVENT_ARMS_PER_STATE). Core considers this legal; this
	 * firmware cannot hold it and says so rather than dropping the third
	 * transition, which would be a state machine that runs and branches
	 * wrongly. */
	zassert_not_equal(decode_built(body, len), 0,
			  "an event-arm count past this firmware's cap must be rejected");
}

ZTEST(serial_protocol, test_a_protocols_span_past_the_byte_cap_is_refused)
{
	static uint8_t body[DBM_MAX_PROTOCOLS_WIRE_LEN + 256];
	size_t n = 0;
	/* A protocol *name* long enough to push the span past the byte cap on
	 * its own. Names are the bulk of what a manifest spends bytes on and
	 * the whole of what this firmware discards, which is exactly why the
	 * cap is on bytes rather than on the counts. */
	const size_t name_len = DBM_MAX_PROTOCOLS_WIRE_LEN + 1;

	body[n++] = 0x06;
	body[n++] = 0x00; /* steps: empty */
	body[n++] = 0x00; /* steps_crc */
	body[n++] = 0x00; /* streams: empty */
	body[n++] = 0x00; /* streams_crc */
	body[n++] = 0x00; /* dev_bench_log_level */
	body[n++] = 0x01; /* protocols: 1 */
	/* name: a varint length, then that many bytes. */
	body[n++] = (uint8_t)((name_len & 0x7f) | 0x80);
	body[n++] = (uint8_t)(name_len >> 7);
	memset(body + n, 'x', name_len);
	n += name_len;
	body[n++] = 0x00; /* sources: none */
	body[n++] = 0x00; /* frames: none */
	body[n++] = 0x00; /* session: none */
	body[n++] = 0x00; /* states: none */
	body[n++] = 0x00; /* protocols_crc */

	zassert_true(n <= sizeof(body), "test body overflowed its own buffer");
	zassert_not_equal(decode_built(body, n), 0,
			  "a protocols span past DBM_MAX_PROTOCOLS_WIRE_LEN must be refused");
}
