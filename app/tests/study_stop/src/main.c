/*
 * study_stop.c against a scripted byte source: the stop reaches a step that
 * is waiting, a capture that is running, and is a harmless no-op when no
 * study is running. The frames are built by serial_protocol.c's own encoder,
 * whose bytes the study-designer crate's pinned vector fixes.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "analog_stream.h"
#include "serial_protocol.h"
#include "study_stop.h"

/* The wait a "step" is given, far longer than any test should take. */
#define STEP_TIMEOUT_MS 5000U
/* A stop must end that wait within this: many poll slices, still far below
 * the timeout. */
#define STOP_NOTICED_WITHIN_MS 500
#define SCRIPT_BYTES 64U
#define FRAME_BYTES 16U
#define BLOCK_CYCLES 256000U
#define WORK_SETTLE_MS 20

/* The bytes the link "receives", handed out one per call. `hold_after` stops
 * the supply after that many bytes until released, to stand in for a frame
 * that arrives in pieces or late. */
static struct {
	uint8_t bytes[SCRIPT_BYTES];
	size_t len;
	size_t next;
	size_t hold_after;
	unsigned int polls;
} link_rx;

static int script_byte(uint8_t *byte)
{
	link_rx.polls++;
	if (link_rx.next >= link_rx.len || link_rx.next >= link_rx.hold_after) {
		return -1;
	}
	*byte = link_rx.bytes[link_rx.next++];
	return 0;
}

static void script_append(const struct dev_bench_message *msg)
{
	int n = dbm_encode_frame(msg, &link_rx.bytes[link_rx.len], sizeof(link_rx.bytes) - link_rx.len);

	zassert_true(n > 0, "the frame did not encode");
	link_rx.len += (size_t)n;
}

static void script_stop(uint8_t reason)
{
	struct dev_bench_message msg = {.tag = DBM_TAG_STUDY_STOP, .study_stop = {.reason = reason}};

	script_append(&msg);
}

static void before(void *unused)
{
	ARG_UNUSED(unused);
	memset(&link_rx, 0, sizeof(link_rx));
	link_rx.hold_after = SIZE_MAX;
	/* A study that has not begun: forgets the last one's latched stop. */
	study_stop_begin(NULL);
}

static void after(void *unused)
{
	ARG_UNUSED(unused);
	study_stop_end();
}

/* ---- the fake analog backend ---------------------------------------- */

static struct {
	analog_source_block_cb cb;
	void *ctx;
	bool started;
	uint32_t sink_bytes;
} fake;

static int fake_prepare(const struct analog_source_params *params, struct analog_source_info *info)
{
	*info = (struct analog_source_info){
		.sample_rate_hz = params->sample_rate_hz,
		.resolution_bits = 12,
		.gain_numerator = 1,
		.gain_denominator = 4,
		.reference = ANALOG_REFERENCE_VDD_DIV4,
		.channel_count = params->channel_count,
		.vdd_mv = 3300,
		.input_name = "A0",
	};
	return 0;
}

static int fake_start(analog_source_block_cb cb, void *ctx)
{
	fake.cb = cb;
	fake.ctx = ctx;
	fake.started = true;
	return 0;
}

static void fake_stop(void)
{
	fake.cb = NULL;
	fake.started = false;
}

static const struct analog_source_backend fake_backend = {
	.prepare = fake_prepare,
	.start = fake_start,
	.stop = fake_stop,
};

static int sink_write(void *ctx, const uint8_t *data, size_t len)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(data);
	fake.sink_bytes += (uint32_t)len;
	return 0;
}

static void deliver_block(uint32_t cycles)
{
	int16_t codes[ANALOG_SAMPLES_PER_BLOCK] = {0};

	fake.cb(fake.ctx, codes, cycles);
}

static int start_capture(void)
{
	struct analog_stream_config config = {
		.backend = &fake_backend,
		.sink = {.write = sink_write},
		.params = {.sample_rate_hz = 32000, .channel = 0, .channel_count = 1},
		.cycles_per_sec = 32000000U,
		.board = "native_sim",
		.image_version = "test",
	};

	memset(&fake, 0, sizeof(fake));
	return analog_stream_start(&config);
}

/* ---- the frame reader ------------------------------------------------ */

ZTEST(study_stop, test_a_stop_frame_is_latched_with_its_reason)
{
	script_stop(DBM_STOP_REASON_CAP);
	study_stop_begin(script_byte);

	zassert_true(study_stop_poll());
	zassert_equal(study_stop_reason(), DBM_STOP_REASON_CAP);
}

ZTEST(study_stop, test_every_reason_is_read)
{
	for (uint8_t reason = 0; reason < CRATE_STOP_REASON_COUNT; reason++) {
		before(NULL);
		script_stop(reason);
		study_stop_begin(script_byte);
		zassert_true(study_stop_poll());
		zassert_equal(study_stop_reason(), reason);
	}
}

ZTEST(study_stop, test_a_frame_that_arrives_in_pieces_is_read_when_whole)
{
	script_stop(DBM_STOP_REASON_DONE);
	link_rx.hold_after = 2U;
	study_stop_begin(script_byte);

	zassert_false(study_stop_poll(), "half a frame is not a stop");
	link_rx.hold_after = SIZE_MAX;
	zassert_true(study_stop_poll());
	zassert_equal(study_stop_reason(), DBM_STOP_REASON_DONE);
}

ZTEST(study_stop, test_nothing_received_is_not_a_stop)
{
	study_stop_begin(script_byte);

	zassert_false(study_stop_poll());
	zassert_false(study_stop_poll());
}

ZTEST(study_stop, test_another_message_is_discarded_and_a_later_stop_still_reads)
{
	struct dev_bench_message hello = {.tag = DBM_TAG_HELLO, .hello = {.schema_version = 17}};

	script_append(&hello);
	study_stop_begin(script_byte);
	zassert_false(study_stop_poll(), "a Hello is not a StudyStop");

	script_stop(DBM_STOP_REASON_CANCEL);
	zassert_true(study_stop_poll());
	zassert_equal(study_stop_reason(), DBM_STOP_REASON_CANCEL);
}

ZTEST(study_stop, test_an_unknown_reason_is_not_a_stop)
{
	/* tag 9, reason 7: COBS code 3, then the two bytes, then the delimiter. */
	const uint8_t frame[] = {0x03, DBM_TAG_STUDY_STOP, 0x07, 0x00};

	memcpy(link_rx.bytes, frame, sizeof(frame));
	link_rx.len = sizeof(frame);
	study_stop_begin(script_byte);

	zassert_false(study_stop_poll());
}

ZTEST(study_stop, test_a_long_frame_is_dropped_whole_and_the_next_one_reads)
{
	/* More bytes than a StudyStop can be, then its delimiter, then a real one. */
	memset(link_rx.bytes, 0x11, 10U);
	link_rx.len = 10U;
	link_rx.bytes[link_rx.len++] = 0x00;
	script_stop(DBM_STOP_REASON_CAP);
	study_stop_begin(script_byte);

	zassert_true(study_stop_poll());
	zassert_equal(study_stop_reason(), DBM_STOP_REASON_CAP);
}

ZTEST(study_stop, test_begin_forgets_an_earlier_stop)
{
	script_stop(DBM_STOP_REASON_CAP);
	study_stop_begin(script_byte);
	zassert_true(study_stop_poll());

	memset(&link_rx, 0, sizeof(link_rx));
	link_rx.hold_after = SIZE_MAX;
	study_stop_begin(script_byte);
	zassert_false(study_stop_poll(), "the next study starts with no stop pending");
}

ZTEST(study_stop, test_after_end_a_stop_is_left_unread)
{
	script_stop(DBM_STOP_REASON_CAP);
	study_stop_begin(script_byte);
	study_stop_end();

	zassert_false(study_stop_poll());
	zassert_equal(link_rx.next, 0U, "nothing was taken from the link: the idle loop reads it");
}

/* ---- a step that is waiting ------------------------------------------ */

ZTEST(study_stop, test_a_stop_ends_a_step_that_is_waiting)
{
	script_stop(DBM_STOP_REASON_DONE);
	/* The stop is not on the link yet when the wait begins. */
	link_rx.hold_after = 0U;
	study_stop_begin(script_byte);

	int64_t begun = k_uptime_get();

	k_msleep(3 * STUDY_STOP_POLL_SLICE_MS);
	link_rx.hold_after = SIZE_MAX;

	zassert_true(study_stop_sleep_ms(STEP_TIMEOUT_MS), "the stop must end the wait");
	zassert_true(k_uptime_get() - begun < STOP_NOTICED_WITHIN_MS,
		     "the wait ran to near its timeout instead of ending");
}

ZTEST(study_stop, test_a_wait_with_no_stop_runs_its_full_time)
{
	study_stop_begin(script_byte);

	int64_t begun = k_uptime_get();

	zassert_false(study_stop_sleep_ms(3 * STUDY_STOP_POLL_SLICE_MS));
	zassert_true(k_uptime_get() - begun >= 3 * STUDY_STOP_POLL_SLICE_MS,
		     "the wait ended early with no stop");
	zassert_true(link_rx.polls > 1U, "the wait polled the link while it slept");
}

ZTEST(study_stop, test_a_zero_wait_still_looks_once)
{
	script_stop(DBM_STOP_REASON_CAP);
	study_stop_begin(script_byte);

	zassert_true(study_stop_sleep_ms(0U), "a stop already on the link is seen at once");
}

/* ---- nothing running ------------------------------------------------- */

ZTEST(study_stop, test_a_stop_with_nothing_running_is_a_harmless_no_op)
{
	/* No study began (before() gave the reader no source): nothing latches. */
	zassert_false(study_stop_poll());
	zassert_false(study_stop_sleep_ms(STUDY_STOP_POLL_SLICE_MS));
	study_stop_end();
	zassert_false(study_stop_poll());
}

/* ---- the idle loop's decode ------------------------------------------ */

ZTEST(study_stop, test_the_full_decoder_reads_a_stop_for_the_idle_loop)
{
	static struct dev_bench_message decoded;
	uint8_t frame[FRAME_BYTES];
	struct dev_bench_message msg = {.tag = DBM_TAG_STUDY_STOP,
					.study_stop = {.reason = DBM_STOP_REASON_CANCEL}};
	int n = dbm_encode_frame(&msg, frame, sizeof(frame));

	zassert_true(n > 1);
	/* The delimiter is the caller's to strip. */
	zassert_ok(dbm_decode_frame(frame, (size_t)n - 1U, &decoded));
	zassert_equal(decoded.tag, DBM_TAG_STUDY_STOP);
	zassert_equal(decoded.study_stop.reason, DBM_STOP_REASON_CANCEL);
}

/* ---- an analog capture ----------------------------------------------- */

ZTEST(study_stop, test_a_stop_during_an_analog_capture_stops_it_and_counts_what_it_lost)
{
	struct analog_stream_stats stats;
	unsigned int slots = 4U;

	zassert_ok(start_capture());
	study_stop_begin(script_byte);

	/* Blocks deliver; with the scheduler locked the work cannot drain the
	 * slots, so the last two are lost before the stop arrives. */
	k_sched_lock();
	for (unsigned int b = 0; b < slots + 2U; b++) {
		deliver_block((b + 1U) * BLOCK_CYCLES);
	}
	k_sched_unlock();
	k_msleep(WORK_SETTLE_MS);

	zassert_false(study_stop_poll(), "no stop yet, the capture is still running");
	zassert_true(analog_stream_running());

	script_stop(DBM_STOP_REASON_CAP);
	zassert_true(study_stop_poll());

	/* What main.c does on a stop: end the capture and report its loss. */
	analog_stream_stop(&stats);
	zassert_false(analog_stream_running());
	zassert_false(fake.started, "the backend was stopped");
	zassert_equal(stats.blocks_lost, 2U, "the two blocks with no slot are the tap's dropped");
	zassert_equal(stats.blocks_sent, slots);

	/* A second close changes nothing. */
	analog_stream_stop(&stats);
	zassert_false(analog_stream_running());
	zassert_equal(stats.blocks_lost, 2U, "the finished session's totals stay put");
}

ZTEST(study_stop, test_a_stop_with_no_capture_open_leaves_the_stream_idle)
{
	struct analog_stream_stats stats;

	study_stop_begin(script_byte);
	script_stop(DBM_STOP_REASON_CANCEL);
	zassert_true(study_stop_poll());

	/* The totals of an earlier session are not asserted: analog_stream keeps
	 * them until the next start, which is why analog_capture_stop() only
	 * reports them for a session that was running. */
	analog_stream_stop(&stats);
	zassert_false(analog_stream_running());
}

ZTEST_SUITE(study_stop, NULL, NULL, before, after, NULL);
