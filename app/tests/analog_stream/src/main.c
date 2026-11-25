/*
 * analog_stream.c against a fake backend and a fake sink: the start header
 * and the blocks it writes are compared byte for byte with the bytes
 * embarch-study-designer's encoder produces (fixtures.h), and the index,
 * loss accounting and marker placement are checked on top.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "analog_stream.h"
#include "fixtures.h"

#define BLOCK_1CH ANALOG_BLOCK_BYTES(1)
#define BLOCK_2CH ANALOG_BLOCK_BYTES(2)
#define SINK_CAPACITY 8192U
/* The timestamp clock of the tests: 1000 cycles per frame at 32 ksps. */
#define CYCLES_PER_SEC 32000000U
#define CYCLES_PER_FRAME 1000U
#define BLOCK_CYCLES (ANALOG_SAMPLES_PER_BLOCK * CYCLES_PER_FRAME)
/* Four slots, set in CMakeLists.txt. */
#define SLOTS 4U
#define WORK_SETTLE_MS 20

static struct {
	uint8_t bytes[SINK_CAPACITY];
	size_t len;
	unsigned int writes;
	/* Refuse the next write when non-zero, counting down. */
	unsigned int refuse_after;
	bool refuse_armed;
} sink;

static struct {
	analog_source_block_cb cb;
	void *ctx;
	bool prepared;
	bool started;
	int prepare_result;
	struct analog_source_info info;
} fake;

static int fake_prepare(const struct analog_source_params *params, struct analog_source_info *info)
{
	if (fake.prepare_result != 0) {
		return fake.prepare_result;
	}
	*info = fake.info;
	info->channel_count = params->channel_count;
	fake.prepared = true;
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
	fake.prepared = false;
}

static const struct analog_source_backend fake_backend = {
	.prepare = fake_prepare,
	.start = fake_start,
	.stop = fake_stop,
};

static int sink_write(void *ctx, const uint8_t *data, size_t len)
{
	ARG_UNUSED(ctx);

	if (sink.refuse_armed) {
		if (sink.refuse_after == 0U) {
			sink.refuse_armed = false;
			return -ENOSPC;
		}
		sink.refuse_after--;
	}
	zassert_true(sink.len + len <= SINK_CAPACITY, "the fake sink is full");
	memcpy(&sink.bytes[sink.len], data, len);
	sink.len += len;
	sink.writes++;
	return 0;
}

static void reset_fakes(void)
{
	memset(&sink, 0, sizeof(sink));
	memset(&fake, 0, sizeof(fake));
	fake.info = (struct analog_source_info){
		.sample_rate_hz = 32000,
		.resolution_bits = 12,
		.gain_numerator = 1,
		.gain_denominator = 4,
		.reference = ANALOG_REFERENCE_VDD_DIV4,
		.channel_count = 1,
		.vdd_mv = 3300,
		.input_name = "A0",
	};
}

static void before(void *unused)
{
	ARG_UNUSED(unused);
	reset_fakes();
}

static struct analog_stream_config make_config(uint8_t channels)
{
	return (struct analog_stream_config){
		.backend = &fake_backend,
		.sink = {.write = sink_write},
		.params = {.sample_rate_hz = 32000, .channel = 0, .channel_count = channels},
		.cycles_per_sec = CYCLES_PER_SEC,
		.board = "nrf52840dk",
		.image_version = "v1.0.0-abc",
	};
}

/* The test codes the Rust fixtures use: (i * mul + seed) % 4096 - offset. */
static void fill_codes(int16_t *codes, size_t count, int mul, int seed, int offset)
{
	for (size_t i = 0; i < count; i++) {
		codes[i] = (int16_t)((((int)i * mul + seed) % 4096) - offset);
	}
}

static void deliver_1ch(int seed, uint32_t cycles)
{
	int16_t codes[ANALOG_SAMPLES_PER_BLOCK];

	fill_codes(codes, ARRAY_SIZE(codes), 37, seed, 100);
	fake.cb(fake.ctx, codes, cycles);
}

static void settle(void)
{
	k_msleep(WORK_SETTLE_MS);
}

static uint32_t block_index(const uint8_t *block)
{
	return sys_get_le32(&block[4]);
}

ZTEST(analog_stream, test_crc_is_iso_hdlc)
{
	/* The check value of CRC-32/ISO-HDLC, the algorithm the wire names. */
	zassert_equal(crc32_ieee((const uint8_t *)"123456789", 9), 0xCBF43926U);
}

ZTEST(analog_stream, test_header_and_blocks_match_the_encoders_bytes)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;
	uint32_t block1_end = 2U * BLOCK_CYCLES;
	/* 155.5 frames before the block's last frame: frame offset
	 * 255 - 155 = 100. */
	uint32_t first_edge = block1_end - 155U * CYCLES_PER_FRAME - CYCLES_PER_FRAME / 2U;

	zassert_ok(analog_stream_start(&config));
	zassert_true(analog_stream_running());
	zassert_true(fake.started);

	deliver_1ch(0, BLOCK_CYCLES);
	/* Two edges in the second block: high first, low after. */
	analog_stream_marker_edge(true, first_edge);
	analog_stream_marker_edge(false, block1_end - CYCLES_PER_FRAME);
	deliver_1ch(1234, block1_end);

	analog_stream_stop(&stats);
	zassert_false(analog_stream_running());
	zassert_false(fake.started);

	zassert_equal(stats.blocks_sent, 2U);
	zassert_equal(stats.blocks_lost, 0U);
	zassert_equal(sink.len, sizeof(fixture_start_header) + 2U * BLOCK_1CH);
	zassert_mem_equal(&sink.bytes[0], fixture_start_header, sizeof(fixture_start_header),
			  "start header");
	zassert_mem_equal(&sink.bytes[75], fixture_block_0, BLOCK_1CH, "block 0");
	zassert_mem_equal(&sink.bytes[75 + BLOCK_1CH], fixture_block_1, BLOCK_1CH, "block 1");
}

ZTEST(analog_stream, test_two_channels_are_frame_interleaved_with_the_channel_count_in_the_header)
{
	struct analog_stream_config config = make_config(2);
	int16_t codes[2 * ANALOG_SAMPLES_PER_BLOCK];
	struct analog_stream_stats stats;

	zassert_ok(analog_stream_start(&config));
	/* Blocks 0 to 2 are filler; the fourth is the fixture's, at index 768. */
	for (unsigned int b = 0; b < 3U; b++) {
		fill_codes(codes, ARRAY_SIZE(codes), 1, (int)b, 0);
		fake.cb(fake.ctx, codes, (b + 1U) * BLOCK_CYCLES);
		settle();
	}
	fill_codes(codes, ARRAY_SIZE(codes), 29, 77, 2048);
	fake.cb(fake.ctx, codes, 4U * BLOCK_CYCLES);
	analog_stream_stop(&stats);

	zassert_equal(stats.blocks_sent, 4U);
	zassert_mem_equal(&sink.bytes[0], fixture_start_header_2ch,
			  sizeof(fixture_start_header_2ch), "two-channel header");
	zassert_mem_equal(&sink.bytes[75 + 3U * BLOCK_2CH], fixture_block_2ch, BLOCK_2CH,
			  "two-channel block");
}

ZTEST(analog_stream, test_a_refused_block_is_lost_and_the_index_skips_it)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;

	zassert_ok(analog_stream_start(&config));
	/* The header was write 1. Let block 0 through, refuse block 1. */
	sink.refuse_armed = true;
	sink.refuse_after = 1U;
	for (unsigned int b = 0; b < 3U; b++) {
		deliver_1ch((int)b, (b + 1U) * BLOCK_CYCLES);
		settle();
	}
	analog_stream_stop(&stats);

	zassert_equal(stats.blocks_sent, 2U);
	zassert_equal(stats.blocks_lost, 1U);
	zassert_equal(sink.len, 75U + 2U * BLOCK_1CH);
	zassert_equal(block_index(&sink.bytes[75]), 0U);
	zassert_equal(block_index(&sink.bytes[75 + BLOCK_1CH]), 2U * ANALOG_SAMPLES_PER_BLOCK,
		      "the lost block leaves a gap of one block in the index");
}

ZTEST(analog_stream, test_no_free_slot_drops_the_block_and_keeps_the_index)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;
	unsigned int delivered = SLOTS + 2U;

	zassert_ok(analog_stream_start(&config));
	/* With the scheduler locked the delivery work cannot run, so the slots
	 * fill: the last two blocks find none. */
	k_sched_lock();
	for (unsigned int b = 0; b < delivered; b++) {
		deliver_1ch((int)b, (b + 1U) * BLOCK_CYCLES);
	}
	k_sched_unlock();
	settle();
	deliver_1ch((int)delivered, (delivered + 1U) * BLOCK_CYCLES);
	analog_stream_stop(&stats);

	zassert_equal(stats.blocks_lost, 2U);
	zassert_equal(stats.blocks_sent, SLOTS + 1U);
	/* The block after the two lost ones carries index 6 * 256. */
	zassert_equal(block_index(&sink.bytes[75 + SLOTS * BLOCK_1CH]),
		      delivered * ANALOG_SAMPLES_PER_BLOCK);
}

ZTEST(analog_stream, test_an_edge_after_the_block_waits_for_the_next_one)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;

	zassert_ok(analog_stream_start(&config));
	analog_stream_marker_edge(true, BLOCK_CYCLES + 1U);
	deliver_1ch(0, BLOCK_CYCLES);
	settle();
	deliver_1ch(0, 2U * BLOCK_CYCLES);
	analog_stream_stop(&stats);

	zassert_equal(sys_get_le32(&sink.bytes[75 + 8]), 0U, "no edge in the first block");
	uint32_t word = sys_get_le32(&sink.bytes[75 + BLOCK_1CH + 8]);

	zassert_equal(word >> 16, 1U, "one edge in the second");
	zassert_equal((word >> 8) & 1U, 1U, "high after it");
}

ZTEST(analog_stream, test_a_backend_that_cannot_prepare_writes_nothing)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;

	fake.prepare_result = -ENOTSUP;
	zassert_equal(analog_stream_start(&config), -ENOTSUP);
	zassert_false(analog_stream_running());
	zassert_equal(sink.len, 0U);

	analog_stream_stop(&stats);
	zassert_equal(stats.blocks_sent, 0U);
}

ZTEST(analog_stream, test_a_sink_that_refuses_the_header_stops_the_start)
{
	struct analog_stream_config config = make_config(1);

	sink.refuse_armed = true;
	sink.refuse_after = 0U;
	zassert_equal(analog_stream_start(&config), -ENOSPC);
	zassert_false(analog_stream_running());
	zassert_false(fake.prepared, "the backend is released");
}

ZTEST(analog_stream, test_a_second_start_while_running_is_refused)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;

	zassert_ok(analog_stream_start(&config));
	zassert_equal(analog_stream_start(&config), -EBUSY);
	analog_stream_stop(&stats);
}

ZTEST(analog_stream, test_a_channel_count_past_the_build_is_refused)
{
	struct analog_stream_config config = make_config(3);

	zassert_equal(analog_stream_start(&config), -EINVAL);
}

ZTEST(analog_stream, test_an_over_long_version_is_cut_to_its_field)
{
	struct analog_stream_config config = make_config(1);
	struct analog_stream_stats stats;

	config.image_version = "v1.0.0-12-gabcdef12-dirty";
	zassert_ok(analog_stream_start(&config));
	analog_stream_stop(&stats);

	/* Sixteen bytes, no terminator: the decoder accepts a full field. */
	zassert_mem_equal(&sink.bytes[55], "v1.0.0-12-gabcde", 16);
	zassert_equal(sys_get_le32(&sink.bytes[71]), crc32_ieee(sink.bytes, 71), "header CRC");
}

ZTEST_SUITE(analog_stream, NULL, NULL, before, NULL, NULL);
