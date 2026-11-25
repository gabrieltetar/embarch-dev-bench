/*
 * The analog stream, see analog_stream.h.
 *
 * Context flow: the backend calls on_block() (interrupt context) once per
 * block; it copies the codes into a free slot, stamps index and event word
 * and queues the slot. A work item computes the CRC and writes the slot to
 * the sink. Nothing here blocks: when no slot is free or the sink refuses, the
 * block is counted lost and the index skips it, which is how the reader finds
 * the gap (dev-bench decision 64).
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "analog_stream.h"

LOG_MODULE_REGISTER(analog_stream, LOG_LEVEL_INF);

/* "EAH1" and "EAB1" as written to the wire, interpreted as little-endian
 * u32 so sys_put_le32() emits them in reading order. */
#define START_MAGIC 0x31484145U
#define BLOCK_MAGIC 0x31424145U

/* Start header offsets (interfaces/analog.md). */
#define HDR_OFF_RATE 4U
#define HDR_OFF_RESOLUTION 8U
#define HDR_OFF_GAIN_NUM 9U
#define HDR_OFF_GAIN_DEN 10U
#define HDR_OFF_REFERENCE 11U
#define HDR_OFF_CHANNELS 12U
#define HDR_OFF_VDD_MV 13U
#define HDR_OFF_INPUT_NAME 15U
#define HDR_OFF_BOARD (HDR_OFF_INPUT_NAME + ANALOG_HEADER_INPUT_NAME_BYTES)
#define HDR_OFF_IMAGE_VERSION (HDR_OFF_BOARD + ANALOG_HEADER_BOARD_BYTES)
#define HDR_OFF_CRC (HDR_OFF_IMAGE_VERSION + ANALOG_HEADER_IMAGE_VERSION_BYTES)
BUILD_ASSERT(HDR_OFF_CRC + sizeof(uint32_t) == ANALOG_START_HEADER_BYTES,
	     "the start header is 75 bytes");

/* Block offsets. */
#define BLK_OFF_INDEX 4U
#define BLK_OFF_EVENTS 8U
#define BLK_OFF_CODES 12U
#define CRC_BYTES 4U

/* Event word: bits 0..7 first-edge frame offset, bit 8 the level after that
 * edge, bits 16..23 the edge count. */
#define EVENT_LEVEL_SHIFT 8U
#define EVENT_COUNT_SHIFT 16U
#define EVENT_COUNT_MAX 255U

#define MAX_BLOCK_BYTES ANALOG_BLOCK_BYTES(ANALOG_MAX_CHANNELS)
#define SLOT_COUNT CONFIG_EMBARCH_ANALOG_STREAM_SLOTS

struct marker_state {
	uint8_t edge_count;
	bool first_level;
	uint32_t first_cycles;
};

/* All module state in one place. Which context touches what:
 *  - config, running, block_bytes, cycles_per_frame: written by start/stop
 *    while no backend callback can run, read by on_block();
 *  - next_index, the slot data it fills: on_block() only;
 *  - marker: the marker interrupt and on_block(), under irq_lock;
 *  - the counters: atomics, since on_block() and the work item both bump them. */
static struct {
	struct analog_stream_config config;
	bool running;
	size_t block_bytes;
	uint32_t cycles_per_frame;
	uint32_t next_index;
	struct marker_state marker;
	uint8_t slot[SLOT_COUNT][MAX_BLOCK_BYTES];
	struct k_msgq free_q;
	struct k_msgq full_q;
	uint8_t free_q_storage[SLOT_COUNT];
	uint8_t full_q_storage[SLOT_COUNT];
	struct k_work work;
	struct k_work_sync work_sync;
	atomic_t blocks_sent;
	atomic_t blocks_lost;
} m_self;

/* Copies `src` into a NUL-padded field of `field_bytes`, warning when it had
 * to cut it. */
static void put_text(uint8_t *field, size_t field_bytes, const char *src, const char *what)
{
	size_t len = strlen(src);

	memset(field, 0, field_bytes);
	if (len > field_bytes) {
		LOG_WRN("%s: %s \"%s\" is longer than %zu bytes, cut", __func__, what, src,
			field_bytes);
		len = field_bytes;
	}
	memcpy(field, src, len);
}

/* Builds the 75-byte start header for what the backend reported. */
static void build_start_header(uint8_t *out, const struct analog_source_info *info)
{
	memset(out, 0, ANALOG_START_HEADER_BYTES);
	sys_put_le32(START_MAGIC, out);
	sys_put_le32(info->sample_rate_hz, &out[HDR_OFF_RATE]);
	out[HDR_OFF_RESOLUTION] = info->resolution_bits;
	out[HDR_OFF_GAIN_NUM] = info->gain_numerator;
	out[HDR_OFF_GAIN_DEN] = info->gain_denominator;
	out[HDR_OFF_REFERENCE] = (uint8_t)info->reference;
	out[HDR_OFF_CHANNELS] = info->channel_count;
	sys_put_le16(info->vdd_mv, &out[HDR_OFF_VDD_MV]);
	put_text(&out[HDR_OFF_INPUT_NAME], ANALOG_HEADER_INPUT_NAME_BYTES, info->input_name,
		 "input name");
	put_text(&out[HDR_OFF_BOARD], ANALOG_HEADER_BOARD_BYTES, m_self.config.board, "board");
	put_text(&out[HDR_OFF_IMAGE_VERSION], ANALOG_HEADER_IMAGE_VERSION_BYTES,
		 m_self.config.image_version, "image version");
	sys_put_le32(crc32_ieee(out, HDR_OFF_CRC), &out[HDR_OFF_CRC]);
}

/* The event word for the pending marker edges that happened by `cycles`, the
 * moment the block's last frame was taken, and the pending state cleared.
 * Edges stamped after `cycles` stay pending for the next block.
 *
 * The first edge's frame offset is counted back from the block's end by the
 * time since the edge; that time comes from two interrupt timestamps, so it
 * carries their latency (not measured here). */
static uint32_t take_event_word(uint32_t cycles)
{
	uint32_t word = 0;
	unsigned int key = irq_lock();
	struct marker_state edges = m_self.marker;

	if (edges.edge_count != 0U && (int32_t)(cycles - edges.first_cycles) >= 0) {
		memset(&m_self.marker, 0, sizeof(m_self.marker));
	} else {
		edges.edge_count = 0U;
	}
	irq_unlock(key);

	if (edges.edge_count != 0U) {
		uint32_t frames_ago = (cycles - edges.first_cycles) / m_self.cycles_per_frame;
		uint32_t offset = (frames_ago >= ANALOG_SAMPLES_PER_BLOCK)
					  ? 0U
					  : (ANALOG_SAMPLES_PER_BLOCK - 1U - frames_ago);

		word = ((uint32_t)edges.edge_count << EVENT_COUNT_SHIFT) |
		       ((uint32_t)edges.first_level << EVENT_LEVEL_SHIFT) | offset;
	}
	return word;
}

static void count_lost(void)
{
	atomic_inc(&m_self.blocks_lost);
}

/* The backend's block callback: interrupt context, so only copies and
 * queues. */
static void on_block(void *ctx, const int16_t *codes, uint32_t cycles)
{
	uint8_t slot_index;
	uint8_t *block;
	size_t code_bytes = m_self.block_bytes - BLK_OFF_CODES - CRC_BYTES;

	ARG_UNUSED(ctx);

	if (!m_self.running) {
		return;
	}

	uint32_t index = m_self.next_index;

	m_self.next_index += ANALOG_SAMPLES_PER_BLOCK;

	if (k_msgq_get(&m_self.free_q, &slot_index, K_NO_WAIT) != 0) {
		count_lost();
		return;
	}

	block = m_self.slot[slot_index];
	sys_put_le32(BLOCK_MAGIC, block);
	sys_put_le32(index, &block[BLK_OFF_INDEX]);
	sys_put_le32(take_event_word(cycles), &block[BLK_OFF_EVENTS]);
	/* The codes are int16 in the host's byte order; the wire is
	 * little-endian, so a big-endian target would need a swap here. */
	BUILD_ASSERT(IS_ENABLED(CONFIG_LITTLE_ENDIAN), "block codes are copied as they are");
	memcpy(&block[BLK_OFF_CODES], codes, code_bytes);

	/* The queue holds SLOT_COUNT entries and there are SLOT_COUNT slots, so
	 * a slot we hold always fits. */
	(void)k_msgq_put(&m_self.full_q, &slot_index, K_NO_WAIT);
	(void)k_work_submit(&m_self.work);
}

/* Work item: seals and delivers every queued block. */
static void deliver_work(struct k_work *work)
{
	uint8_t slot_index;

	ARG_UNUSED(work);

	while (k_msgq_get(&m_self.full_q, &slot_index, K_NO_WAIT) == 0) {
		uint8_t *block = m_self.slot[slot_index];
		size_t body = m_self.block_bytes - CRC_BYTES;

		sys_put_le32(crc32_ieee(block, body), &block[body]);

		if (m_self.config.sink.write(m_self.config.sink.ctx, block, m_self.block_bytes) ==
		    0) {
			atomic_inc(&m_self.blocks_sent);
		} else {
			count_lost();
		}
		(void)k_msgq_put(&m_self.free_q, &slot_index, K_NO_WAIT);
	}
}

static void reset_slots(void)
{
	k_msgq_purge(&m_self.free_q);
	k_msgq_purge(&m_self.full_q);
	for (uint8_t i = 0; i < SLOT_COUNT; i++) {
		(void)k_msgq_put(&m_self.free_q, &i, K_NO_WAIT);
	}
}

int analog_stream_start(const struct analog_stream_config *config)
{
	struct analog_source_info info;
	uint8_t header[ANALOG_START_HEADER_BYTES];
	int err;

	if (m_self.running) {
		LOG_ERR("%s: a session is already running", __func__);
		return -EBUSY;
	}
	if (config->backend == NULL || config->sink.write == NULL || config->cycles_per_sec == 0U ||
	    config->params.channel_count == 0U ||
	    config->params.channel_count > ANALOG_MAX_CHANNELS) {
		LOG_ERR("%s: bad configuration", __func__);
		return -EINVAL;
	}

	m_self.config = *config;
	m_self.block_bytes = ANALOG_BLOCK_BYTES(config->params.channel_count);
	m_self.next_index = 0U;
	memset(&m_self.marker, 0, sizeof(m_self.marker));
	atomic_set(&m_self.blocks_sent, 0);
	atomic_set(&m_self.blocks_lost, 0);
	reset_slots();

	err = config->backend->prepare(&config->params, &info);
	if (err != 0) {
		LOG_ERR("%s: the backend could not prepare (%d)", __func__, err);
		return err;
	}
	if (info.sample_rate_hz == 0U) {
		LOG_ERR("%s: the backend reported a zero rate", __func__);
		config->backend->stop();
		return -EINVAL;
	}
	if (info.sample_rate_hz != config->params.sample_rate_hz) {
		LOG_WRN("%s: %u Hz asked, the hardware runs at %u Hz", __func__,
			config->params.sample_rate_hz, info.sample_rate_hz);
	}
	m_self.cycles_per_frame = MAX(1U, config->cycles_per_sec / info.sample_rate_hz);

	build_start_header(header, &info);
	err = config->sink.write(config->sink.ctx, header, sizeof(header));
	if (err != 0) {
		LOG_ERR("%s: the sink refused the start header (%d)", __func__, err);
		config->backend->stop();
		return err;
	}

	m_self.running = true;
	err = config->backend->start(on_block, NULL);
	if (err != 0) {
		LOG_ERR("%s: the backend could not start (%d)", __func__, err);
		m_self.running = false;
		config->backend->stop();
		return err;
	}
	return 0;
}

void analog_stream_stop(struct analog_stream_stats *stats)
{
	if (m_self.running) {
		m_self.config.backend->stop();
		m_self.running = false;
		/* Waits for a delivery already under way; the handler empties
		 * the queue before it returns. */
		(void)k_work_flush(&m_self.work, &m_self.work_sync);
		LOG_INF("%s: %u blocks sent, %u lost", __func__,
			(unsigned int)atomic_get(&m_self.blocks_sent),
			(unsigned int)atomic_get(&m_self.blocks_lost));
	}
	if (stats != NULL) {
		stats->blocks_sent = (uint32_t)atomic_get(&m_self.blocks_sent);
		stats->blocks_lost = (uint32_t)atomic_get(&m_self.blocks_lost);
	}
}

bool analog_stream_running(void)
{
	return m_self.running;
}

void analog_stream_marker_edge(bool level, uint32_t cycles)
{
	unsigned int key;

	if (!m_self.running) {
		return;
	}
	key = irq_lock();
	if (m_self.marker.edge_count == 0U) {
		m_self.marker.first_level = level;
		m_self.marker.first_cycles = cycles;
	}
	if (m_self.marker.edge_count < EVENT_COUNT_MAX) {
		m_self.marker.edge_count++;
	}
	irq_unlock(key);
}

static int analog_stream_init(void)
{
	k_msgq_init(&m_self.free_q, m_self.free_q_storage, sizeof(uint8_t), SLOT_COUNT);
	k_msgq_init(&m_self.full_q, m_self.full_q_storage, sizeof(uint8_t), SLOT_COUNT);
	k_work_init(&m_self.work, deliver_work);
	return 0;
}

SYS_INIT(analog_stream_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
