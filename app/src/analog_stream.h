/*
 * The analog stream: turns a backend's blocks of codes into the bytes of the
 * samples link (start header, then CRC-checked blocks) and hands them to a
 * sink. Pure framing and bookkeeping, no hardware, so a native_sim test drives
 * it with a fake backend and a fake sink.
 *
 * The byte layout is embarch-study-designer's src/analog.rs (interfaces/
 * analog.md), little-endian throughout; analog_stream.c does not redefine it,
 * it emits it, and the test compares it with that encoder's bytes.
 */
#ifndef EMBARCH_ANALOG_STREAM_H_
#define EMBARCH_ANALOG_STREAM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "analog_source.h"

/* Sizes of the fixed text fields of the start header. */
#define ANALOG_HEADER_INPUT_NAME_BYTES 16U
#define ANALOG_HEADER_BOARD_BYTES 24U
#define ANALOG_HEADER_IMAGE_VERSION_BYTES 16U

/* The start header, CRC included. */
#define ANALOG_START_HEADER_BYTES 75U

/* One block of `channels` channels: magic, index and event word, the codes,
 * then the CRC. */
#define ANALOG_BLOCK_BYTES(channels) (16U + 2U * (channels) * ANALOG_SAMPLES_PER_BLOCK)

/* Where the bytes go. `write` takes the whole buffer or none of it: 0 if
 * taken, a negative errno if not (the stream then counts the block lost and
 * the index shows the gap). It runs in a work item, never in an interrupt. */
struct analog_stream_sink {
	int (*write)(void *ctx, const uint8_t *data, size_t len);
	void *ctx;
};

struct analog_stream_config {
	const struct analog_source_backend *backend;
	struct analog_stream_sink sink;
	struct analog_source_params params;
	/* Cycles per second of the clock the backend stamps blocks with
	 * (sys_clock_hw_cycles_per_sec()); places a marker edge in a block. */
	uint32_t cycles_per_sec;
	/* Written to the header, truncated to their fields with a warning. */
	const char *board;
	const char *image_version;
};

struct analog_stream_stats {
	uint32_t blocks_sent;
	/* Blocks the stream could not deliver: no free slot, or the sink
	 * refused. Their samples are absent and the index skips them. */
	uint32_t blocks_lost;
};

/* Prepares the backend, writes the start header to the sink, then starts
 * sampling. Returns 0 or a negative errno; on failure nothing is running. */
int analog_stream_start(const struct analog_stream_config *config);

/* Stops sampling, delivers every block already taken and returns the totals.
 * Safe when nothing is running (the totals are then zero). */
void analog_stream_stop(struct analog_stream_stats *stats);

/* Whether a session is running. */
bool analog_stream_running(void);

/* Records a marker edge, `level` being the marker's level after it and
 * `cycles` the time it happened on the backend's clock. Callable from an
 * interrupt; ignored when nothing is running. */
void analog_stream_marker_edge(bool level, uint32_t cycles);

#endif /* EMBARCH_ANALOG_STREAM_H_ */
