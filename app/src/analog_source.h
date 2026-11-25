/*
 * The analog source: what the dev bench samples and how the samples reach
 * analog_stream.c. SoC-neutral, with one backend per SoC family; the only one
 * today is analog_source_nrf52.c, a stated, narrow exception to the
 * portable-APIs-only rule (dev-bench decision 60).
 *
 * N-channel-ready: the channel count is a parameter and a delivery is
 * `ANALOG_SAMPLES_PER_BLOCK` frames of `channel_count` interleaved codes. The
 * v1 backend samples one channel.
 */
#ifndef EMBARCH_ANALOG_SOURCE_H_
#define EMBARCH_ANALOG_SOURCE_H_

#include <stdint.h>

#include <zephyr/autoconf.h>

/* Frames per block, per channel. Fixed by the owner (EMB-469 decision 7). */
#define ANALOG_SAMPLES_PER_BLOCK 256U

/* The block's edge offset is one byte, so a block can hold at most 256 frames. */
_Static_assert(ANALOG_SAMPLES_PER_BLOCK <= 256U, "a block's marker edge offset is 8 bits");

/* Most channels a build carries; sizes every block buffer. */
#define ANALOG_MAX_CHANNELS CONFIG_EMBARCH_ANALOG_MAX_CHANNELS

/* The ADC reference, numbered as the start header carries it. */
enum analog_reference {
	ANALOG_REFERENCE_INTERNAL = 0,
	ANALOG_REFERENCE_VDD_DIV4 = 1,
};

/* What a session asks of a backend. */
struct analog_source_params {
	/* The rate the study asked for; the backend reports the one it can
	 * reach in `analog_source_info.sample_rate_hz`. */
	uint32_t sample_rate_hz;
	/* The study's channel index into the board's analog inputs. */
	uint8_t channel;
	uint8_t channel_count;
};

/* What a prepared backend reports, and what the start header carries. */
struct analog_source_info {
	/* The rate the hardware will really run at. */
	uint32_t sample_rate_hz;
	uint8_t resolution_bits;
	uint8_t gain_numerator;
	uint8_t gain_denominator;
	enum analog_reference reference;
	uint8_t channel_count;
	/* VDD measured once while preparing. */
	uint16_t vdd_mv;
	/* The input's devicetree label, NUL-terminated. */
	const char *input_name;
};

/* Delivers one block's worth of frames: ANALOG_SAMPLES_PER_BLOCK frames of
 * `channel_count` interleaved raw codes, valid only for the call. `cycles` is
 * k_cycle_get_32() when the last frame was taken. May run in interrupt
 * context. */
typedef void (*analog_source_block_cb)(void *ctx, const int16_t *codes, uint32_t cycles);

struct analog_source_backend {
	/* Claims the hardware, calibrates it, measures VDD and configures the
	 * input, without sampling yet. Returns 0 or a negative errno. */
	int (*prepare)(const struct analog_source_params *params, struct analog_source_info *info);
	/* Starts sampling after a successful prepare; `cb` then runs once per
	 * block. Returns 0 or a negative errno. */
	int (*start)(analog_source_block_cb cb, void *ctx);
	/* Stops sampling and releases the hardware. No callback runs once it
	 * has returned. Safe after a prepare that was never started. */
	void (*stop)(void);
};

/* The backend this build has: nRF52's SAADC, or one whose prepare returns
 * -ENOTSUP. */
extern const struct analog_source_backend analog_source_backend;

#endif /* EMBARCH_ANALOG_SOURCE_H_ */
