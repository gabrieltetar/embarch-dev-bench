/*
 * The analog source on nRF52: nrfx SAADC in advanced mode on its internal
 * timer, taken through nrfx directly. Zephyr's ADC driver does not expose the
 * internal timer and must not also claim the SAADC, so the board overlay
 * disables `&adc` (dev-bench decision 60: the one stated exception to
 * portable-APIs-only).
 *
 * After nrfx's nrfx_saadc/advanced_non_blocking_internal_timer sample:
 * continuous, double-buffered, one channel (the internal timer needs a single
 * input), 12-bit, gain 1/4 against a VDD/4 reference so full scale is VDD,
 * offset calibration at every session start, VDD measured once for the
 * header. The input comes from the board's `embarch,analog-input` chosen node.
 *
 * Written from the nrfx sample and its header; not run on hardware by the
 * change that added it.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <nrfx_saadc.h>

#include "analog_source.h"

LOG_MODULE_REGISTER(analog_nrf52, LOG_LEVEL_INF);

#define ANALOG_NODE DT_CHOSEN(embarch_analog_input)
#define ADC_NODE DT_NODELABEL(adc)

BUILD_ASSERT(DT_HAS_CHOSEN(embarch_analog_input),
	     "the board overlay must choose an embarch,analog-input node");
BUILD_ASSERT(ANALOG_MAX_CHANNELS >= 1, "at least one channel");

/* The data channel: 12-bit, gain 1/4, reference VDD/4. */
#define RESOLUTION NRF_SAADC_RESOLUTION_12BIT
#define RESOLUTION_BITS 12U
#define GAIN_NUMERATOR 1U
#define GAIN_DENOMINATOR 4U
#define DATA_ACQ_TIME NRF_SAADC_ACQTIME_10US

/* The internal timer runs at 16 MHz (SAMPLERATE register, nrfx sample) with a
 * compare value of 80 to 2047. */
#define TIMER_HZ 16000000U
#define TIMER_CC_MIN 80U
#define TIMER_CC_MAX 2047U

/* Interrupt priority and number of the SAADC, from the (disabled) &adc node:
 * the same numbers Zephyr's ADC driver would have used. */
#define SAADC_IRQN DT_IRQN(ADC_NODE)
#define SAADC_IRQ_PRIORITY DT_IRQ(ADC_NODE, priority)

/* The VDD reading: the VDD input, gain 1/6 against the internal 0.6 V
 * reference, so full scale (4096 codes in single-ended 12-bit) is 3.6 V. */
#define VDD_FULL_SCALE_MV 3600U
#define VDD_CODES 4096U

/* How long stop() waits for the SAADC to report FINISHED after an abort. */
#define STOP_TIMEOUT_MS 100

/* All module state in one place; the SAADC events run in interrupt context
 * and read what prepare()/start() wrote before sampling began. */
static struct {
	nrf_saadc_value_t buffer[2][ANALOG_SAMPLES_PER_BLOCK];
	uint8_t next_buffer;
	analog_source_block_cb cb;
	void *cb_ctx;
	struct k_sem finished;
	bool claimed;
	/* Sampling has been triggered, so an abort ends in FINISHED. */
	bool started;
	bool delivering;
} m_self;

static void data_handler(nrfx_saadc_evt_t const *event)
{
	switch (event->type) {
	case NRFX_SAADC_EVT_BUF_REQ:
		/* The buffer the peripheral just released is free again: it
		 * was copied out in its DONE event, which precedes this one. */
		(void)nrfx_saadc_buffer_set(m_self.buffer[m_self.next_buffer],
					    ANALOG_SAMPLES_PER_BLOCK);
		m_self.next_buffer ^= 1U;
		break;
	case NRFX_SAADC_EVT_DONE:
		if (m_self.delivering && m_self.cb != NULL) {
			m_self.cb(m_self.cb_ctx, (const int16_t *)event->data.done.p_buffer,
				  k_cycle_get_32());
		}
		break;
	case NRFX_SAADC_EVT_FINISHED:
		k_sem_give(&m_self.finished);
		break;
	default:
		break;
	}
}

/* Reads VDD once, blocking, with the SAADC initialised and calibrated.
 * Returns millivolts or a negative errno. */
static int measure_vdd_mv(void)
{
	nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_VDD, 0);
	nrf_saadc_value_t code = 0;
	int err;

	channel.channel_config.gain = NRF_SAADC_GAIN1_6;
	channel.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;

	err = nrfx_saadc_channel_config(&channel);
	if (err != 0) {
		LOG_ERR("%s: channel config failed (%d)", __func__, err);
		return -EIO;
	}
	err = nrfx_saadc_simple_mode_set(nrfx_saadc_channels_configured_get(), RESOLUTION,
					 NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
	if (err != 0) {
		LOG_ERR("%s: simple mode failed (%d)", __func__, err);
		return -EIO;
	}
	err = nrfx_saadc_buffer_set(&code, 1);
	if (err != 0) {
		LOG_ERR("%s: buffer refused (%d)", __func__, err);
		return -EIO;
	}
	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("%s: conversion failed (%d)", __func__, err);
		return -EIO;
	}
	/* Deliberately not clamped: a negative code is an offset error worth
	 * seeing as a low VDD, not hiding. */
	return (int)(((int32_t)code * (int32_t)VDD_FULL_SCALE_MV) / (int32_t)VDD_CODES);
}

static int configure_data_channel(void)
{
	nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(
		NRF_SAADC_INPUT_AIN0 + DT_PROP(ANALOG_NODE, saadc_input), 0);
	int err;

	channel.channel_config.gain = NRF_SAADC_GAIN1_4;
	channel.channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
	channel.channel_config.acq_time = DATA_ACQ_TIME;

	err = nrfx_saadc_channels_config(&channel, 1);
	if (err != 0) {
		LOG_ERR("%s: channel config failed (%d)", __func__, err);
		return -EIO;
	}
	return 0;
}

static void release(void)
{
	if (m_self.claimed) {
		nrfx_saadc_uninit();
		m_self.claimed = false;
	}
}

static int nrf52_prepare(const struct analog_source_params *params, struct analog_source_info *info)
{
	uint32_t cc;
	int vdd_mv;
	int err;

	if (params->channel != 0U || params->channel_count != 1U) {
		LOG_ERR("%s: this backend samples channel 0 only (asked for %u of %u)", __func__,
			params->channel, params->channel_count);
		return -EINVAL;
	}
	if (params->sample_rate_hz == 0U) {
		return -EINVAL;
	}
	/* Round to the nearest compare value; the header carries the rate it gives. */
	cc = (TIMER_HZ + params->sample_rate_hz / 2U) / params->sample_rate_hz;
	if (cc < TIMER_CC_MIN || cc > TIMER_CC_MAX) {
		LOG_ERR("%s: %u Hz is outside the internal timer's %u to %u Hz", __func__,
			params->sample_rate_hz, TIMER_HZ / TIMER_CC_MAX, TIMER_HZ / TIMER_CC_MIN);
		return -EINVAL;
	}
	if (m_self.claimed) {
		return -EBUSY;
	}

	IRQ_CONNECT(SAADC_IRQN, SAADC_IRQ_PRIORITY, nrfx_isr, nrfx_saadc_irq_handler, 0);

	err = nrfx_saadc_init(SAADC_IRQ_PRIORITY);
	if (err != 0) {
		LOG_ERR("%s: nrfx_saadc_init failed (%d)", __func__, err);
		return -EIO;
	}
	m_self.claimed = true;
	k_sem_init(&m_self.finished, 0, 1);
	m_self.next_buffer = 1U;

	/* VDD first, on its own channel; then the data channel replaces it. */
	vdd_mv = measure_vdd_mv();
	if (vdd_mv < 0) {
		/* Survivable: the samples are raw codes and need no VDD; only a
		 * conversion to volts does, and a header VDD of 0 says it was
		 * not measured. The data channel set below replaces whatever the
		 * failed reading left configured. */
		LOG_WRN("%s: measuring VDD failed (%d); the header says VDD unmeasured", __func__,
			vdd_mv);
		nrfx_saadc_abort();
		vdd_mv = 0;
	}

	err = configure_data_channel();
	if (err != 0) {
		release();
		return err;
	}

	nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;

	adv.internal_timer_cc = (uint16_t)cc;
	adv.start_on_end = true;
	err = nrfx_saadc_advanced_mode_set(nrfx_saadc_channels_configured_get(), RESOLUTION, &adv,
					   data_handler);
	if (err != 0) {
		LOG_ERR("%s: advanced mode failed (%d)", __func__, err);
		release();
		return -EIO;
	}

	/* Offset calibration at each session start, blocking: nothing samples
	 * yet, so waiting for it costs only the start's latency. */
	err = nrfx_saadc_offset_calibrate(NULL);
	if (err != 0) {
		LOG_ERR("%s: offset calibration failed (%d)", __func__, err);
		release();
		return -EIO;
	}

	err = nrfx_saadc_buffer_set(m_self.buffer[0], ANALOG_SAMPLES_PER_BLOCK);
	if (err != 0) {
		LOG_ERR("%s: first buffer refused (%d)", __func__, err);
		release();
		return -EIO;
	}

	info->sample_rate_hz = TIMER_HZ / cc;
	info->resolution_bits = RESOLUTION_BITS;
	info->gain_numerator = GAIN_NUMERATOR;
	info->gain_denominator = GAIN_DENOMINATOR;
	info->reference = ANALOG_REFERENCE_VDD_DIV4;
	info->channel_count = 1U;
	info->vdd_mv = (uint16_t)CLAMP(vdd_mv, 0, UINT16_MAX);
	info->input_name = DT_PROP(ANALOG_NODE, label);
	return 0;
}

static int nrf52_start(analog_source_block_cb cb, void *ctx)
{
	int err;

	if (!m_self.claimed) {
		return -EINVAL;
	}
	m_self.cb = cb;
	m_self.cb_ctx = ctx;
	m_self.delivering = true;

	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("%s: triggering sampling failed (%d)", __func__, err);
		m_self.delivering = false;
		return -EIO;
	}
	m_self.started = true;
	return 0;
}

static void nrf52_stop(void)
{
	if (!m_self.claimed) {
		return;
	}
	m_self.delivering = false;
	nrfx_saadc_abort();
	if (m_self.started &&
	    k_sem_take(&m_self.finished, K_MSEC(STOP_TIMEOUT_MS)) != 0) {
		LOG_WRN("%s: the SAADC did not report FINISHED within %d ms", __func__,
			STOP_TIMEOUT_MS);
	}
	release();
	m_self.started = false;
	m_self.cb = NULL;
}

const struct analog_source_backend analog_source_backend = {
	.prepare = nrf52_prepare,
	.start = nrf52_start,
	.stop = nrf52_stop,
};
