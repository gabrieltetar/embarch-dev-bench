/*
 * One analog capture session on the dev bench (dev-bench decisions 60, 64):
 * the study's analog tap starts it at the tap's rate and the tap's close ends
 * it; there is no other control command. The marker input is the board's
 * `embarch,marker-input` chosen node; a board without one captures without
 * marker edges.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "analog_capture.h"
#include "analog_stream.h"
#include "samples_port.h"

LOG_MODULE_REGISTER(analog_capture, LOG_LEVEL_INF);

#define MARKER_NODE DT_CHOSEN(embarch_marker_input)

#if DT_HAS_CHOSEN(embarch_marker_input)
static const struct gpio_dt_spec m_marker = GPIO_DT_SPEC_GET(MARKER_NODE, gpios);
static struct gpio_callback m_marker_cb;

static void marker_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Logical level: 1 while the marker is asserted (a pressed button on an
	 * active-low pin). */
	analog_stream_marker_edge(gpio_pin_get_dt(&m_marker) > 0, k_cycle_get_32());
}

static int marker_enable(void)
{
	int err;

	if (!gpio_is_ready_dt(&m_marker)) {
		LOG_ERR("%s: the marker GPIO is not ready", __func__);
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&m_marker, GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("%s: configuring the marker failed (%d)", __func__, err);
		return err;
	}
	gpio_init_callback(&m_marker_cb, marker_isr, BIT(m_marker.pin));
	err = gpio_add_callback(m_marker.port, &m_marker_cb);
	if (err != 0) {
		LOG_ERR("%s: adding the marker callback failed (%d)", __func__, err);
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&m_marker, GPIO_INT_EDGE_BOTH);
	if (err != 0) {
		LOG_ERR("%s: enabling the marker interrupt failed (%d)", __func__, err);
		(void)gpio_remove_callback(m_marker.port, &m_marker_cb);
	}
	return err;
}

static void marker_disable(void)
{
	(void)gpio_pin_interrupt_configure_dt(&m_marker, GPIO_INT_DISABLE);
	(void)gpio_remove_callback(m_marker.port, &m_marker_cb);
}
#else
static int marker_enable(void)
{
	return 0;
}

static void marker_disable(void)
{
}
#endif

int analog_capture_start(uint32_t sample_hz, uint8_t channel)
{
	struct analog_stream_config config = {
		.backend = &analog_source_backend,
		.sink = {.write = samples_port_write},
		.params = {.sample_rate_hz = sample_hz, .channel = channel, .channel_count = 1U},
		.cycles_per_sec = sys_clock_hw_cycles_per_sec(),
		.board = CONFIG_BOARD,
		.image_version = APP_FIRMWARE_VERSION,
	};
	int err = samples_port_reset();

	if (err != 0) {
		LOG_ERR("%s: no samples port (%d)", __func__, err);
		return err;
	}
	err = marker_enable();
	if (err != 0) {
		return err;
	}
	err = analog_stream_start(&config);
	if (err != 0) {
		marker_disable();
	}
	return err;
}

uint32_t analog_capture_stop(void)
{
	struct analog_stream_stats stats;
	bool was_running = analog_stream_running();

	analog_stream_stop(&stats);
	if (!was_running) {
		/* analog_stream keeps the last session's totals until the next
		 * start. A tap whose capture never started (or that a stop already
		 * ended) has lost nothing of its own, and the totals left over
		 * would be reported as its `StreamClose.dropped`. */
		return 0;
	}
	marker_disable();
	return stats.blocks_lost;
}
