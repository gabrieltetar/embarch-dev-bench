/*
 * The samples USB port, on the CDC ACM port labelled "EmbArch samples"
 * (cdc_acm_uart1, board overlay). Transmit only: a byte the host sends is read
 * and discarded.
 *
 * A producer (the analog stream's work item) puts whole blocks into a ring;
 * the UART interrupt drains it into the port's FIFO. The ring is single
 * producer, single consumer, so it needs no lock.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include "analog_stream.h"
#include "samples_port.h"

LOG_MODULE_REGISTER(samples_port, LOG_LEVEL_INF);

#define SAMPLES_PORT_NODE DT_NODELABEL(cdc_acm_uart1)

/* Scratch for discarding what the host sends. */
#define RX_DISCARD_BYTES 16U

RING_BUF_DECLARE(m_tx_ring, CONFIG_EMBARCH_SAMPLES_PORT_RING_BLOCKS *ANALOG_BLOCK_BYTES(ANALOG_MAX_CHANNELS));

static const struct device *const m_port = DEVICE_DT_GET(SAMPLES_PORT_NODE);
static bool m_irq_ready;

static void port_irq(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t discard[RX_DISCARD_BYTES];

			(void)uart_fifo_read(dev, discard, sizeof(discard));
		}
		if (uart_irq_tx_ready(dev)) {
			uint8_t *data;
			uint32_t len = ring_buf_get_claim(&m_tx_ring, &data, UINT32_MAX);
			int sent = 0;

			if (len == 0U) {
				uart_irq_tx_disable(dev);
				continue;
			}
			sent = uart_fifo_fill(dev, data, (int)len);
			(void)ring_buf_get_finish(&m_tx_ring, sent > 0 ? (uint32_t)sent : 0U);
			if (sent <= 0) {
				/* The port took nothing (host not listening): leave the
				 * rest for the next ready interrupt. */
				uart_irq_tx_disable(dev);
			}
		}
	}
}

static int ensure_irq(void)
{
	if (m_irq_ready) {
		return 0;
	}
	if (!device_is_ready(m_port)) {
		LOG_ERR("%s: the samples port is not ready", __func__);
		return -ENODEV;
	}
	uart_irq_callback_user_data_set(m_port, port_irq, NULL);
	uart_irq_rx_enable(m_port);
	m_irq_ready = true;
	return 0;
}

int samples_port_reset(void)
{
	int err = ensure_irq();

	if (err != 0) {
		return err;
	}
	uart_irq_tx_disable(m_port);
	ring_buf_reset(&m_tx_ring);
	return 0;
}

int samples_port_write(void *ctx, const uint8_t *data, size_t len)
{
	ARG_UNUSED(ctx);

	if (ring_buf_space_get(&m_tx_ring) < len) {
		return -ENOSPC;
	}
	(void)ring_buf_put(&m_tx_ring, data, (uint32_t)len);
	uart_irq_tx_enable(m_port);
	return 0;
}
