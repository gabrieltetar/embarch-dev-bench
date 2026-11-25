/* The samples USB port: the second CDC ACM port of the dev bench's composite
 * device (usb_device.c), carrying the analog stream's bytes to the host. */
#ifndef EMBARCH_SAMPLES_PORT_H_
#define EMBARCH_SAMPLES_PORT_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/autoconf.h>

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
/* Empties the transmit ring, so a session's start header is the first byte
 * the host reads. Returns 0 or a negative errno. */
int samples_port_reset(void);

/* Queues `len` bytes for transmission, all of them or none. Returns 0, or
 * -ENOSPC when the ring cannot take them whole. Never blocks. */
int samples_port_write(void *ctx, const uint8_t *data, size_t len);
#else
#include <errno.h>

/* Boards without the native USB device have no samples port. */
static inline int samples_port_reset(void)
{
	return -ENOTSUP;
}

static inline int samples_port_write(void *ctx, const uint8_t *data, size_t len)
{
	return -ENOTSUP;
}
#endif

#endif /* EMBARCH_SAMPLES_PORT_H_ */
