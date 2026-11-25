/* The dev bench's native USB device: one composite device with three CDC ACM
 * ports (control, samples, SMP), see usb_device.c. */
#ifndef EMBARCH_USB_DEVICE_H_
#define EMBARCH_USB_DEVICE_H_

#include <errno.h>

#include <zephyr/autoconf.h>

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
/* Builds the descriptors, registers the three CDC ACM ports and enables the
 * device. Returns 0 or a negative errno. */
int usb_device_start(void);
#else
/* Boards whose conf does not select the USB device stack have no native USB. */
static inline int usb_device_start(void)
{
	return -ENOTSUP;
}
#endif

#endif /* EMBARCH_USB_DEVICE_H_ */
