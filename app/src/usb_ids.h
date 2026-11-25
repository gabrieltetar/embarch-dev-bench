/* USB identity of an EmbArch device (embarch-dev-bench, native USB link).
 *
 * VID 0xEBAC is for local use, not a reserved or registered vendor ID. The PID
 * is two bytes: the high byte says what kind of EmbArch device this is (the
 * table below, one entry per device type, dev bench first), the low byte
 * tells apart two boards of the same type on one host. The serial number is
 * the chip's unique ID in hex, so the PID low byte is the last two hex digits
 * of the serial.
 *
 * Pure arithmetic with no Zephyr dependency, so native_sim tests can pin it.
 */
#ifndef EMBARCH_USB_IDS_H_
#define EMBARCH_USB_IDS_H_

#include <stddef.h>
#include <stdint.h>

#define EMBARCH_USB_VID 0xEBAC

/* The table of EmbArch device types: the PID's high byte. Add a type by
 * appending an entry; never renumber one, hosts match on the value. */
enum embarch_usb_device_type {
	EMBARCH_USB_DEVICE_TYPE_DEV_BENCH = 0x01,
};

/* The PID: `type` in the high byte, the last byte of the unique ID in the low
 * byte. `id_len` must be at least 1. */
static inline uint16_t embarch_usb_pid(enum embarch_usb_device_type type, const uint8_t *id,
				       size_t id_len)
{
	return (uint16_t)(((uint16_t)type << 8) | id[id_len - 1U]);
}

/* Writes `id` as uppercase hex into `out` (2 * id_len characters plus a NUL,
 * so `out_size` must be at least 2 * id_len + 1) and returns the number of
 * characters, or 0 when `out` is too small. This is the form of the USB serial
 * number string. */
size_t embarch_usb_serial_hex(const uint8_t *id, size_t id_len, char *out, size_t out_size);

#endif /* EMBARCH_USB_IDS_H_ */
