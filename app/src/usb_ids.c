#include "usb_ids.h"

#define HEX_DIGITS_PER_BYTE 2U
#define NIBBLE_BITS 4U
#define NIBBLE_MASK 0xFU

size_t embarch_usb_serial_hex(const uint8_t *id, size_t id_len, char *out, size_t out_size)
{
	static const char hex[] = "0123456789ABCDEF";
	size_t chars = id_len * HEX_DIGITS_PER_BYTE;

	if (out_size < chars + 1U) {
		return 0;
	}

	for (size_t i = 0; i < id_len; i++) {
		out[i * HEX_DIGITS_PER_BYTE] = hex[id[i] >> NIBBLE_BITS];
		out[i * HEX_DIGITS_PER_BYTE + 1U] = hex[id[i] & NIBBLE_MASK];
	}
	out[chars] = '\0';

	return chars;
}
