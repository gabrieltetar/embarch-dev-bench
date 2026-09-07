#include "scan_seen_mfg.h"

#include <string.h>

#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

void scan_seen_mfg_data_parse(struct scan_seen_mfg_data *out, const uint8_t *data,
			       uint8_t data_len)
{
	if (out->present) {
		return; /* first element for this address wins */
	}
	out->present = true;

	if (data_len < (uint8_t)sizeof(uint16_t)) {
		/* Too short to carry a company ID -- 0 or 1 payload byte is a legal
		 * (if unusual) AD element on the air, not a reason to read past it. */
		out->id_valid = false;
		out->total_len = data_len;
		out->stored_len = 0;
		return;
	}

	out->id_valid = true;
	/* Company ID is transmitted little-endian (Bluetooth Core Spec, Company
	 * Identifiers assignment). */
	out->company_id = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
	out->total_len = (uint8_t)(data_len - sizeof(uint16_t));
	out->stored_len = (uint8_t)MIN(out->total_len, SCAN_SEEN_MFG_PAYLOAD_MAX);
	memcpy(out->payload, data + sizeof(uint16_t), out->stored_len);
}

size_t scan_seen_mfg_data_render(char *buf, size_t buf_len, const struct scan_seen_mfg_data *mfg)
{
	size_t used;
	int written;

	if (buf_len == 0) {
		return 0;
	}

	if (!mfg->present) {
		written = snprintk(buf, buf_len, "(none advertised)");
		return (written < 0) ? 0 : (size_t)written;
	}
	if (!mfg->id_valid) {
		written = snprintk(buf, buf_len, "(malformed: %u byte(s), shorter than a company ID)",
				    (unsigned int)mfg->total_len);
		return (written < 0) ? 0 : (size_t)written;
	}

	written = snprintk(buf, buf_len, "company=0x%04X data=", (unsigned int)mfg->company_id);
	/* Every segment below checks its own write against the space actually left
	 * before trusting `written` -- `buf` is sized generously enough (see the
	 * call site) that none of this should ever fire, but a segment that
	 * *would* overrun stops appending rather than letting `used` run past
	 * `buf_len`, which is exactly the "post-hoc test of a write that already
	 * happened" shape `007` found in this file's sibling function. */
	if (written < 0 || (size_t)written >= buf_len) {
		return (written < 0) ? 0 : buf_len - 1;
	}
	used = (size_t)written;

	for (uint8_t i = 0; i < mfg->stored_len; i++) {
		written = snprintk(buf + used, buf_len - used, "%02X", (unsigned int)mfg->payload[i]);
		if (written < 0 || (size_t)written >= buf_len - used) {
			return used;
		}
		used += (size_t)written;
	}

	if (mfg->total_len > mfg->stored_len) {
		written = snprintk(buf + used, buf_len - used, " (+%u more byte(s), not stored)",
				    (unsigned int)(mfg->total_len - mfg->stored_len));
		if (written > 0 && (size_t)written < buf_len - used) {
			used += (size_t)written;
		}
	}

	return used;
}
