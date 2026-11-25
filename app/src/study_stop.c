/* See study_stop.h. */

#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "serial_protocol.h"
#include "study_stop.h"

/* The stuffed bytes of a `StudyStop`: a code byte, the tag and the reason.
 * A longer frame is not one, so it is discarded as it arrives. */
#define STOP_FRAME_MAX_BYTES 4U

#define FRAME_DELIMITER 0x00U

static struct {
	study_stop_byte_source source;
	bool stopped;
	uint8_t reason;
	uint8_t frame[STOP_FRAME_MAX_BYTES];
	size_t frame_len;
	/* The frame in progress already overran `frame`; drop it at its
	 * delimiter rather than decode a truncated one. */
	bool frame_overran;
} m_self;

void study_stop_begin(study_stop_byte_source source)
{
	m_self.source = source;
	m_self.stopped = false;
	m_self.reason = 0;
	m_self.frame_len = 0;
	m_self.frame_overran = false;
}

void study_stop_end(void)
{
	m_self.source = NULL;
}

static void end_of_frame(void)
{
	uint8_t reason;

	if (!m_self.frame_overran && m_self.frame_len > 0 &&
	    dbm_decode_study_stop(m_self.frame, m_self.frame_len, &reason) == 0) {
		m_self.stopped = true;
		m_self.reason = reason;
	}
	m_self.frame_len = 0;
	m_self.frame_overran = false;
}

bool study_stop_poll(void)
{
	uint8_t byte;

	while (!m_self.stopped && m_self.source != NULL && m_self.source(&byte) == 0) {
		if (byte == FRAME_DELIMITER) {
			end_of_frame();
		} else if (m_self.frame_len < ARRAY_SIZE(m_self.frame)) {
			m_self.frame[m_self.frame_len++] = byte;
		} else {
			m_self.frame_overran = true;
		}
	}
	return m_self.stopped;
}

uint8_t study_stop_reason(void)
{
	return m_self.reason;
}

bool study_stop_sleep_ms(uint32_t ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)ms;

	while (!study_stop_poll()) {
		int64_t left = deadline - k_uptime_get();

		if (left <= 0) {
			return false;
		}
		k_sleep(K_MSEC(MIN(left, (int64_t)STUDY_STOP_POLL_SLICE_MS)));
	}
	return true;
}
