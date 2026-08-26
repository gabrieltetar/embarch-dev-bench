/* See dev_bench_log.h for what this is and why it exists (design.md §3
 * decisions 7 and 38). */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "dev_bench_log.h"
#include "serial_protocol.h" /* DBM_MAX_LOG_LINE_LEN -- one definition, not a third copy */

#ifdef CONFIG_LOG

#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_msg.h>
#include <zephyr/logging/log_output.h>

/* Appended in place of the tail of a line that did not fit. Deliberately not
 * "..." -- a log message can legitimately end in an ellipsis, and a reader who
 * cannot tell truncation from content is the failure mode design.md §3
 * decision 36 already spent a whole feature removing from the GATT
 * transcript. */
#define TRUNC_MARK " [cut]"

static dev_bench_log_sink_fn log_sink;

/* Set once by the log subsystem's own panic path and never cleared: from that
 * point the system is stopping and every line takes the lock-free route. */
static bool log_panic_mode;

/* The line currently being assembled out of log_output's chunks. Touched only
 * by the log processing thread (deferred mode, enforced in prj.conf) and by
 * the fault context after `log_panic_mode` is set -- never concurrently, since
 * nothing else is scheduled once the fatal path runs. */
static char cur_line[DBM_MAX_LOG_LINE_LEN + 1];
static size_t cur_len;
static bool cur_truncated;

/* The pre-handshake hold (dev_bench_log.h). Written by the log thread and
 * drained by whichever thread calls dev_bench_log_set_sink, hence the
 * spinlock -- which is never held across a sink call, because a sink writes to
 * the UART and blocks. */
static char backlog[DEV_BENCH_LOG_BACKLOG_DEPTH][DBM_MAX_LOG_LINE_LEN + 1];
static uint32_t backlog_head;
static uint32_t backlog_count;
static uint32_t backlog_dropped;
static struct k_spinlock backlog_lock;

static void backlog_put(const char *text)
{
	k_spinlock_key_t key = k_spin_lock(&backlog_lock);

	strcpy(backlog[backlog_head], text);
	backlog_head = (backlog_head + 1) % DEV_BENCH_LOG_BACKLOG_DEPTH;
	if (backlog_count < DEV_BENCH_LOG_BACKLOG_DEPTH) {
		backlog_count++;
	} else {
		/* Full: this write overwrote the oldest held line. */
		backlog_dropped++;
	}
	k_spin_unlock(&backlog_lock, key);
}

/* Pops the oldest held line into `out` (which must be at least
 * DBM_MAX_LOG_LINE_LEN + 1 bytes). Returns false when the backlog is empty.
 * One line at a time, rather than snapshotting the whole ring, so the caller
 * never puts a kilobyte of it on its stack. */
static bool backlog_pop(char *out)
{
	k_spinlock_key_t key = k_spin_lock(&backlog_lock);

	if (backlog_count == 0) {
		k_spin_unlock(&backlog_lock, key);
		return false;
	}
	uint32_t oldest =
		(backlog_head + DEV_BENCH_LOG_BACKLOG_DEPTH - backlog_count) %
		DEV_BENCH_LOG_BACKLOG_DEPTH;

	strcpy(out, backlog[oldest]);
	backlog_count--;
	k_spin_unlock(&backlog_lock, key);
	return true;
}

/* One finished line, out to wherever it can go right now. */
static void emit_line(const char *text)
{
	dev_bench_log_sink_fn sink = log_sink;

	if (sink != NULL) {
		sink(text, log_panic_mode);
	} else if (!log_panic_mode) {
		/* No sink and not panicking: hold it for the handshake. In
		 * panic mode there is no handshake coming, so a held line
		 * would only be a line nobody ever reads. */
		backlog_put(text);
	}
}

/* log_output's sink. Accumulates into `cur_line`, flushing at every '\n' so a
 * multi-line record (a LOG_HEXDUMP, most of all) becomes one `LogLine` per
 * line rather than one unreadable blob.
 *
 * Any other byte outside printable ASCII is replaced rather than passed
 * through: COBS carries arbitrary bytes perfectly well, but Core appends each
 * line to a line-oriented debug file, and a stray control byte there turns one
 * record into two (or truncates the file's parser). Log messages are ASCII in
 * practice, so this only ever fires on genuinely malformed input. */
static int line_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);

	for (size_t i = 0; i < length; i++) {
		char c = (char)data[i];

		if (c == '\n') {
			if (cur_len > 0) {
				cur_line[cur_len] = '\0';
				emit_line(cur_line);
			}
			cur_len = 0;
			cur_truncated = false;
			continue;
		}
		if (c < 0x20 || c > 0x7e) {
			c = '.';
		}
		if (cur_len < DBM_MAX_LOG_LINE_LEN) {
			cur_line[cur_len++] = c;
		} else if (!cur_truncated) {
			/* Overwrite the tail with the marker exactly once, then
			 * keep discarding the rest of this record's bytes. */
			cur_truncated = true;
			memcpy(&cur_line[DBM_MAX_LOG_LINE_LEN - (sizeof(TRUNC_MARK) - 1)],
			       TRUNC_MARK, sizeof(TRUNC_MARK) - 1);
		}
	}
	return (int)length;
}

/* log_output stages formatted bytes here before calling line_out. Small on
 * purpose: it is a chunking buffer, not a line buffer -- `cur_line` above is
 * where a whole line accumulates. */
static uint8_t out_buf[32];

LOG_OUTPUT_DEFINE(dev_bench_log_output, line_out, out_buf, sizeof(out_buf));

static void dev_bench_log_process(const struct log_backend *const backend,
				 union log_msg_generic *msg)
{
	ARG_UNUSED(backend);

	/* LEVEL, and deliberately no TIMESTAMP: the whole line budget is
	 * DBM_MAX_LOG_LINE_LEN (128) and log_output's timestamp costs 19 of it,
	 * while Core stamps every line's arrival into the debug file on the same
	 * clock as every other Core-mediated record. Ordering -- which is what a
	 * log is actually read for -- is preserved either way, since deferred
	 * processing hands records to a backend in the order they were made.
	 *
	 * CRLF_NONE because line_out flushes on '\n' itself: without it every
	 * record would end in a '\r' this backend would then have to strip. */
	log_output_msg_process(&dev_bench_log_output, &msg->log,
			       LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_CRLF_NONE);
	log_output_flush(&dev_bench_log_output);

	/* End of record. Whatever is still buffered is this record's last (or
	 * only) line -- log_output emits no terminator under CRLF_NONE, so
	 * this, not a newline, is what ends a line. */
	if (cur_len > 0) {
		cur_line[cur_len] = '\0';
		emit_line(cur_line);
		cur_len = 0;
		cur_truncated = false;
	}
}

static void dev_bench_log_dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	/* Reported, never swallowed -- the same discipline the GATT
	 * transcript's own drop counter follows (design.md §3 decision 36).
	 * Hand-prefixed `<wrn>` so Core's level classifier reads it as the
	 * warning it is, since this line has no log record behind it. */
	char note[DBM_MAX_LOG_LINE_LEN + 1];

	snprintk(note, sizeof(note),
		 "<wrn> log: %u log record(s) dropped before this backend saw them",
		 (unsigned int)cnt);
	emit_line(note);
}

static void dev_bench_log_panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	/* From here the log subsystem processes synchronously in the faulting
	 * context. `emit_line` reads this to tell the sink to skip its mutex. */
	log_panic_mode = true;
}

static const struct log_backend_api dev_bench_log_backend_api = {
	.process = dev_bench_log_process,
	.dropped = dev_bench_log_dropped,
	.panic = dev_bench_log_panic,
};

/* autostart: true -- the backend must be live from the first record, long
 * before any sink exists, precisely so boot-time records reach the backlog. */
LOG_BACKEND_DEFINE(dev_bench_log_backend, dev_bench_log_backend_api, true);

void dev_bench_log_set_sink(dev_bench_log_sink_fn sink)
{
	if (sink == NULL) {
		log_sink = NULL;
		return;
	}

	/* Installed *before* the flush, not after: a record produced by the log
	 * thread while this drains then goes straight out rather than into a
	 * backlog nobody will read again. The cost is that it may interleave
	 * with the held lines; the sink serializes the two writers, so nothing
	 * is lost or corrupted, only ordered by arrival. */
	log_sink = sink;

	k_spinlock_key_t key = k_spin_lock(&backlog_lock);
	uint32_t dropped = backlog_dropped;

	backlog_dropped = 0;
	k_spin_unlock(&backlog_lock, key);

	if (dropped > 0) {
		char note[DBM_MAX_LOG_LINE_LEN + 1];

		snprintk(note, sizeof(note),
			 "<wrn> log: %u log line(s) dropped before the link came up",
			 (unsigned int)dropped);
		sink(note, false);
	}

	char held[DBM_MAX_LOG_LINE_LEN + 1];

	while (backlog_pop(held)) {
		sink(held, false);
	}
}

uint32_t dev_bench_log_backlog_dropped(void)
{
	k_spinlock_key_t key = k_spin_lock(&backlog_lock);
	uint32_t dropped = backlog_dropped;

	k_spin_unlock(&backlog_lock, key);
	return dropped;
}

#else /* !CONFIG_LOG */

/* A CONFIG_LOG=n build still links: there is simply nothing to forward. Kept
 * rather than made conditional in CMakeLists so turning the logging subsystem
 * off (to measure its cost, or on a board too tight for it) stays a one-line
 * Kconfig change, not a build-file edit. */
void dev_bench_log_set_sink(dev_bench_log_sink_fn sink)
{
	ARG_UNUSED(sink);
}

uint32_t dev_bench_log_backlog_dropped(void)
{
	return 0;
}

#endif /* CONFIG_LOG */
