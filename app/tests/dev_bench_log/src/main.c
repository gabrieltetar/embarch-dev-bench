/* dev_bench_log.c's own tests (embarch-dev-bench/design.md §3 decision 38).
 *
 * What is worth testing here is not "does Zephyr's log subsystem work" but the
 * three behaviours this backend adds on top of it, each of which is a promise
 * something else depends on:
 *   - records produced before Core's `Hello` are *held*, then flushed in order
 *     (this is the whole reason a reboot is visible from Core's side)
 *   - a line too long for `DBM_MAX_LOG_LINE_LEN` is marked, never silently cut
 *   - a backlog overflow drops the oldest and *reports the count*
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/ztest.h>

#include "dev_bench_log.h"
#include "serial_protocol.h"

/* src/app_module.c -- a module registered under DEV_BENCH_LOG_APP_MODULE. */
void app_module_emit_inf(void);
void app_module_emit_dbg(void);

LOG_MODULE_REGISTER(dbm_log_test, LOG_LEVEL_INF);

#define CAPTURE_DEPTH 32

static char captured[CAPTURE_DEPTH][DBM_MAX_LOG_LINE_LEN + 1];
static uint32_t captured_len;
static bool captured_panic[CAPTURE_DEPTH];

static void capture_sink(const char *line, bool panic)
{
	if (captured_len >= CAPTURE_DEPTH) {
		return;
	}
	strcpy(captured[captured_len], line);
	captured_panic[captured_len] = panic;
	captured_len++;
}

static void capture_reset(void)
{
	captured_len = 0;
	memset(captured, 0, sizeof(captured));
}

/* Hands every pending record to the backend, synchronously -- prj.conf turns
 * the log processing thread off so this, not a sleep, is what makes the tests
 * deterministic. */
static void drain(void)
{
	while (log_process()) {
	}
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Whatever a previous test left pending or held goes into the capture
	 * and is then thrown away, so each test starts from a genuinely empty
	 * backlog rather than one holding a neighbour's records. */
	drain();
	dev_bench_log_set_sink(capture_sink);
	drain();
	dev_bench_log_set_sink(NULL);
	capture_reset();

	/* Every test starts fully audible, so a test that is not *about*
	 * filtering cannot be affected by it — and so a test that changes the
	 * level cannot silently set up the next one (design.md §3 decision 39).
	 * The filtering tests below each set the level they mean to exercise. */
	dev_bench_log_set_level(DBM_LOG_LEVEL_DBG);
}

ZTEST_SUITE(dev_bench_log, NULL, NULL, before, NULL, NULL);

ZTEST(dev_bench_log, test_records_are_held_until_a_sink_is_installed)
{
	LOG_INF("first line");
	LOG_INF("second line");
	drain();

	zassert_equal(captured_len, 0,
		      "records must be held, not dropped, while no sink is installed");

	dev_bench_log_set_sink(capture_sink);

	zassert_equal(captured_len, 2, "both held lines should flush, got %u", captured_len);
	/* In order, oldest first -- a log read out of order is worse than no
	 * log, since it invents a sequence of events that never happened. */
	zassert_not_null(strstr(captured[0], "first line"), "got '%s'", captured[0]);
	zassert_not_null(strstr(captured[1], "second line"), "got '%s'", captured[1]);
	/* log_output's own level/source prefix rides along, which is what Core's
	 * level classifier reads. */
	zassert_true(strncmp(captured[0], "<inf> dbm_log_test:", 19) == 0, "got '%s'",
		     captured[0]);
	zassert_false(captured_panic[0], "not a fatal-path line");
}

ZTEST(dev_bench_log, test_records_go_straight_out_once_a_sink_is_installed)
{
	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_WRN("live line");
	drain();

	zassert_equal(captured_len, 1, "expected exactly one line, got %u", captured_len);
	zassert_not_null(strstr(captured[0], "live line"), "got '%s'", captured[0]);
	zassert_true(strncmp(captured[0], "<wrn> ", 6) == 0,
		     "the level prefix is what Core classifies on; got '%s'", captured[0]);
}

ZTEST(dev_bench_log, test_an_overlong_line_is_marked_not_silently_cut)
{
	char long_msg[DBM_MAX_LOG_LINE_LEN * 2];

	memset(long_msg, 'x', sizeof(long_msg) - 1);
	long_msg[sizeof(long_msg) - 1] = '\0';

	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_INF("%s", long_msg);
	drain();

	zassert_equal(captured_len, 1, "expected one line, got %u", captured_len);
	zassert_equal(strlen(captured[0]), DBM_MAX_LOG_LINE_LEN,
		      "a line must fill, and never exceed, LogLine's own capacity (got %u)",
		      (unsigned int)strlen(captured[0]));
	zassert_not_null(strstr(captured[0], " [cut]"),
			 "truncation must be visible in the line itself; got '%s'",
			 captured[0]);
}

ZTEST(dev_bench_log, test_backlog_overflow_drops_the_oldest_and_reports_the_count)
{
	const uint32_t extra = 3;

	for (uint32_t i = 0; i < DEV_BENCH_LOG_BACKLOG_DEPTH + extra; i++) {
		LOG_INF("held %u", i);
	}
	drain();

	zassert_equal(dev_bench_log_backlog_dropped(), extra,
		      "expected %u drops, got %u", extra, dev_bench_log_backlog_dropped());

	dev_bench_log_set_sink(capture_sink);

	/* The drop report comes first, then the DEPTH newest lines. */
	zassert_equal(captured_len, DEV_BENCH_LOG_BACKLOG_DEPTH + 1,
		      "expected a drop report plus %u held lines, got %u",
		      DEV_BENCH_LOG_BACKLOG_DEPTH, captured_len);
	zassert_not_null(strstr(captured[0], "3 log line(s) dropped"), "got '%s'",
			 captured[0]);
	zassert_true(strncmp(captured[0], "<wrn> ", 6) == 0,
		     "a drop report has no log record behind it, so it carries its own "
		     "level prefix; got '%s'",
		     captured[0]);
	/* Oldest-dropped, not newest-dropped: what a bench said most recently is
	 * what a reader needs. `held 0`..`held 2` are the ones gone. */
	zassert_not_null(strstr(captured[1], "held 3"), "got '%s'", captured[1]);
	zassert_not_null(strstr(captured[DEV_BENCH_LOG_BACKLOG_DEPTH], "held 10"), "got '%s'",
			 captured[DEV_BENCH_LOG_BACKLOG_DEPTH]);
	zassert_equal(dev_bench_log_backlog_dropped(), 0, "the count resets at flush");
}

ZTEST(dev_bench_log, test_a_multi_line_record_becomes_one_line_each)
{
	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_INF("top%cbottom", '\n');
	drain();

	zassert_equal(captured_len, 2,
		      "an embedded newline must split the record, so Core's line-oriented "
		      "debug file stays one record per line (got %u)",
		      captured_len);
	zassert_not_null(strstr(captured[0], "top"), "got '%s'", captured[0]);
	zassert_equal(strcmp(captured[1], "bottom"), 0, "got '%s'", captured[1]);
}

ZTEST(dev_bench_log, test_control_bytes_are_replaced_rather_than_forwarded)
{
	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_INF("a%cb", '\t');
	drain();

	zassert_equal(captured_len, 1, "expected one line, got %u", captured_len);
	zassert_not_null(strstr(captured[0], "a.b"), "got '%s'", captured[0]);
}

/* ---- per-study verbosity (design.md §3 decision 39) -------------------- */

ZTEST(dev_bench_log, test_the_idle_level_is_quiet_but_never_silent)
{
	dev_bench_log_set_level(DEV_BENCH_LOG_BOOT_LEVEL);
	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_INF("chatty subsystem");
	LOG_WRN("something worth knowing");
	drain();

	/* This is what "quiet" has to mean for the default to be defensible: an
	 * ordinary module's info line is dropped, and its warning is not. */
	zassert_equal(captured_len, 1, "expected only the warning, got %u lines", captured_len);
	zassert_not_null(strstr(captured[0], "something worth knowing"), "got '%s'",
			 captured[0]);
}

ZTEST(dev_bench_log, test_the_app_module_keeps_info_at_the_idle_level)
{
	dev_bench_log_set_level(DEV_BENCH_LOG_BOOT_LEVEL);
	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	app_module_emit_inf();
	drain();

	/* The floor dev_bench_log.h argues for: the bench going quiet must not
	 * mean the bench stopping saying what it is doing, or the boot record
	 * and handshake diagnostics decision 38 exists for would vanish the
	 * moment a study asked for less. */
	zassert_equal(captured_len, 1,
		      "the app's own module must keep INF while others are held at WRN "
		      "(got %u lines)",
		      captured_len);
	zassert_not_null(strstr(captured[0], "app module speaking"), "got '%s'", captured[0]);
}

ZTEST(dev_bench_log, test_a_study_can_ask_for_info_and_get_it)
{
	uint8_t reached = dev_bench_log_set_level(DBM_LOG_LEVEL_INF);

	zassert_equal(reached, DBM_LOG_LEVEL_INF,
		      "this build compiles in DBG, so INF must be reachable (got %u)",
		      (unsigned int)reached);

	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_INF("now audible");
	drain();

	zassert_equal(captured_len, 1, "expected the info line, got %u", captured_len);
	zassert_not_null(strstr(captured[0], "now audible"), "got '%s'", captured[0]);
}

ZTEST(dev_bench_log, test_off_is_taken_literally_including_the_app_module)
{
	uint8_t reached = dev_bench_log_set_level(DBM_LOG_LEVEL_OFF);

	zassert_equal(reached, DBM_LOG_LEVEL_OFF, "Off must be reachable (got %u)",
		      (unsigned int)reached);

	dev_bench_log_set_sink(capture_sink);
	capture_reset();

	LOG_ERR("an error nobody will hear");
	app_module_emit_inf();
	drain();

	/* Off is the one setting that overrides the app-module floor. A study
	 * that asks for a completely clear link gets one -- and gives up the
	 * fatal-error dump to get it, which is why it is not the default. */
	zassert_equal(captured_len, 0, "Off must silence everything, got %u lines",
		      captured_len);
}

ZTEST(dev_bench_log, test_set_level_reports_the_level_it_actually_reached)
{
	/* Every source in this build compiles at DBG, so every request is
	 * honoured exactly. The value of the return is the case where that
	 * isn't true -- a build compiled at INF asked for DBG -- which cannot be
	 * constructed here without a second build, so what this pins is that the
	 * app-module floor does not leak into the answer: a request for WRN must
	 * report WRN, not the INF the app module was held at. */
	zassert_equal(dev_bench_log_set_level(DBM_LOG_LEVEL_DBG), DBM_LOG_LEVEL_DBG,
		      "DBG request");
	zassert_equal(dev_bench_log_set_level(DBM_LOG_LEVEL_WRN), DBM_LOG_LEVEL_WRN,
		      "a WRN request must not report back the app module's INF floor");
	zassert_equal(dev_bench_log_set_level(DBM_LOG_LEVEL_ERR), DBM_LOG_LEVEL_ERR,
		      "ERR request");
}
