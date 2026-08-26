/* Zephyr log backend that forwards this firmware's `CONFIG_LOG` output to
 * Core over the existing `LogLine` DevBenchMessage.
 *
 * embarch-dev-bench/design.md §3 decision 7 already routes dev-bench's own
 * log output through `LogLine` rather than raw text on the shared wire, and
 * §3 decision 38 is what finally turns `CONFIG_LOG` itself on: until then the
 * only things that reached Core were the handful of hand-written
 * `send_log_line()` diagnostics, and every `LOG_INF`/`LOG_ERR` in Zephyr's own
 * subsystems (BT host, drivers) plus every fatal-error dump was compiled out
 * entirely -- which is exactly why "did this bench reboot mid-study" was not a
 * question anything observable could answer (see main.c's handle_hello).
 *
 * The backend deliberately knows nothing about the UART, the message union, or
 * main.c: it formats a record into one bounded line and hands it to a sink
 * this header lets main.c install. That keeps the Zephyr-log glue linkable in
 * a ztest against a capturing sink (app/tests/dev_bench_log), and it is also
 * what implements the pre-handshake hold below.
 */
#ifndef EMBARCH_DEV_BENCH_LOG_H_
#define EMBARCH_DEV_BENCH_LOG_H_

#include <stdbool.h>
#include <stdint.h>

/* How many formatted lines are held while no sink is installed -- i.e. from
 * power-on until Core's first `Hello` (design.md §3 decision 38). Boot-time
 * records have nowhere to go before that: Core opens the serial port
 * per-study, so anything written to the wire beforehand is shifted out to
 * nobody. Holding them and flushing at handshake is what makes a reboot
 * visible from Core's side at all.
 *
 * Eight, not a rounder number: this is `8 * (DBM_MAX_LOG_LINE_LEN + 1)` = ~1
 * KB of static RAM on a board whose SRAM has already overflowed twice at link
 * time (design.md §3 decisions 27 and the tx_scratch consolidation in
 * main.c), and eight lines is enough for the boot record plus whatever a
 * subsystem says on its way up. Overflow drops the *oldest* and is counted,
 * then reported as its own line at flush time -- never silently.
 */
#define DEV_BENCH_LOG_BACKLOG_DEPTH 8

/* Installed by main.c. `line` is NUL-terminated and never longer than
 * `DBM_MAX_LOG_LINE_LEN` (serial_protocol.h), so a sink can hand it straight
 * to a `LogLine` without re-checking length.
 *
 * `panic` is true only on the fatal-error path, where the log subsystem has
 * switched to synchronous processing inside the faulting context: a sink must
 * then NOT take a mutex (k_mutex_lock from an ISR/fault context is illegal)
 * and should write directly, accepting that it may interleave with a frame
 * some now-halted thread had half-written. Getting the fault out is the whole
 * point at that stage, and Core resyncs on the next COBS delimiter anyway.
 */
typedef void (*dev_bench_log_sink_fn)(const char *line, bool panic);

/* Installs `sink` and flushes whatever the backlog holds, oldest first, plus
 * a line reporting any backlog drops. Called from main.c's handle_hello once
 * `HelloAck` is on the wire -- deliberately after it, so the very first frame
 * Core sees on a freshly booted bench is still the ack it is waiting for.
 *
 * Passing NULL detaches the sink and resumes holding lines in the backlog.
 */
void dev_bench_log_set_sink(dev_bench_log_sink_fn sink);

/* Test-only introspection (app/tests/dev_bench_log). Counts lines the backlog
 * dropped since the last flush, not since boot. */
uint32_t dev_bench_log_backlog_dropped(void);

#endif /* EMBARCH_DEV_BENCH_LOG_H_ */
