/* embarch-dev-bench firmware entry point.
 *
 * embarch-dev-bench/design.md §1, §2, §3 decisions 6/7/12/19/20/21. Shared
 * across both workspaces (nordic/native_sim) — only ble_bridge_real.c vs
 * ble_bridge_stub.c differs per workspace (decision 16).
 *
 * The link (decision 6, revised) is whatever UART the board's own
 * `zephyr,console` chosen node names: the nRF54L15DK's on-board J-Link VCOM
 * for workspaces/nordic, native_sim's host-stdio-backed UART for
 * workspaces/native_sim. Using the same chosen node on both boards means this
 * file needs no per-workspace UART wiring. Zephyr's own console/log/shell
 * output must NOT also be routed to this device (prj.conf disables every
 * backend that would) — sharing the wire with raw log text would corrupt COBS
 * framing (decision 7); dev-bench's own log output travels as a `LogLine`
 * DevBenchMessage instead, and since decision 38 that includes the whole
 * `CONFIG_LOG` subsystem's output, forwarded by dev_bench_log.c through the
 * sink this file installs at handshake.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#ifdef CONFIG_HWINFO
#include <zephyr/drivers/hwinfo.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>

#include "ble_bridge.h"
#include "dev_bench_log.h"
#include "serial_protocol.h"
#include "study_ffi.h"

/* This application's own log module.
 *
 * **Registered at DBG, not INF, and the difference is not cosmetic.** A
 * module's `LOG_MODULE_REGISTER` level is a *compile-time* ceiling: Zephyr's
 * runtime filtering can only reduce below it, never raise above it. Decision
 * 38 registered this at INF, which quietly compiled every LOG_DBG in this file
 * out of existence -- so a study asking for `Debug` under decision 39 got the
 * bench's own step-by-step account of nothing at all. Found by running exactly
 * that study on hardware and seeing no lines.
 *
 * Everything is compiled in; what a run actually forwards is decided at
 * runtime by the study's level, with dev_bench_log.c holding this module at no
 * less than INF so the boot record and handshake diagnostics survive an idle
 * bench (dev_bench_log.h). */
LOG_MODULE_REGISTER(dev_bench, LOG_LEVEL_DBG);

#ifndef APP_FIRMWARE_VERSION
#define APP_FIRMWARE_VERSION "dev-bench-unknown"
#endif

static const struct device *const link_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Shared outbound scratch, built and sent by exactly one of
 * send_log_line/send_study_done/send_step_result/handle_hello at a time —
 * this single-threaded RX/dispatch loop never has two outbound messages in
 * flight simultaneously, so one buffer serves all of them rather than each
 * keeping its own.
 *
 * Real gap found and fixed, Milestone 3 (Study Designer: Feature-Branch
 * Iteration): before this, each of those four functions declared its own
 * `static struct dev_bench_message` -- harmless while `struct
 * dev_bench_message`'s union was a few KB (decision 21's own `StudyStart`
 * bump), but once `StepResult` had to grow to also hold `gatt_services`/
 * `gatt_activity` (design.md §3 decisions 31/32), four separate copies of
 * that much larger union pushed a real ESP32-C5 build past this board's
 * available RAM at link time -- confirmed empirically, not assumed (a real
 * `west build` failed with `region 'sram0_0_seg' overflowed`). Consolidating
 * to one shared instance (plus `receive_message`'s own separate inbound
 * buffer below, which must stay alive across a whole `dispatch_study` call
 * and so can't share this one) is what makes it fit. */
static struct dev_bench_message tx_scratch;

/* Guards the link UART and send_message's own `frame` static.
 *
 * Until design.md §3 decision 36 there was exactly one writer (this file's
 * RX/dispatch loop) and no lock was needed. The GATT transcript adds a second
 * one: the transcript TX thread below, which must keep draining while the
 * dispatch loop is blocked inside a long ble_bridge_execute() -- that's the
 * whole point, since a capture window's notifications arrive *during* the
 * steps that run inside it. Interleaving two COBS frames on the wire would
 * corrupt both, so the two writers serialize here. */
static K_MUTEX_DEFINE(link_tx_mutex);

/* Caller must hold link_tx_mutex -- which also covers tx_scratch, the buffer
 * every caller builds its message in before calling this. Locking inside here
 * instead would leave the build half unprotected, which is the half that
 * actually races. */
static void send_message_locked(const struct dev_bench_message *msg)
{
	/* `static`, not a stack local: even at the outbound bound this is a
	 * few KB, more than a small embedded call stack should hold. Safe
	 * because every writer holds link_tx_mutex.
	 *
	 * **Sized by DBM_MAX_OUTBOUND_FRAME_LEN, not DBM_MAX_FRAME_LEN** --
	 * this is the TX path, and dev-bench never sends a `StudyStart` (Core
	 * is the only sender), so the general bound was sizing this for a
	 * message this node cannot produce. That cost ~6.6 KB before schema
	 * v15 and would have cost ~9.7 KB after it, since
	 * DBM_MAX_STUDY_START_LEN now carries the `.eap` protocols span too.
	 * Exactly the same finding as `receive_message`'s RX buffer
	 * (DBM_MAX_INBOUND_FRAME_LEN, decision 38's SRAM pass), in the other
	 * direction; the general encoder's own staging buffer is deliberately
	 * left alone, because the round-trip tests encode a StudyStart through
	 * it. */
	static uint8_t frame[DBM_MAX_OUTBOUND_FRAME_LEN];
	int frame_len = dbm_encode_frame(msg, frame, sizeof(frame));

	if (frame_len < 0) {
		return;
	}
	for (int i = 0; i < frame_len; i++) {
		uart_poll_out(link_uart, frame[i]);
	}
}

/* ---- GATT transcript plumbing (design.md §3 decision 36) ---------------- */

/* Depth chosen against what a burst actually looks like on this link rather
 * than a round number: at 1 Mbaud a full 244-byte entry clears the wire in
 * well under a millisecond, so the queue only has to absorb the gap between
 * several notifications landing back-to-back in one connection interval and
 * the TX thread being scheduled. 16 slots is roughly 4.7 KB of static RAM --
 * deliberately modest on a board whose SRAM has already overflowed once
 * (design.md §3 decision 27's own finding). */
#define TRANSCRIPT_QUEUE_DEPTH 16

struct transcript_item {
	uint32_t step_index;
	struct ble_transcript_entry entry;
};

K_MSGQ_DEFINE(transcript_q, sizeof(struct transcript_item), TRANSCRIPT_QUEUE_DEPTH, 4);

/* ---- declared stream taps (schema v9, design.md §3 decision 29(a)) ------
 *
 * Copied out of the decoded `StudyStart` at dispatch, rather than read
 * through a pointer into the rx message buffer: that buffer is reused by the
 * next frame, and the transcript TX thread reads this concurrently with the
 * dispatch loop.
 */
static struct dbm_stream_tap study_taps[DBM_MAX_STREAMS_PER_STUDY];
static uint32_t study_taps_len;

/* Which taps are open right now, by index into `study_taps`. Only the
 * dispatch loop writes this; the transcript sink reads
 * `transcript_tap_open`, which is a single bool it can sample without
 * walking the array. */
static bool tap_open[DBM_MAX_STREAMS_PER_STUDY];

/* The tap a `StreamSource::GattTranscript` was declared on, as an index into
 * `study_taps`, or -1 when this study declared none.
 *
 * **-1 is the ordinary case, not an error**: a study that never asked for a
 * transcript does not get one. That is decision 39's model -- capture is
 * declared, not implicit -- and it is a real behaviour change from the
 * retired tag-10 path, which sent every entry unconditionally whether or not
 * anything upstream wanted it. */
static int transcript_tap_index = -1;

/* Sampled by the transcript sink in BT RX context: whether the transcript
 * tap is declared *and* currently open. A plain bool read is atomic on both
 * supported targets, and a stale sample costs one entry at a window edge --
 * far cheaper than taking a lock in that callback, which ble_bridge.h's
 * contract forbids anyway. */
static volatile bool transcript_tap_open;

/* Bumped by dispatch_study so each entry is stamped with the step that was
 * running when it happened. Read from the BLE callback context; a plain
 * uint32_t write/read is atomic on both supported targets, and a torn value
 * here would mislabel one transcript row, not corrupt the link. */
static volatile uint32_t transcript_step_index;

/* Counts entries dropped because the queue was full -- reported over LogLine
 * at the end of a study rather than silently swallowed, since a transcript
 * that quietly lost rows while claiming to be exhaustive is exactly the
 * failure mode decision 36 exists to remove. */
static volatile uint32_t transcript_dropped;

/* ble_bridge's sink: runs in the BT RX thread for anything inbound. Does the
 * least possible work -- one bounded copy into the queue -- and never touches
 * the UART, per ble_bridge.h's contract. */
/* Forward declaration: `send_log_line` is defined further down, alongside the
 * other send_* helpers that share `tx_scratch`. */
static void send_log_line(const char *text);

/* ble_bridge.h's `ble_log_sink`: forwards a bridge diagnostic as a `LogLine`
 * DevBenchMessage, the same channel dev-bench's own log output already uses
 * (design.md §3 decision 7). */
static void log_sink_cb(const char *line, void *user_data)
{
	ARG_UNUSED(user_data);
	send_log_line(line);
}

/* Whether any open DBM_STREAM_SRC_GATT_NOTIFY tap named this entry's
 * characteristic -- embarch-study-designer/design.md §3 decision 55.
 *
 * Read from the BT RX thread. `tap_open[]` is written only by the dispatch
 * thread and `study_taps[]` only between studies, so a stale read here costs
 * one record at a window edge, exactly like `transcript_tap_open`'s own
 * documented race. Taking a lock in this callback is what ble_bridge.h's
 * contract forbids. */
static bool notify_tap_wants(const struct ble_transcript_entry *entry)
{
	if (!entry->has_characteristic_uuid) {
		return false;
	}
	if (entry->kind != BLE_GATT_EVT_NOTIFICATION && entry->kind != BLE_GATT_EVT_INDICATION) {
		/* A GattNotify tap is the characteristic's *data*, not the
		 * story of the connection. Subscribed/unsubscribed/error
		 * events are real and are kept -- in the transcript, which is
		 * where the story belongs. Letting them into the tap's file
		 * would put zero-length rows in the middle of a decoded
		 * waveform. */
		return false;
	}
	for (uint32_t i = 0; i < study_taps_len; i++) {
		if (study_taps[i].source_tag != DBM_STREAM_SRC_GATT_NOTIFY || !tap_open[i]) {
			continue;
		}
		if (memcmp(study_taps[i].characteristic_uuid, entry->characteristic_uuid, 16) == 0) {
			return true;
		}
	}
	return false;
}

static void transcript_sink_cb(const struct ble_transcript_entry *entry, void *user_data)
{
	ARG_UNUSED(user_data);

	/* No declared transcript tap and no GattNotify tap that named this
	 * characteristic: the study didn't ask for this. Dropping here rather
	 * than at send time keeps the queue for entries that have somewhere to
	 * go, and is not counted as a drop -- nothing was lost that anyone
	 * asked to capture. */
	if (!transcript_tap_open && !notify_tap_wants(entry)) {
		return;
	}

	struct transcript_item item;

	item.step_index = transcript_step_index;
	item.entry = *entry;

	/* K_NO_WAIT: blocking the BT RX thread on a full queue would stall the
	 * very notifications being recorded. A dropped row is counted and
	 * reported instead. */
	if (k_msgq_put(&transcript_q, &item, K_NO_WAIT) != 0) {
		transcript_dropped++;
	}
}

static void transcript_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	/* Builds into the same tx_scratch every other sender uses, under the
	 * same mutex, rather than keeping a `struct dev_bench_message` of its
	 * own. Not a style preference: that union is sized by its largest
	 * member (`dbm_study_start`, ~10 KB), and a second instance overflowed
	 * this board's SRAM by 2720 bytes at link time -- measured, not
	 * predicted, exactly the way design.md §3 decision 27's own SRAM
	 * finding was. The mutex is what makes one buffer safe for two
	 * threads. */
	struct transcript_item item;

	while (1) {
		k_msgq_get(&transcript_q, &item, K_FOREVER);

		k_mutex_lock(&link_tx_mutex, K_FOREVER);

		struct dev_bench_message *msg = &tx_scratch;

		/* Re-checked here rather than trusted from queue time: the
		 * window can close while an entry is still queued, and a
		 * record sent on a closed tap is one Core keeps but warns
		 * about. Dropping it at the edge is the honest read -- the
		 * entry arrived outside the capture the study declared. */
		int tap_index = transcript_tap_index;
		bool want_transcript = tap_index >= 0 && tap_open[tap_index];

		/* Fan-out to every open GattNotify tap that named this
		 * characteristic (embarch-study-designer/design.md §3 decision 55). One captured
		 * notification can legitimately land in two files: the
		 * transcript, which is the complete story of the connection,
		 * and the characteristic's own tap, which is its data on its
		 * own with a declared layout to decode it. Neither is a copy of
		 * the other and neither is optional given the other.
		 *
		 * Sent *before* the transcript record below purely so the loop
		 * can reuse `msg` afterwards; order on the wire doesn't matter,
		 * since Core stamps arrival per record and each tap's file is
		 * written independently. */
		if (item.entry.has_characteristic_uuid &&
		    (item.entry.kind == BLE_GATT_EVT_NOTIFICATION ||
		     item.entry.kind == BLE_GATT_EVT_INDICATION)) {
			for (uint32_t i = 0; i < study_taps_len; i++) {
				if (study_taps[i].source_tag != DBM_STREAM_SRC_GATT_NOTIFY ||
				    !tap_open[i]) {
					continue;
				}
				if (memcmp(study_taps[i].characteristic_uuid,
					   item.entry.characteristic_uuid, 16) != 0) {
					continue;
				}
				if (item.entry.payload_len > DBM_MAX_STREAM_CHUNK_BYTES) {
					/* Cannot happen for an entry this
					 * firmware produced (one ATT MTU is
					 * smaller), but counted rather than
					 * silently skipped so a truncated
					 * capture never reads as complete. */
					transcript_dropped++;
					continue;
				}

				struct dev_bench_message *nmsg = &tx_scratch;

				memset(nmsg, 0, sizeof(*nmsg));
				nmsg->tag = DBM_TAG_STREAM_CHUNK_BATCH;
				nmsg->stream_chunk_batch.id = study_taps[i].id;
				nmsg->stream_chunk_batch.records_len = 1;

				struct dbm_stream_record *nrec =
					&nmsg->stream_chunk_batch.records[0];

				nrec->rx_utc_ms = item.entry.rx_utc_ms;
				/* **The raw ATT value, nothing around it.**
				 * This tap's records are the characteristic's
				 * payload bytes and only those -- no postcard
				 * envelope, no direction, no UUID -- because
				 * Core decodes them against the layout the
				 * engineer declared (`StreamEncoding::Struct`),
				 * and a layout describes the DUT's packet, not
				 * a dev-bench record wrapping it. */
				memcpy(nrec->bytes, item.entry.payload,
				       item.entry.payload_len);
				nrec->bytes_len = item.entry.payload_len;

				send_message_locked(nmsg);
			}
		}

		if (!want_transcript) {
			k_mutex_unlock(&link_tx_mutex);
			continue;
		}

		/* One entry, encoded to bare postcard bytes, carried as the
		 * payload of a single-record StreamChunkBatch on the declared
		 * transcript tap (design.md §3 decision 29(a)). The entry's
		 * own `rx_utc_ms` and the record's are the same value from
		 * the same clock -- kept both places so the transcript row
		 * shape decision 36 pinned stays byte-for-byte what it was.
		 *
		 * `step_index` is deliberately *not* carried any more. The
		 * generic record has no field for one, and Core takes the
		 * column from whichever step it has open when the record
		 * arrives -- which is what decision 36 defined that column to
		 * mean in the first place, and what the retired tag-10 path
		 * had been getting off-by-one at every step boundary. */
		struct dbm_gatt_transcript_entry entry;

		memset(&entry, 0, sizeof(entry));
		entry.rx_utc_ms = item.entry.rx_utc_ms;
		entry.direction = item.entry.direction;
		entry.kind = item.entry.kind;
		entry.has_service_uuid = item.entry.has_service_uuid;
		memcpy(entry.service_uuid, item.entry.service_uuid, 16);
		entry.has_characteristic_uuid = item.entry.has_characteristic_uuid;
		memcpy(entry.characteristic_uuid, item.entry.characteristic_uuid, 16);
		entry.att_status = item.entry.att_status;
		entry.payload_len = item.entry.payload_len;
		memcpy(entry.payload, item.entry.payload, item.entry.payload_len);

		memset(msg, 0, sizeof(*msg));
		msg->tag = DBM_TAG_STREAM_CHUNK_BATCH;
		msg->stream_chunk_batch.id = study_taps[tap_index].id;
		msg->stream_chunk_batch.records_len = 1;

		struct dbm_stream_record *record = &msg->stream_chunk_batch.records[0];

		record->rx_utc_ms = item.entry.rx_utc_ms;

		int entry_len = dbm_encode_transcript_entry(&entry, record->bytes,
							     sizeof(record->bytes));

		if (entry_len < 0) {
			/* Cannot happen for an entry this firmware produced --
			 * DBM_MAX_TRANSCRIPT_PAYLOAD_LEN is one ATT MTU and
			 * DBM_MAX_STREAM_CHUNK_BYTES is larger -- but counted
			 * rather than silently skipped, so the study's own
			 * "NOT exhaustive" report stays true if it ever does.
			 */
			transcript_dropped++;
			k_mutex_unlock(&link_tx_mutex);
			continue;
		}
		record->bytes_len = (uint32_t)entry_len;

		send_message_locked(msg);
		k_mutex_unlock(&link_tx_mutex);
	}
}

K_THREAD_DEFINE(transcript_tx_tid, 2048, transcript_tx_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(7), 0, 0);

/* `skip_lock` exists for exactly one caller: the log backend's fatal-error
 * path (dev_bench_log.h), which runs in the faulting context where
 * k_mutex_lock is illegal. Everything else locks. */
static void send_log_line_ex(const char *text, bool skip_lock)
{
	struct dev_bench_message *msg = &tx_scratch;

	if (!skip_lock) {
		k_mutex_lock(&link_tx_mutex, K_FOREVER);
	}
	memset(msg, 0, sizeof(*msg));
	msg->tag = DBM_TAG_LOG_LINE;
	strncpy(msg->log_line.text, text, DBM_MAX_LOG_LINE_LEN);
	msg->log_line.text[DBM_MAX_LOG_LINE_LEN] = '\0';
	send_message_locked(msg);
	if (!skip_lock) {
		k_mutex_unlock(&link_tx_mutex);
	}
}

static void send_log_line(const char *text)
{
	send_log_line_ex(text, false);
}

/* dev_bench_log.h's sink: every `CONFIG_LOG` record this firmware or any
 * Zephyr subsystem produces, already formatted and length-bounded, out on the
 * same `LogLine` channel decision 7 established for hand-written diagnostics.
 * Deliberately the same channel and not a new message variant -- it costs no
 * wire schema version, and Core has to store both kinds in the same place
 * anyway (embarch-core/design.md §3 decision 37).
 *
 * Runs on the log processing thread (deferred mode), except on the fatal path
 * where it runs in the faulting context and `panic` is true. */
static void log_backend_sink(const char *line, bool panic)
{
	send_log_line_ex(line, panic);
}

/* ---- stream tap open/close (design.md §3 decision 29(a)) ---------------- */

/* Sends StreamOpen/StreamClose for `study_taps[index]` and records the new
 * state. `dropped` is only meaningful on close.
 *
 * Both go out under link_tx_mutex like every other sender, and both are sent
 * from the dispatch thread only -- the transcript TX thread reads
 * `tap_open[]` but never writes it, so there is one writer and no lock is
 * needed to protect the array itself. */
static void set_tap_open(uint32_t index, bool open, uint32_t dropped)
{
	struct dev_bench_message *msg = &tx_scratch;

	k_mutex_lock(&link_tx_mutex, K_FOREVER);
	memset(msg, 0, sizeof(*msg));
	if (open) {
		msg->tag = DBM_TAG_STREAM_OPEN;
		msg->stream_open.id = study_taps[index].id;
	} else {
		msg->tag = DBM_TAG_STREAM_CLOSE;
		msg->stream_close.id = study_taps[index].id;
		msg->stream_close.dropped = dropped;
	}
	send_message_locked(msg);
	k_mutex_unlock(&link_tx_mutex);

	tap_open[index] = open;
	if ((int)index == transcript_tap_index) {
		transcript_tap_open = open;
	}
}

/* Brings every dev-bench-mediated tap's open/closed state into line with what
 * its declared `StreamScope` says for `step_index`, sending only the
 * transitions.
 *
 * Called before each step runs, so a tap whose window starts at step N is
 * already open when N's action begins -- and, with `step_index` one past the
 * last step, as the way to close whatever is still open at the end.
 *
 * A DBM_STREAM_SRC_SIGNAL tap is skipped entirely: Core opens that one on its
 * own carrier, and dev-bench announcing it too would put two producers on one
 * id. */
static void sync_taps_for_step(uint32_t step_index)
{
	for (uint32_t i = 0; i < study_taps_len; i++) {
		const struct dbm_stream_tap *tap = &study_taps[i];

		if (!dbm_stream_tap_is_ours(tap)) {
			continue;
		}

		bool want_open = dbm_stream_tap_covers(tap, step_index);

		if (want_open == tap_open[i]) {
			continue;
		}
		/* `transcript_dropped` is this firmware's only drop counter,
		 * and the transcript tap is the only tap it can describe --
		 * every other tap reports 0 because nothing here produces
		 * records for one yet. Reporting a count that isn't this
		 * tap's would be worse than reporting none. */
		uint32_t dropped = (!want_open && (int)i == transcript_tap_index)
					   ? transcript_dropped
					   : 0;

		set_tap_open(i, want_open, dropped);
	}
}

/* Copies the declared taps out of a decoded StudyStart and resets per-study
 * tap state. Returns nothing: a study declaring no taps, or only taps this
 * node doesn't mediate, is perfectly ordinary. */
static void load_study_taps(const struct dbm_study_start *study)
{
	study_taps_len = study->streams_len;
	transcript_tap_index = -1;
	transcript_tap_open = false;
	memset(tap_open, 0, sizeof(tap_open));

	for (uint32_t i = 0; i < study_taps_len; i++) {
		study_taps[i] = study->streams[i];
		if (study_taps[i].source_tag == DBM_STREAM_SRC_GATT_TRANSCRIPT &&
		    transcript_tap_index < 0) {
			transcript_tap_index = (int)i;
		}
	}
}

static void send_study_done(bool completed)
{
	struct dev_bench_message *msg = &tx_scratch;

	k_mutex_lock(&link_tx_mutex, K_FOREVER);
	memset(msg, 0, sizeof(*msg));
	msg->tag = DBM_TAG_STUDY_DONE;
	msg->study_done.completed = completed;
	send_message_locked(msg);
	k_mutex_unlock(&link_tx_mutex);
}

/* Builds and sends one `StepResult` from a step's device-observed `struct
 * outcome` (ble_bridge.h) — the C-side `Outcome`/`captured_data` shapes
 * mirror embarch-study-designer's `result::Outcome`/`StepResult` closely
 * enough (design.md §4.5) that this is a direct field-by-field translation,
 * not a reinterpretation. `power_samples_ref`/`waveform_ref` are never set
 * (encoded as `None` by serial_protocol.c's own encode_body): this pass has
 * no power/waveform capture yet (decision 21's scope). */
static void send_step_result(uint32_t step_index, const char *step_name,
			      const struct outcome *bridge_outcome)
{
	struct dev_bench_message *msg = &tx_scratch;

	k_mutex_lock(&link_tx_mutex, K_FOREVER);
	memset(msg, 0, sizeof(*msg));
	msg->tag = DBM_TAG_STEP_RESULT;
	msg->step_result.step_index = step_index;
	strncpy(msg->step_result.result.step_name, step_name, DBM_MAX_NAME_LEN);
	msg->step_result.result.step_name[DBM_MAX_NAME_LEN] = '\0';

	switch (bridge_outcome->kind) {
	case OUTCOME_PASS:
		msg->step_result.result.outcome.tag = 0;
		break;
	case OUTCOME_FAIL:
		msg->step_result.result.outcome.tag = 1;
		strncpy(msg->step_result.result.outcome.fail_reason, bridge_outcome->fail_reason,
			DBM_MAX_FAIL_REASON_LEN);
		msg->step_result.result.outcome.fail_reason[DBM_MAX_FAIL_REASON_LEN] = '\0';
		break;
	case OUTCOME_TIMED_OUT:
	default:
		msg->step_result.result.outcome.tag = 2;
		break;
	}

	if (bridge_outcome->captured_data != NULL && bridge_outcome->captured_len > 0) {
		size_t len = bridge_outcome->captured_len;

		if (len > sizeof(msg->step_result.result.captured_data)) {
			len = sizeof(msg->step_result.result.captured_data);
		}
		memcpy(msg->step_result.result.captured_data, bridge_outcome->captured_data, len);
		msg->step_result.result.captured_data_len = (uint32_t)len;
		msg->step_result.result.has_captured_data = true;
	}

	/* gatt_services (design.md §3 decisions 31/32) -- populated by every
	 * discovering action; NULL/0 from ble_bridge_execute() for every other
	 * action kind, same borrowed-pointer lifetime as captured_data above.
	 *
	 * `gatt_activity` was copied here too and is retired at schema v14
	 * (that doc's decision 54). */
	if (bridge_outcome->gatt_services != NULL && bridge_outcome->gatt_service_count > 0) {
		size_t count = bridge_outcome->gatt_service_count;

		if (count > DBM_MAX_DISCOVERED_SERVICES) {
			count = DBM_MAX_DISCOVERED_SERVICES;
		}
		for (size_t s = 0; s < count; s++) {
			const struct ble_gatt_service_info *src = &bridge_outcome->gatt_services[s];
			struct dbm_gatt_service_info *dst = &msg->step_result.result.gatt_services[s];

			memcpy(dst->uuid, src->uuid, 16);
			size_t char_count = src->characteristics_len;

			if (char_count > DBM_MAX_CHARS_PER_SERVICE) {
				char_count = DBM_MAX_CHARS_PER_SERVICE;
			}
			for (size_t c = 0; c < char_count; c++) {
				memcpy(dst->characteristics[c].uuid, src->characteristics[c].uuid, 16);
				dst->characteristics[c].properties = src->characteristics[c].properties;
			}
			dst->characteristics_len = (uint32_t)char_count;
		}
		msg->step_result.result.gatt_services_len = (uint32_t)count;
		msg->step_result.result.has_gatt_services = true;
	}

	/* `security_level` (embarch-study-designer/design.md §3 decision 44) --
	 * the bridge stamps this for every action kind, so this is a straight
	 * pass-through and not a per-action-kind decision. `enum
	 * ble_security_level` and `enum dbm_security_level` carry the same
	 * values by construction (both headers say so), so no remapping. */
	msg->step_result.result.has_security_level = bridge_outcome->has_security_level;
	msg->step_result.result.security_level = bridge_outcome->security_level;

	/* `protocol` (embarch-study-designer/design.md §3 decision 62) -- set by
	 * ACTION_RUN_PROTOCOL alone, which is the one action kind that runs a
	 * state machine at all. Another straight pass-through: the outcome tags
	 * on both sides are the same `Outcome` discriminants, so there is
	 * nothing to remap here either.
	 *
	 * Copied even when the *step* failed, deliberately. A run cut short by
	 * a lost link reports a step failure whose protocol half still names
	 * the state the machine was sitting in, and telling that apart from a
	 * protocol that reached its own `failed` state is the whole reason
	 * this field is separate from the one above it. */
	msg->step_result.result.has_protocol = bridge_outcome->has_protocol;
	if (bridge_outcome->has_protocol) {
		strncpy(msg->step_result.result.protocol_final_state,
			bridge_outcome->protocol_final_state, EAP_MAX_STATE_NAME_LEN);
		msg->step_result.result.protocol_final_state[EAP_MAX_STATE_NAME_LEN] = '\0';
		msg->step_result.result.protocol_outcome.tag = bridge_outcome->protocol_outcome_tag;
		strncpy(msg->step_result.result.protocol_outcome.fail_reason,
			bridge_outcome->protocol_fail_reason, DBM_MAX_FAIL_REASON_LEN);
		msg->step_result.result.protocol_outcome.fail_reason[DBM_MAX_FAIL_REASON_LEN] =
			'\0';
	}

	send_message_locked(msg);
	k_mutex_unlock(&link_tx_mutex);
}

/* ---- Inbound link RX (embarch-dev-bench/design.md §3 decision 29) -------
 *
 * The hardware FIFO is drained by an ISR into this ring buffer, and the
 * dispatch loop parses frames out of the ring buffer. Those two jobs used to
 * be the same loop: `receive_message` called `uart_poll_in` directly and
 * `k_sleep(K_MSEC(1))` whenever the FIFO happened to be empty.
 *
 * That silently capped the largest `StudyStart` dev-bench could receive at
 * the size of the UART's hardware FIFO. This link runs at 1 Mbaud (this
 * board's own overlay), so ~100 bytes land per millisecond slept, against an
 * ESP32 FIFO 128 bytes deep -- so any frame bigger than the FIFO lost
 * whatever arrived while the reader was asleep, `dbm_decode_frame` failed on
 * the truncated result, and the loop below just `continue`d. Core saw no
 * reply at all and reported "no message received from dev-bench" on step 0.
 *
 * Found on real hardware running the first stimulate-and-capture study
 * (roadmap Milestone 6), whose four-step `StudyStart` is 132 payload bytes --
 * the first study this suite has ever authored large enough to cross that
 * line, which is why every earlier study worked and this one never did. The
 * threshold was then confirmed directly by sweeping one step's `Write`
 * payload length: 128-byte frames completed, 134-byte frames timed out with
 * no message, every time.
 *
 * Sized to cover *scheduling latency*, not a whole frame. It deliberately
 * isn't `DBM_MAX_FRAME_LEN`-sized: that constant is the worst-case 64-step
 * `StudyStart` (~10 KB, see serial_protocol.h), which this board cannot
 * spare -- its SRAM has already overflowed once (design.md §3 decision 27)
 * and sits at ~95% used. It doesn't need to: `receive_message` drains this
 * buffer continuously into `rx_buf` as bytes arrive, so the buffer only has
 * to hold what accumulates while the dispatch loop isn't running, not the
 * whole frame at once. At 1 Mbaud, 2 KB is ~20 ms of wire time -- far more
 * slack than a 1 ms sleep needs, and nothing else competes: Core sends
 * exactly Hello then StudyStart and then waits (embarch-study-designer/
 * design.md §3 decision 24), so there is no inbound traffic at all during
 * the long `ble_bridge_execute` calls this loop blocks in.
 *
 * Overruns are counted and reported rather than silently dropped -- the
 * whole point of this change is that a lost inbound byte must never again
 * look like a dead link.
 */
#define LINK_RX_RING_BYTES 2048
RING_BUF_DECLARE(link_rx_ring, LINK_RX_RING_BYTES);

/* Bytes the ISR had to discard because the ring buffer was full. `volatile`:
 * written in interrupt context, read by the dispatch loop. */
static volatile uint32_t link_rx_overruns;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void link_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) > 0 && uart_irq_rx_ready(dev) > 0) {
		uint8_t chunk[64];
		int read = uart_fifo_read(dev, chunk, sizeof(chunk));

		if (read <= 0) {
			break;
		}
		uint32_t stored = ring_buf_put(&link_rx_ring, chunk, (uint32_t)read);

		if (stored < (uint32_t)read) {
			link_rx_overruns += (uint32_t)read - stored;
		}
	}
}
#endif

/* Reads one byte from the ring buffer, or returns -1 if none is available.
 *
 * The poll-mode fallback exists only for a platform whose UART driver has no
 * interrupt-driven mode (native_sim, if the app is ever built for it -- the
 * ztest suite doesn't compile this file). It carries the FIFO-depth limit
 * described above and is not what real hardware uses.
 */
static int link_rx_byte(uint8_t *byte)
{
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	return ring_buf_get(&link_rx_ring, byte, 1) == 1 ? 0 : -1;
#else
	return uart_poll_in(link_uart, byte) == 0 ? 0 : -1;
#endif
}

/* Blocks until one full COBS frame has been read off the link UART and
 * decoded. Malformed frames are dropped silently and the reader resyncs on
 * the next 0x00 delimiter, per COBS's own resync property
 * (embarch-study-designer/design.md §3 decision 10). */
static int receive_message(struct dev_bench_message *out)
{
	/* DBM_MAX_INBOUND_FRAME_LEN, not DBM_MAX_FRAME_LEN: Core sends dev-bench
	 * only `Hello` and `StudyStart`, and sizing this for StepResult's much
	 * larger bound cost ~10 KB of SRAM for a frame that cannot arrive. See
	 * serial_protocol.h's own comment on that constant for the full
	 * reasoning and for what pays attention to it. The overflow branch below
	 * is what makes the narrower bound safe rather than merely smaller. */
	static uint8_t rx_buf[DBM_MAX_INBOUND_FRAME_LEN];
	size_t rx_len = 0;
	uint32_t reported_overruns = 0;

	while (1) {
		uint8_t byte;

		if (link_rx_byte(&byte) != 0) {
			/* Report an overrun only while idle between frames: this
			 * is the one point in the loop where sending is safe and
			 * where no partial frame is pending. */
			uint32_t overruns = link_rx_overruns;

			if (overruns != reported_overruns && rx_len == 0) {
				char note[DBM_MAX_LOG_LINE_LEN + 1];

				snprintk(note, sizeof(note),
					 "link RX overran: %u inbound byte(s) dropped -- a "
					 "frame was almost certainly lost",
					 (unsigned int)(overruns - reported_overruns));
				reported_overruns = overruns;
				send_log_line(note);
			}
			k_sleep(K_MSEC(1));
			continue;
		}
		if (byte == 0x00) {
			int status = (rx_len > 0) ? dbm_decode_frame(rx_buf, rx_len, out) : -1;

			rx_len = 0;
			if (status == 0) {
				return 0;
			}
			continue;
		}
		if (rx_len < sizeof(rx_buf)) {
			rx_buf[rx_len++] = byte;
		} else {
			/* Drop and resync on the next delimiter, as before -- but say
			 * so now that there is somewhere to say it (decision 38).
			 * Silence here is how "Core sent something this firmware
			 * cannot receive" would look exactly like "the link is
			 * quiet", and this branch is the one thing standing between
			 * DBM_MAX_INBOUND_FRAME_LEN being a sound bound and being a
			 * buffer overrun. */
			LOG_WRN("inbound frame exceeded %zu bytes; dropped and resyncing",
				sizeof(rx_buf));
			rx_len = 0;
		}
	}
}

/* `Hello` doubles as a hard reset (embarch-study-designer/design.md §3
 * decision 12 / embarch-dev-bench decision 11) and a schema-compatibility
 * handshake. Returns whether dev-bench should now wait for a `StudyStart`
 * (i.e. the schema versions matched) — `false` on a mismatch, matching the
 * previous bring-up behavior of not running anything in that case. */
/* This board's own factory-unique chip ID, hex-encoded lowercase into `out`
 * (embarch-study-designer/design.md §3 decision 47, embarch-core/design.md §3
 * decision 35). Core compares it against the identity its JTAG probe just
 * read -- the only thing that ties the runtime serial link and the JTAG
 * connection to the same silicon, since the port migration made them
 * physically separate USB devices.
 *
 * **Reports an empty string rather than a substitute** when this build has no
 * `hwinfo` driver (native_sim, most of all) or the driver refuses. A bench
 * that cannot answer says nothing; inventing a plausible ID here would defeat
 * the entire check, and Core's side is where "no ID" gets its meaning.
 */
/* Uptime at handshake, plus the reset cause when `hwinfo` can name one — see
 * handle_hello's own comment for what this is for. */
static void send_reset_diagnostics(void)
{
	char line[DBM_MAX_LOG_LINE_LEN + 1];
	int64_t uptime_ms = k_uptime_get();

#ifdef CONFIG_HWINFO
	uint32_t cause = 0;
	int err = hwinfo_get_reset_cause(&cause);

	if (err == 0) {
		snprintk(line, sizeof(line),
			 "uptime %lld ms at handshake, reset cause 0x%08x", (long long)uptime_ms,
			 (unsigned int)cause);
		/* Cleared so the *next* handshake's cause describes the next
		 * reset rather than accumulating every cause since power-on,
		 * which is what the driver's own flags do if nobody clears
		 * them. A driver that does not support clearing is not an
		 * error worth reporting -- the uptime is the load-bearing
		 * half. */
		(void)hwinfo_clear_reset_cause();
	} else {
		snprintk(line, sizeof(line),
			 "uptime %lld ms at handshake, reset cause unavailable (%d)",
			 (long long)uptime_ms, err);
	}
#else
	snprintk(line, sizeof(line), "uptime %lld ms at handshake, no hwinfo driver",
		 (long long)uptime_ms);
#endif
	send_log_line(line);
}

static void read_hardware_id(char *out, size_t out_cap)
{
	out[0] = '\0';

#ifdef CONFIG_HWINFO
	uint8_t id[DBM_MAX_HARDWARE_ID_LEN / 2];
	ssize_t len = hwinfo_get_device_id(id, sizeof(id));

	if (len <= 0) {
		return;
	}
	/* Truncate rather than overflow, and keep it even-length so the hex is
	 * always whole bytes. `out_cap` is DBM_MAX_HARDWARE_ID_LEN + 1, and
	 * `id` is sized so this cannot actually bite -- belt and braces around
	 * a driver returning more than asked for. */
	if ((size_t)len * 2 >= out_cap) {
		len = (ssize_t)((out_cap - 1) / 2);
	}
	static const char hex[] = "0123456789abcdef";

	for (ssize_t i = 0; i < len; i++) {
		out[i * 2] = hex[(id[i] >> 4) & 0x0f];
		out[i * 2 + 1] = hex[id[i] & 0x0f];
	}
	out[len * 2] = '\0';
#else
	(void)out_cap;
#endif
}

static bool handle_hello(const struct dbm_hello *hello)
{
	ble_bridge_reset();

	/* `Hello` is a hard reset (embarch-study-designer/design.md §3 decision
	 * 12), and since decision 39 verbosity is study state like any other:
	 * a study that died mid-run without reaching dispatch_study's own revert
	 * must not leave the next one running at its level. */
	dev_bench_log_set_level(DEV_BENCH_LOG_BOOT_LEVEL);

	uint32_t our_schema = study_ffi_schema_version();
	struct dev_bench_message *ack = &tx_scratch;

	k_mutex_lock(&link_tx_mutex, K_FOREVER);
	memset(ack, 0, sizeof(*ack));
	ack->tag = DBM_TAG_HELLO_ACK;
	ack->hello_ack.schema_version = our_schema;
	ack->hello_ack.compatible = (hello->schema_version == our_schema);
	strncpy(ack->hello_ack.firmware_version, APP_FIRMWARE_VERSION, DBM_MAX_FIRMWARE_VERSION_LEN);
	ack->hello_ack.firmware_version[DBM_MAX_FIRMWARE_VERSION_LEN] = '\0';
	read_hardware_id(ack->hello_ack.hardware_id, sizeof(ack->hello_ack.hardware_id));
	bool compatible = ack->hello_ack.compatible;

	send_message_locked(ack);
	k_mutex_unlock(&link_tx_mutex);

	/* Deliberately here -- after `HelloAck` is on the wire, before anything
	 * else. Core's handshake tolerates a `LogLine` arriving ahead of the ack
	 * (embarch-core/design.md §3 decision 37) so a bench that has already
	 * handshaked once and is logging live cannot break a later handshake,
	 * but on a freshly booted bench the ack still comes first. Installing
	 * the sink flushes whatever the boot backlog holds, so the *first* thing
	 * Core learns after the ack is how this bench came up -- which is the
	 * gap the uptime/reset-cause line below was a stand-in for. */
	dev_bench_log_set_sink(log_backend_sink);

	/* **Why this uptime/reset-cause line exists.** A study failed on
	 * 2026-08-26 with Core reporting a one-byte `00` frame — an empty COBS
	 * frame, which is not something any encoder here can produce. The
	 * candidate explanations were "dev-bench sent a malformed message" and
	 * "dev-bench died mid-study and the line went to garbage", and nothing
	 * observable distinguished them: this firmware runs with
	 * CONFIG_BOOT_BANNER=n and CONFIG_LOG=n (decision 7), so a reboot is
	 * completely silent from Core's side and the next `Hello` looks
	 * identical to one on a bench that never faltered.
	 *
	 * An uptime measured in *milliseconds* at handshake time is the whole
	 * tell: Core sends `Hello` when it opens the port, so a bench that has
	 * been sitting there reports seconds-to-minutes and a bench that just
	 * rebooted reports a few hundred milliseconds. The reset cause names
	 * *why* when the driver knows.
	 *
	 * A `LogLine` rather than a `HelloAck` field on purpose: this is a
	 * diagnostic, not something Core acts on, and it costs no wire version
	 * (§3 decision 7 already routes dev-bench's log output here). */
	send_reset_diagnostics();

	if (!compatible) {
		send_log_line("schema version mismatch, not awaiting a StudyStart");
		return false;
	}
	return true;
}

/* Range-checks an `Action::RunProtocol` step's two indices against the study
 * that carries them, sending a failing `StepResult` naming the specific
 * problem when either is out of range.
 *
 * Split out rather than folded into `step_to_action` because that function
 * returns a value and this one has to *report*: a step that cannot be
 * translated has to reach Core as a named failure, not as a silently
 * mistranslated action. */
static bool protocol_index_valid(const struct dbm_study_start *study,
				 const struct dbm_step *step, uint32_t step_index)
{
	struct outcome bad = {.kind = OUTCOME_FAIL};
	uint8_t protocol = step->action.run_protocol.protocol;
	uint8_t entry_state = step->action.run_protocol.entry_state;

	if (protocol >= study->protocols_len) {
		snprintk(bad.fail_reason, sizeof(bad.fail_reason),
			 "step names protocol %u; this study carries %u",
			 (unsigned int)protocol, (unsigned int)study->protocols_len);
	} else if (entry_state >= study->protocols[protocol].states_len) {
		snprintk(bad.fail_reason, sizeof(bad.fail_reason),
			 "protocol %u has no state %u", (unsigned int)protocol,
			 (unsigned int)entry_state);
	} else {
		return true;
	}

	send_step_result(step_index, step->name, &bad);
	return false;
}

/* Translates one decoded `struct dbm_step`'s action into the `struct action`
 * ble_bridge.h's shared BleAdvertise/BleConnect/DataExchange/GattDiscover/
 * GattMonitorAll/GattMonitorStart/GattMonitorStop/BleSecurity/BleUnbond
 * surface expects — a direct field-by-field mapping per kind,
 * decision 21's own original BleAdvertise-only translation extended to cover
 * decisions 31/32's two new kinds and the BleConnect/DataExchange dispatch
 * that had never actually been wired up to a real decoded `Study` before
 * this pass (`ble_bridge_real.c` itself has implemented both since
 * design.md §3 decision 16's own implementation note — this is the missing
 * wiring, not new BLE logic).
 */
static struct action step_to_action(const struct dbm_study_start *study,
				    const struct dbm_step *step)
{
	struct action action = {0};

	switch (step->action_tag) {
	case DBM_ACTION_BLE_ADVERTISE:
		action.kind = ACTION_BLE_ADVERTISE;
		action.advertise.has_local_name = step->action.advertise.has_local_name;
		action.advertise.service_uuid_count = 0; /* not carried this far, decision 21 */
		action.advertise.adv_interval_ms = step->action.advertise.adv_interval_ms;
		strncpy(action.advertise.local_name, step->action.advertise.local_name,
			BLE_MAX_LOCAL_NAME_LEN);
		action.advertise.local_name[BLE_MAX_LOCAL_NAME_LEN] = '\0';
		break;

	case DBM_ACTION_BLE_CONNECT:
		action.kind = ACTION_BLE_CONNECT;
		action.connect.role =
			(step->action.connect.role == 1) ? BLE_ROLE_PERIPHERAL : BLE_ROLE_CENTRAL;
		action.connect.has_target_address = step->action.connect.has_target_address;
		action.connect.target_address_kind = (step->action.connect.target_address_kind == 1)
							      ? BLE_ADDR_RANDOM
							      : BLE_ADDR_PUBLIC;
		memcpy(action.connect.target_address, step->action.connect.target_address, 6);
		action.connect.has_target_name = step->action.connect.has_target_name;
		strncpy(action.connect.target_name, step->action.connect.target_name,
			BLE_MAX_LOCAL_NAME_LEN);
		action.connect.target_name[BLE_MAX_LOCAL_NAME_LEN] = '\0';
		break;

	case DBM_ACTION_DATA_EXCHANGE: {
		const struct dbm_data_exchange_action *de = &step->action.data_exchange;

		action.kind = ACTION_DATA_EXCHANGE;
		memcpy(action.data_exchange.service_uuid, de->service_uuid, 16);
		memcpy(action.data_exchange.characteristic_uuid, de->characteristic_uuid, 16);
		action.data_exchange.operation.kind = (enum gatt_operation_kind)de->operation.kind;
		switch (de->operation.kind) {
		case DBM_GATT_OP_WRITE:
			/* Borrowed, not copied -- ble_bridge.h's own contract for
			 * gatt_operation.write.payload: valid for the duration of
			 * this ble_bridge_execute() call only, which is exactly
			 * `step`'s own lifetime here (still owned by the caller's
			 * `study->steps[i]` for that whole call). */
			action.data_exchange.operation.write.payload = de->operation.payload;
			action.data_exchange.operation.write.payload_len = de->operation.payload_len;
			break;
		case DBM_GATT_OP_NOTIFY:
			action.data_exchange.operation.notify.timeout_ms = de->operation.timeout_ms;
			break;
		case DBM_GATT_OP_INDICATE:
			action.data_exchange.operation.indicate.timeout_ms = de->operation.timeout_ms;
			break;
		default:
			break; /* Read/Subscribe/StreamCapture: no operation fields */
		}
		break;
	}

	case DBM_ACTION_GATT_DISCOVER:
		action.kind = ACTION_GATT_DISCOVER;
		break;

	case DBM_ACTION_GATT_MONITOR_ALL:
		action.kind = ACTION_GATT_MONITOR_ALL;
		break;

	case DBM_ACTION_GATT_MONITOR_START:
		action.kind = ACTION_GATT_MONITOR_START;
		break;

	case DBM_ACTION_GATT_MONITOR_STOP:
		action.kind = ACTION_GATT_MONITOR_STOP;
		break;

	case DBM_ACTION_BLE_SECURITY:
		action.kind = ACTION_BLE_SECURITY;
		/* Straight copy: `enum dbm_security_level` and `enum
		 * ble_security_level` carry the same values by construction
		 * (each header states it), and the decoder already refused any
		 * value outside the range. */
		action.set_security.level = step->action.set_security.level;
		break;

	case DBM_ACTION_BLE_UNBOND:
		action.kind = ACTION_BLE_UNBOND;
		break;

	case DBM_ACTION_RUN_PROTOCOL:
		/* The one translation that resolves an index rather than
		 * copying a field: `Action::RunProtocol` names a slot in
		 * `Study.protocols`, and the bridge takes the manifest itself
		 * (ble_bridge.h's `struct run_protocol_params`). Borrowed, not
		 * copied -- `study` is the decoded `StudyStart` that outlives
		 * every step, so a 4 KB copy per step would buy nothing.
		 *
		 * Range-checked by `protocol_index_valid` before this runs, so
		 * the subscript here cannot be the raw one §3 decision 18's
		 * rule exists to turn into a sentence. */
		action.kind = ACTION_RUN_PROTOCOL;
		action.run_protocol.def = &study->protocols[step->action.run_protocol.protocol];
		action.run_protocol.entry_state = step->action.run_protocol.entry_state;
		break;

	case DBM_ACTION_GATT_MONITOR_SELECTED:
	case DBM_ACTION_GATT_MONITOR_SELECTED_START: {
		const struct dbm_gatt_monitor_selected_action *ms = &step->action.monitor_selected;
		size_t count = ms->targets_len;

		action.kind = (step->action_tag == DBM_ACTION_GATT_MONITOR_SELECTED)
				      ? ACTION_GATT_MONITOR_SELECTED
				      : ACTION_GATT_MONITOR_SELECTED_START;
		/* Copied rather than borrowed, unlike a Write payload above:
		 * ACTION_GATT_MONITOR_SELECTED_START's subscriptions outlive
		 * this ble_bridge_execute() call by design, so a pointer into
		 * `step` would dangle the moment the study moved on. The two
		 * caps are equal by construction (both mirror
		 * limits::MAX_MONITOR_TARGETS) and the decoder already refused
		 * anything larger; clamped anyway rather than trusting that
		 * across two headers. */
		if (count > BLE_MAX_MONITOR_TARGETS) {
			count = BLE_MAX_MONITOR_TARGETS;
		}
		for (size_t t = 0; t < count; t++) {
			memcpy(action.monitor_selected.targets[t].service_uuid,
			       ms->targets[t].service_uuid, 16);
			memcpy(action.monitor_selected.targets[t].characteristic_uuid,
			       ms->targets[t].characteristic_uuid, 16);
		}
		action.monitor_selected.targets_len = count;
		break;
	}

	default:
		/* Unreachable: serial_protocol.c's own decode already rejected any
		 * action tag it doesn't recognize (has_unsupported_action) before a
		 * dbm_study_start ever reaches dispatch_study. */
		break;
	}

	return action;
}

/* Real per-`Study` dispatch (embarch-dev-bench/design.md §3 decision 21) —
 * every `Action` kind embarch-study-designer defines is dispatched now
 * (decisions 31/32 closed the BleAdvertise-only scope this originally
 * shipped with; embarch-study-designer decisions 44/50 added the security
 * pair). `study`/`serial_protocol.c`'s own decode already rejected
 * any action kind it doesn't recognize at all (`has_unsupported_action`)
 * rather than partially decoding it, so by the time a `struct
 * dbm_study_start` reaches here every step in
 * `study->steps[0..study->steps_len)` is one of today's nine known kinds
 * (schema v12 added ACTION_BLE_SECURITY/ACTION_BLE_UNBOND).
 *
 * `steps_crc` was already verified during decode (serial_protocol.c's own
 * CRC-32 over the raw wire bytes of `steps`, independently reproducing
 * embarch-study-designer's `steps_crc()` — see serial_protocol.h's doc
 * comment on `dbm_decode_frame` for why this needs no FFI round-trip into
 * that crate to get the identical answer `essd_study_decode_and_verify`
 * would). This function only needs to act on the verdict.
 */
static void dispatch_study(const struct dbm_study_start *study)
{
	if (study->has_unsupported_action) {
		send_log_line("StudyStart contains a step whose action this firmware doesn't "
			      "recognize; aborting without running any step");
		send_study_done(false);
		return;
	}
	if (!study->steps_crc_valid) {
		send_log_line("StudyStart steps_crc mismatch; aborting without running any step");
		send_study_done(false);
		return;
	}
	/* The sibling seal over `streams` (embarch-study-designer/design.md §3
	 * decision 39's 2026-08-25 amendment). Checked separately and reported
	 * separately, which is the point of there being two: the log line names
	 * which half of the Study arrived corrupt.
	 *
	 * Aborting on it, rather than only computing it, even though this
	 * firmware does not open taps yet: a Study whose tap declarations are
	 * corrupt would otherwise run to completion and produce results with
	 * captures silently missing or wrong, which is the exact failure this
	 * suite keeps arriving at from other directions. */
	if (!study->streams_crc_valid) {
		send_log_line("StudyStart streams_crc mismatch; aborting without running any step");
		send_study_done(false);
		return;
	}

	/* The study's own verbosity (decision 39), applied after the guards
	 * above -- those three abort paths run no step and report through
	 * `send_log_line`, which bypasses the log subsystem entirely, so they
	 * neither need the study's level nor have anything to revert.
	 *
	 * Reverted just before `send_study_done` below, so the whole study body
	 * runs at what it asked for and the bench is quiet again the moment it
	 * ends. */
	uint8_t want_level = study->dev_bench_log_level;
	uint8_t reached_level = dev_bench_log_set_level(want_level);

	if (reached_level != want_level) {
		char note[DBM_MAX_LOG_LINE_LEN + 1];

		/* Reported only on a mismatch, and this is the honest case to
		 * report: the study asked for a verbosity this *build* does not
		 * contain, so the log it gets back will be quieter than it asked
		 * for and nothing else would say so. */
		snprintk(note, sizeof(note),
			 "study asked for log level %u; this build only reaches %u",
			 (unsigned int)want_level, (unsigned int)reached_level);
		send_log_line(note);
	}

	bool completed = true;

	transcript_dropped = 0;

	/* Taps before steps: a WholeStudy tap has to be open before the first
	 * action runs, or the first thing it was declared to capture is the
	 * thing it misses. */
	load_study_taps(study);
	sync_taps_for_step(0);

	for (uint32_t i = 0; i < study->steps_len; i++) {
		const struct dbm_step *step = &study->steps[i];

		/* **Both indices of a `RunProtocol` step, checked before they
		 * reach a C array subscript** (embarch-study-designer/design.md
		 * §3 decision 60, and `embarch-core/design.md` §3 decision 18's
		 * rule that a failure is named rather than left to fail as a
		 * raw index). Core's pre-flight already checks both against the
		 * same `Study`, so reaching this is either drift between the
		 * two or a corrupt link -- which is exactly why it is checked
		 * twice and why this one fails the step by name rather than
		 * trusting the other end. */
		if (step->action_tag == DBM_ACTION_RUN_PROTOCOL &&
		    !protocol_index_valid(study, step, i)) {
			completed = false;
			break;
		}

		struct action action = step_to_action(study, step);

		/* At DBG rather than INF: a study that asked for `Debug` is asking
		 * to follow the run step by step, and one that did not should not
		 * pay a `LogLine` per step for it (decision 39). This is also the
		 * bench's own answer to "where did it get to" for a study that
		 * stops producing `StepResult`s -- the last line in the debug file
		 * names the step that was executing. */
		LOG_DBG("step %u '%s': dispatching", (unsigned int)i, step->name);

		/* Stamped before the step runs, so every transcript entry it
		 * produces -- including notifications arriving inside a capture
		 * window opened by an *earlier* step -- is attributed to the
		 * step actually executing (design.md §3 decision 36). */
		transcript_step_index = i;

		/* Windows that start or end at this step, applied before the
		 * delay rather than after it: `delay_before_ms` is authored
		 * time inside the step, and a tap scoped to step i is open for
		 * the whole of step i, wait included. */
		sync_taps_for_step(i);

		/* Step::delay_before_ms -- the study's authored "when"
		 * (embarch-study-designer/design.md §3 decision 42). Deliberately
		 * *inside* the loop and *after* transcript_step_index is stamped:
		 * anything the DUT sends unprompted during the wait belongs to this
		 * step in the transcript, which is what makes a delay useful for
		 * separating unsolicited traffic from a response to the write that
		 * follows it. Not deducted from timeout_ms, which still bounds the
		 * action alone. */
		if (step->delay_before_ms > 0) {
			k_sleep(K_MSEC(step->delay_before_ms));
		}

		struct outcome bridge_outcome = ble_bridge_execute(&action, step->timeout_ms);

		send_step_result(i, step->name, &bridge_outcome);

		if (bridge_outcome.kind != OUTCOME_PASS && !step->continue_on_fail) {
			completed = false;
			break;
		}
	}

	/* A study that opened a capture window and never closed it -- either
	 * because its author left the GattMonitorStop off, or because an
	 * earlier step aborted the run before reaching it -- gets it closed
	 * here rather than leaving subscriptions armed into the next study
	 * (design.md §3 decision 36). */
	if (ble_bridge_monitor_window_open()) {
		struct action close = {.kind = ACTION_GATT_MONITOR_STOP};

		(void)ble_bridge_execute(&close, 1000);
		send_log_line("closed a GATT capture window the study left open");
	}

	/* Drain whatever the TX thread hasn't sent yet before StudyDone, so
	 * Core never sees the study end with transcript entries still in
	 * flight behind it. */
	while (k_msgq_num_used_get(&transcript_q) > 0) {
		k_sleep(K_MSEC(5));
	}

	/* One past the last step: no scope covers it, so this closes whatever
	 * is still open -- including a study that ended early on a failed
	 * step. Deliberately after the drain above, so every record a tap
	 * produced is on the wire before its own StreamClose is, and Core
	 * never has to keep bytes that arrived after the close it already
	 * saw. `transcript_dropped` is final by now for the same reason. */
	sync_taps_for_step(study->steps_len);

	/* **A study is a bond's lifetime** (embarch-dev-bench/design.md §3
	 * decision 37). Cleared here so a second run of a study behaves like
	 * the first -- a bench that silently pairs differently on the second
	 * run is the failure study-scoped bonding exists to avoid, and it is
	 * the reading the repo owner asked for by name.
	 *
	 * Before the transcript drain below rather than after it: clearing a
	 * bond disconnects the peer (Zephyr's bt_unpair does), and the
	 * disconnect produces a transcript entry that deserves to reach Core
	 * with everything else rather than arriving after its own StreamClose.
	 *
	 * If "persist for a study" ever turns out to mean *across* studies,
	 * this call is the one line that moves. */
	ble_bridge_clear_bonds();

	if (transcript_dropped > 0) {
		char note[DBM_MAX_LOG_LINE_LEN + 1];

		snprintk(note, sizeof(note),
			 "GATT transcript dropped %u entries (queue full) -- this capture is "
			 "NOT exhaustive",
			 (unsigned int)transcript_dropped);
		send_log_line(note);
	}

	/* Back to idle verbosity before the study is declared over, so the next
	 * link -- or the next study that asks for nothing -- gets a quiet bench
	 * (decision 39). `Hello` reverts it too, for the case where a study
	 * never reaches this line at all. */
	dev_bench_log_set_level(DEV_BENCH_LOG_BOOT_LEVEL);

	send_study_done(completed);
}

int main(void)
{
	/* Before the first record: hold every Zephyr source at the idle level
	 * (decision 39). The runtime filter starts at whatever the build
	 * compiled in -- now DBG for everything, so that a study *can* ask for
	 * it -- and leaving it there would make an idle bench chattier than
	 * decision 38's compiled-out ceiling ever was. This app's own module
	 * stays at INF regardless, which is what lets the boot line below
	 * through (dev_bench_log.h). */
	dev_bench_log_set_level(DEV_BENCH_LOG_BOOT_LEVEL);

	/* The first record of a boot, and (until Core's `Hello` arrives) held in
	 * dev_bench_log.c's backlog rather than written to a wire nobody has
	 * open yet. This is what makes "did this bench reboot mid-study" a
	 * question the debug file can answer -- see handle_hello's own comment on
	 * the 2026-08-26 study that prompted it, and note CONFIG_BOOT_BANNER
	 * stays off (prj.conf) so the banner's raw printk can never reach this
	 * link ahead of the logging subsystem. */
	LOG_INF("dev-bench up: fw %s, wire schema v%u", APP_FIRMWARE_VERSION,
		(unsigned int)study_ffi_schema_version());

	if (ble_bridge_init() != 0) {
		LOG_ERR("ble_bridge_init failed; this bench cannot run a study");
		return -1;
	}

	ble_bridge_set_transcript_sink(transcript_sink_cb, NULL);
	/* Safe to write the link directly from this sink, unlike the transcript
	 * one: ble_bridge.h's `ble_log_sink` contract is that it's only ever
	 * called from this thread, inside ble_bridge_execute. */
	ble_bridge_set_log_sink(log_sink_cb, NULL);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Arm the ISR that keeps the hardware FIFO drained -- see
	 * `link_rx_ring`'s own comment for why the dispatch loop must not be
	 * the thing reading the FIFO. Failing to install it would silently
	 * reintroduce that bug, so it's a hard error rather than a warning. */
	if (uart_irq_callback_user_data_set(link_uart, link_uart_isr, NULL) != 0) {
		return -1;
	}
	uart_irq_rx_enable(link_uart);
#endif

	/* Whether the most recent Hello/HelloAck handshake was schema-compatible
	 * and dev-bench is now expecting the StudyStart that follows it
	 * (embarch-study-designer/design.md §3 decision 24: Core sends it exactly
	 * once, immediately after that handshake completes). A fresh Hello
	 * arriving before a StudyStart does (Core resetting again) is handled
	 * like any other Hello rather than being dropped as "unexpected" here —
	 * see the DBM_TAG_HELLO case below. */
	bool awaiting_study = false;

	while (1) {
		/* `static`: see send_message's own comment on why a
		 * `struct dev_bench_message` shouldn't be a stack local here. */
		static struct dev_bench_message msg;

		if (receive_message(&msg) != 0) {
			continue;
		}

		switch (msg.tag) {
		case DBM_TAG_HELLO:
			awaiting_study = handle_hello(&msg.hello);
			break;
		case DBM_TAG_STUDY_START:
			if (awaiting_study) {
				dispatch_study(&msg.study_start);
			} else {
				send_log_line("StudyStart received without a preceding Hello "
					      "handshake; ignoring");
			}
			awaiting_study = false;
			break;
		default:
			/* Core only ever sends Hello then StudyStart on this link
			 * (embarch-study-designer/design.md §3 decisions 12/24) —
			 * anything else here is unexpected; ignore rather than
			 * misbehave. */
			break;
		}
	}
	return 0;
}
