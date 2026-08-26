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
 * output must NOT also be routed to this device (each workspace's prj.conf
 * disables that) — sharing the wire with raw log text would corrupt COBS
 * framing (decision 7); dev-bench's own log output travels as a `LogLine`
 * DevBenchMessage instead.
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

#include "ble_bridge.h"
#include "serial_protocol.h"
#include "study_ffi.h"

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
	/* `static`, not a stack local: `struct dev_bench_message`'s union is
	 * sized by its largest member (`struct dbm_study_start`, several KB —
	 * see serial_protocol.h), regardless of which tag this particular call
	 * actually uses. Safe because every writer holds link_tx_mutex. */
	static uint8_t frame[DBM_MAX_FRAME_LEN];
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

static void transcript_sink_cb(const struct ble_transcript_entry *entry, void *user_data)
{
	ARG_UNUSED(user_data);

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

		memset(msg, 0, sizeof(*msg));
		msg->tag = DBM_TAG_GATT_TRANSCRIPT_RECORD;
		msg->gatt_transcript.step_index = item.step_index;
		msg->gatt_transcript.entry.rx_utc_ms = item.entry.rx_utc_ms;
		msg->gatt_transcript.entry.direction = item.entry.direction;
		msg->gatt_transcript.entry.kind = item.entry.kind;
		msg->gatt_transcript.entry.has_service_uuid = item.entry.has_service_uuid;
		memcpy(msg->gatt_transcript.entry.service_uuid, item.entry.service_uuid, 16);
		msg->gatt_transcript.entry.has_characteristic_uuid =
			item.entry.has_characteristic_uuid;
		memcpy(msg->gatt_transcript.entry.characteristic_uuid,
		       item.entry.characteristic_uuid, 16);
		msg->gatt_transcript.entry.att_status = item.entry.att_status;
		msg->gatt_transcript.entry.payload_len = item.entry.payload_len;
		memcpy(msg->gatt_transcript.entry.payload, item.entry.payload,
		       item.entry.payload_len);

		send_message_locked(msg);
		k_mutex_unlock(&link_tx_mutex);
	}
}

K_THREAD_DEFINE(transcript_tx_tid, 2048, transcript_tx_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(7), 0, 0);

static void send_log_line(const char *text)
{
	struct dev_bench_message *msg = &tx_scratch;

	k_mutex_lock(&link_tx_mutex, K_FOREVER);
	memset(msg, 0, sizeof(*msg));
	msg->tag = DBM_TAG_LOG_LINE;
	strncpy(msg->log_line.text, text, DBM_MAX_LOG_LINE_LEN);
	msg->log_line.text[DBM_MAX_LOG_LINE_LEN] = '\0';
	send_message_locked(msg);
	k_mutex_unlock(&link_tx_mutex);
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

	/* gatt_services/gatt_activity (design.md §3 decisions 31/32) --
	 * populated by GattDiscover (services only) and GattMonitorAll (both);
	 * NULL/0 from ble_bridge_execute() for every other action kind, same
	 * borrowed-pointer lifetime as captured_data above. */
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
	if (bridge_outcome->gatt_activity != NULL && bridge_outcome->gatt_activity_count > 0) {
		size_t count = bridge_outcome->gatt_activity_count;

		if (count > DBM_MAX_GATT_ACTIVITY_RECORDS) {
			count = DBM_MAX_GATT_ACTIVITY_RECORDS;
		}
		for (size_t a = 0; a < count; a++) {
			const struct ble_gatt_activity_record *src = &bridge_outcome->gatt_activity[a];
			struct dbm_gatt_activity_record *dst = &msg->step_result.result.gatt_activity[a];

			dst->rx_utc_ms = src->rx_utc_ms;
			dst->characteristic_index = src->characteristic_index;
			size_t payload_len = src->payload_len;

			if (payload_len > sizeof(dst->payload)) {
				payload_len = sizeof(dst->payload);
			}
			memcpy(dst->payload, src->payload, payload_len);
			dst->payload_len = (uint32_t)payload_len;
		}
		msg->step_result.result.gatt_activity_len = (uint32_t)count;
		msg->step_result.result.has_gatt_activity = true;
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
	static uint8_t rx_buf[DBM_MAX_FRAME_LEN];
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
			rx_len = 0; /* overflow: drop and resync on the next delimiter */
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

	if (!compatible) {
		send_log_line("schema version mismatch, not awaiting a StudyStart");
		return false;
	}
	return true;
}

/* Translates one decoded `struct dbm_step`'s action into the `struct action`
 * ble_bridge.h's shared BleAdvertise/BleConnect/DataExchange/GattDiscover/
 * GattMonitorAll surface expects — a direct field-by-field mapping per kind,
 * decision 21's own original BleAdvertise-only translation extended to cover
 * decisions 31/32's two new kinds and the BleConnect/DataExchange dispatch
 * that had never actually been wired up to a real decoded `Study` before
 * this pass (`ble_bridge_real.c` itself has implemented both since
 * design.md §3 decision 16's own implementation note — this is the missing
 * wiring, not new BLE logic).
 */
static struct action step_to_action(const struct dbm_step *step)
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
 * (decisions 31/32 close the BleAdvertise-only scope this originally
 * shipped with). `study`/`serial_protocol.c`'s own decode already rejected
 * any action kind it doesn't recognize at all (`has_unsupported_action`)
 * rather than partially decoding it, so by the time a `struct
 * dbm_study_start` reaches here every step in
 * `study->steps[0..study->steps_len)` is one of today's five known kinds.
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

	bool completed = true;

	transcript_dropped = 0;

	for (uint32_t i = 0; i < study->steps_len; i++) {
		const struct dbm_step *step = &study->steps[i];
		struct action action = step_to_action(step);

		/* Stamped before the step runs, so every transcript entry it
		 * produces -- including notifications arriving inside a capture
		 * window opened by an *earlier* step -- is attributed to the
		 * step actually executing (design.md §3 decision 36). */
		transcript_step_index = i;

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

	if (transcript_dropped > 0) {
		char note[DBM_MAX_LOG_LINE_LEN + 1];

		snprintk(note, sizeof(note),
			 "GATT transcript dropped %u entries (queue full) -- this capture is "
			 "NOT exhaustive",
			 (unsigned int)transcript_dropped);
		send_log_line(note);
	}

	send_study_done(completed);
}

int main(void)
{
	if (ble_bridge_init() != 0) {
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
