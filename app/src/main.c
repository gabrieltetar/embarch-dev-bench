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
#include <zephyr/kernel.h>

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

static void send_message(const struct dev_bench_message *msg)
{
	/* `static`, not a stack local: `struct dev_bench_message`'s union is
	 * sized by its largest member (`struct dbm_study_start`, several KB —
	 * see serial_protocol.h), regardless of which tag this particular call
	 * actually uses. Safe because the link is driven from this one
	 * single-threaded RX/dispatch loop (design.md §3 decision 20). */
	static uint8_t frame[DBM_MAX_FRAME_LEN];
	int frame_len = dbm_encode_frame(msg, frame, sizeof(frame));

	if (frame_len < 0) {
		return;
	}
	for (int i = 0; i < frame_len; i++) {
		uart_poll_out(link_uart, frame[i]);
	}
}

static void send_log_line(const char *text)
{
	struct dev_bench_message *msg = &tx_scratch;

	memset(msg, 0, sizeof(*msg));
	msg->tag = DBM_TAG_LOG_LINE;
	strncpy(msg->log_line.text, text, DBM_MAX_LOG_LINE_LEN);
	msg->log_line.text[DBM_MAX_LOG_LINE_LEN] = '\0';
	send_message(msg);
}

static void send_study_done(bool completed)
{
	struct dev_bench_message *msg = &tx_scratch;

	memset(msg, 0, sizeof(*msg));
	msg->tag = DBM_TAG_STUDY_DONE;
	msg->study_done.completed = completed;
	send_message(msg);
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

	send_message(msg);
}

/* Blocks (polling) until one full COBS frame has been read off the link UART
 * and decoded. Malformed frames are dropped silently and the reader resyncs
 * on the next 0x00 delimiter, per COBS's own resync property
 * (embarch-study-designer/design.md §3 decision 10). */
static int receive_message(struct dev_bench_message *out)
{
	static uint8_t rx_buf[DBM_MAX_FRAME_LEN];
	size_t rx_len = 0;

	while (1) {
		uint8_t byte;

		if (uart_poll_in(link_uart, &byte) != 0) {
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
static bool handle_hello(const struct dbm_hello *hello)
{
	ble_bridge_reset();

	uint32_t our_schema = study_ffi_schema_version();
	struct dev_bench_message *ack = &tx_scratch;

	memset(ack, 0, sizeof(*ack));
	ack->tag = DBM_TAG_HELLO_ACK;
	ack->hello_ack.schema_version = our_schema;
	ack->hello_ack.compatible = (hello->schema_version == our_schema);
	strncpy(ack->hello_ack.firmware_version, APP_FIRMWARE_VERSION, DBM_MAX_FIRMWARE_VERSION_LEN);
	ack->hello_ack.firmware_version[DBM_MAX_FIRMWARE_VERSION_LEN] = '\0';
	bool compatible = ack->hello_ack.compatible;

	send_message(ack);

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

	bool completed = true;

	for (uint32_t i = 0; i < study->steps_len; i++) {
		const struct dbm_step *step = &study->steps[i];
		struct action action = step_to_action(step);
		struct outcome bridge_outcome = ble_bridge_execute(&action, step->timeout_ms);

		send_step_result(i, step->name, &bridge_outcome);

		if (bridge_outcome.kind != OUTCOME_PASS && !step->continue_on_fail) {
			completed = false;
			break;
		}
	}

	send_study_done(completed);
}

int main(void)
{
	if (ble_bridge_init() != 0) {
		return -1;
	}

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
