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
	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_LOG_LINE;
	strncpy(msg.log_line.text, text, DBM_MAX_LOG_LINE_LEN);
	msg.log_line.text[DBM_MAX_LOG_LINE_LEN] = '\0';
	send_message(&msg);
}

static void send_study_done(bool completed)
{
	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_STUDY_DONE;
	msg.study_done.completed = completed;
	send_message(&msg);
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
	static struct dev_bench_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.tag = DBM_TAG_STEP_RESULT;
	msg.step_result.step_index = step_index;
	strncpy(msg.step_result.result.step_name, step_name, DBM_MAX_NAME_LEN);
	msg.step_result.result.step_name[DBM_MAX_NAME_LEN] = '\0';

	switch (bridge_outcome->kind) {
	case OUTCOME_PASS:
		msg.step_result.result.outcome.tag = 0;
		break;
	case OUTCOME_FAIL:
		msg.step_result.result.outcome.tag = 1;
		strncpy(msg.step_result.result.outcome.fail_reason, bridge_outcome->fail_reason,
			DBM_MAX_FAIL_REASON_LEN);
		msg.step_result.result.outcome.fail_reason[DBM_MAX_FAIL_REASON_LEN] = '\0';
		break;
	case OUTCOME_TIMED_OUT:
	default:
		msg.step_result.result.outcome.tag = 2;
		break;
	}

	if (bridge_outcome->captured_data != NULL && bridge_outcome->captured_len > 0) {
		size_t len = bridge_outcome->captured_len;

		if (len > sizeof(msg.step_result.result.captured_data)) {
			len = sizeof(msg.step_result.result.captured_data);
		}
		memcpy(msg.step_result.result.captured_data, bridge_outcome->captured_data, len);
		msg.step_result.result.captured_data_len = (uint32_t)len;
		msg.step_result.result.has_captured_data = true;
	}

	send_message(&msg);
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
	static struct dev_bench_message ack;

	memset(&ack, 0, sizeof(ack));
	ack.tag = DBM_TAG_HELLO_ACK;
	ack.hello_ack.schema_version = our_schema;
	ack.hello_ack.compatible = (hello->schema_version == our_schema);
	strncpy(ack.hello_ack.firmware_version, APP_FIRMWARE_VERSION, DBM_MAX_FIRMWARE_VERSION_LEN);
	ack.hello_ack.firmware_version[DBM_MAX_FIRMWARE_VERSION_LEN] = '\0';
	send_message(&ack);

	if (!ack.hello_ack.compatible) {
		send_log_line("schema version mismatch, not awaiting a StudyStart");
		return false;
	}
	return true;
}

/* Real per-`Study` dispatch (embarch-dev-bench/design.md §3 decision 21),
 * scoped to `Action::BleAdvertise` steps only for this pass —
 * `study`/`serial_protocol.c`'s own decode already rejected any other action
 * kind whole (`has_unsupported_action`) rather than partially decoding it, so
 * by the time a `struct dbm_study_start` reaches here every step in
 * `study->steps[0..study->steps_len)` is a `BleAdvertise` action.
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
		send_log_line("StudyStart contains a step whose action isn't BleAdvertise "
			      "(unsupported for now); aborting without running any step");
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
		struct action action = {
			.kind = ACTION_BLE_ADVERTISE,
			.advertise =
				{
					.has_local_name = step->action.has_local_name,
					.service_uuid_count = 0, /* not carried this far, decision 21 */
					.adv_interval_ms = step->action.adv_interval_ms,
				},
		};

		strncpy(action.advertise.local_name, step->action.local_name, BLE_MAX_LOCAL_NAME_LEN);
		action.advertise.local_name[BLE_MAX_LOCAL_NAME_LEN] = '\0';

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
