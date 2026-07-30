/* embarch-dev-bench firmware entry point.
 *
 * embarch-dev-bench/design.md §1, §2, §3 decisions 6/7/12/19/20. Shared
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
	uint8_t frame[DBM_MAX_FRAME_LEN];
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
	struct dev_bench_message msg = {.tag = DBM_TAG_LOG_LINE};

	strncpy(msg.log_line.text, text, DBM_MAX_LOG_LINE_LEN);
	msg.log_line.text[DBM_MAX_LOG_LINE_LEN] = '\0';
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

/* A fixed, illustrative BLE action plus a couple of fake power samples,
 * standing in for a real Study's steps. No DevBenchMessage variant carries an
 * actual Study payload yet (embarch-dev-bench/design.md §4's open item on
 * this) — this proves the serial -> dispatch -> BLE action -> result plumbing
 * against ble_bridge_real.c / ble_bridge_stub.c either way, matching decision
 * 20's "even against fixed/fake decode results" bring-up scope. Replace with
 * real per-Step dispatch once that wire gap and decision 8's FFI wiring both
 * close. */
static void run_demo_sequence(void)
{
	struct action advertise = {
		.kind = ACTION_BLE_ADVERTISE,
		.advertise =
			{
				.has_local_name = true,
				.local_name = "embarch-dev-bench",
				.service_uuid_count = 0,
				.adv_interval_ms = 100,
			},
	};

	send_log_line("demo: advertising");
	struct outcome outcome = ble_bridge_execute(&advertise, 5000);

	if (outcome.kind != OUTCOME_PASS) {
		send_log_line("demo: advertise step did not pass");
		return;
	}

	struct dev_bench_message start = {
		.tag = DBM_TAG_STREAM_START,
		.stream_start = {.step_index = 0, .channel = DBM_CHANNEL_POWER},
	};
	send_message(&start);

	for (int i = 0; i < 3; i++) {
		struct dev_bench_message chunk = {
			.tag = DBM_TAG_STREAM_CHUNK,
			.stream_chunk = {.rx_utc_ms = (uint64_t)k_uptime_get(), .value = 3.3f},
		};
		send_message(&chunk);
		k_sleep(K_MSEC(10));
	}

	struct dev_bench_message end = {
		.tag = DBM_TAG_STREAM_END,
		.stream_end = {.step_index = 0, .channel = DBM_CHANNEL_POWER},
	};
	send_message(&end);
	send_log_line("demo: complete");
}

int main(void)
{
	if (ble_bridge_init() != 0) {
		return -1;
	}

	while (1) {
		struct dev_bench_message hello;

		if (receive_message(&hello) != 0 || hello.tag != DBM_TAG_HELLO) {
			continue;
		}

		/* embarch-study-designer/design.md §3 decision 12 / embarch-dev-bench
		 * decision 11: Hello doubles as a hard reset. No in-progress study
		 * execution state exists yet to abort in this bring-up pass, but the
		 * BT bonding table clear applies unconditionally on every Hello. */
		ble_bridge_reset();

		uint32_t our_schema = study_ffi_schema_version();
		struct dev_bench_message ack = {
			.tag = DBM_TAG_HELLO_ACK,
			.hello_ack =
				{
					.schema_version = our_schema,
					.compatible = (hello.hello.schema_version == our_schema),
				},
		};
		strncpy(ack.hello_ack.firmware_version, APP_FIRMWARE_VERSION,
			DBM_MAX_FIRMWARE_VERSION_LEN);
		ack.hello_ack.firmware_version[DBM_MAX_FIRMWARE_VERSION_LEN] = '\0';
		send_message(&ack);

		if (!ack.hello_ack.compatible) {
			send_log_line("schema version mismatch, not running demo sequence");
			continue;
		}

		run_demo_sequence();
	}
	return 0;
}
