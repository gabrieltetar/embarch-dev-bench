/* Internal API mirroring embarch-study-designer's Action/GattOperation surface
 * (embarch-study-designer/src/study.rs) — the boundary decision 16 splits into two
 * implementations sharing one signature set:
 *   - ble_bridge_real.c: real Zephyr BT host calls (workspaces/nordic/)
 *   - ble_bridge_stub.c: canned Outcomes, no BLE host (workspaces/native_sim/)
 *
 * embarch-dev-bench/design.md §1, §3 decision 16. Field-level Study/Step
 * decoding doesn't exist yet (study_ffi.c is a stub, decision 20) — nothing
 * currently constructs a `struct action` from real wire data. This header
 * exists so main.c's fixed bring-up demo sequence (decision 20's "even
 * against fixed/fake decode results") has a real boundary to call through,
 * matching the shape that a future real Study-dispatch loop will use.
 */
#ifndef EMBARCH_DEV_BENCH_BLE_BRIDGE_H_
#define EMBARCH_DEV_BENCH_BLE_BRIDGE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum ble_role {
	BLE_ROLE_CENTRAL,
	BLE_ROLE_PERIPHERAL,
};

/* Mirrors embarch-study-designer's `BleAddressKind` (src/ids.rs). */
enum ble_address_kind {
	BLE_ADDR_PUBLIC,
	BLE_ADDR_RANDOM,
};

enum gatt_operation_kind {
	GATT_OP_READ,
	GATT_OP_WRITE,
	GATT_OP_NOTIFY,
	GATT_OP_INDICATE,
	GATT_OP_SUBSCRIBE,
	GATT_OP_STREAM_CAPTURE,
};

struct gatt_operation {
	enum gatt_operation_kind kind;
	union {
		struct {
			const uint8_t *payload;
			size_t payload_len;
		} write;
		struct {
			uint32_t timeout_ms;
		} notify;
		struct {
			uint32_t timeout_ms;
		} indicate;
	};
};

#define BLE_MAX_LOCAL_NAME_LEN 26 /* mirrors limits::MAX_LOCAL_NAME_LEN */
#define BLE_MAX_SERVICE_UUIDS 4   /* mirrors limits::MAX_SERVICE_UUIDS */
#define BLE_MAX_PAYLOAD_LEN 512   /* mirrors limits::MAX_PAYLOAD_LEN */

struct ble_advertise_params {
	bool has_local_name;
	char local_name[BLE_MAX_LOCAL_NAME_LEN + 1];
	uint8_t service_uuid_count;
	uint8_t service_uuids[BLE_MAX_SERVICE_UUIDS][16];
	uint16_t adv_interval_ms;
};

/* `target_address`/`service_uuid`/`characteristic_uuid` bytes are in
 * embarch-study-designer's own order (src/ids.rs): UUIDs big-endian, matching
 * the Bluetooth SIG's on-the-wire base-UUID byte order, and addresses in the
 * same display order (AA:BB:CC:DD:EE:FF, most significant byte first). Zephyr's
 * `bt_uuid_create`/`bt_addr_le_t` both want little-endian, so ble_bridge_real.c
 * reverses on the way in — the crate's own docs state this explicitly for
 * `Uuid` but not for `BleAddress` (embarch-dev-bench/design.md §4). */
struct ble_connect_params {
	enum ble_role role;
	bool has_target_address;
	enum ble_address_kind target_address_kind;
	uint8_t target_address[6];
};

struct data_exchange_params {
	uint8_t service_uuid[16];
	uint8_t characteristic_uuid[16];
	struct gatt_operation operation;
};

enum action_kind {
	ACTION_BLE_ADVERTISE,
	ACTION_BLE_CONNECT,
	ACTION_DATA_EXCHANGE,
};

struct action {
	enum action_kind kind;
	union {
		struct ble_advertise_params advertise;
		struct ble_connect_params connect;
		struct data_exchange_params data_exchange;
	};
};

enum outcome_kind {
	OUTCOME_PASS,
	OUTCOME_FAIL,
	OUTCOME_TIMED_OUT,
};

#define OUTCOME_MAX_FAIL_REASON_LEN 64 /* mirrors limits::MAX_FAIL_REASON_LEN */

struct outcome {
	enum outcome_kind kind;
	char fail_reason[OUTCOME_MAX_FAIL_REASON_LEN + 1]; /* valid when kind == OUTCOME_FAIL */
	/* Bytes a DataExchange read/notify/indicate pulled off the DUT, mirroring
	 * `StepResult.captured_data` (embarch-study-designer/src/result.rs); NULL
	 * for every action that captures nothing.
	 *
	 * A borrow into a bridge-owned static buffer, valid only until the next
	 * ble_bridge_execute()/ble_bridge_reset() call — deliberately not an inline
	 * BLE_MAX_PAYLOAD_LEN array, which would make every `struct outcome` return
	 * a 512-byte stack copy. */
	const uint8_t *captured_data;
	size_t captured_len;
};

/* Brings up the BLE bridge (Zephyr BT host enable for _real; a no-op for
 * _stub). Returns 0 on success. Call once at boot, before ble_bridge_execute. */
int ble_bridge_init(void);

/* Executes one action synchronously, bounded by timeout_ms (embarch-dev-bench/design.md
 * §1's Step.timeout_ms), returning its device-observed Outcome. */
struct outcome ble_bridge_execute(const struct action *action, uint32_t timeout_ms);

/* Sink for GATT_OP_STREAM_CAPTURE's continuous notifications, which — unlike
 * every other GattOperation — produce samples throughout the step rather than
 * one captured value at the end (embarch-study-designer/src/study.rs's
 * `GattOperation::StreamCapture`, landing in the `SensorWaveform` channel).
 *
 * `data`/`len` are one notification's raw ATT value, borrowed for the duration
 * of the call only. Invoked from Zephyr's BT RX thread, not from whichever
 * thread called ble_bridge_execute — a handler must not block. No handler
 * registered (the current default: nothing wires one up yet, since how raw
 * notification bytes map onto `Sample.value`'s single f32 is still open,
 * embarch-dev-bench/design.md §4) means captured samples are counted and
 * dropped, not buffered. */
typedef void (*ble_stream_sample_handler)(const uint8_t *data, size_t len, void *user_data);
void ble_bridge_set_stream_handler(ble_stream_sample_handler handler, void *user_data);

/* embarch-dev-bench/design.md §3 decision 11: a fresh Hello unconditionally
 * clears dev-bench's in-RAM BT bonding table (Just Works pairing only, never
 * persisted to flash — not enabling CONFIG_BT_SETTINGS already keeps bonds
 * RAM-only; this clears them explicitly rather than letting them accumulate
 * across a session's repeated Hello handshakes). No-op for _stub, which has
 * no bonding table to clear. */
void ble_bridge_reset(void);

#endif /* EMBARCH_DEV_BENCH_BLE_BRIDGE_H_ */
