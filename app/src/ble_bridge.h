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
	/* Connect only to an advertiser whose advertised local name equals
	 * this exactly (embarch-study-designer/design.md §3 decision 43).
	 * `has_target_name == false` restores the pre-v7 "first connectable
	 * advertiser wins" behavior. Combined with `has_target_address` by AND:
	 * if both are set, both must match. */
	bool has_target_name;
	char target_name[BLE_MAX_LOCAL_NAME_LEN + 1];
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
	/* Both field-less (embarch-study-designer/src/study.rs's `Action::GattDiscover {}`/
	 * `Action::GattMonitorAll {}`) -- no params struct needed, matching design.md §3
	 * decisions 31/32's own "simplest possible" framing. */
	ACTION_GATT_DISCOVER,
	ACTION_GATT_MONITOR_ALL,
	/* design.md §3 decision 36 -- also field-less. Unlike
	 * ACTION_GATT_MONITOR_ALL, which subscribes and tears down inside one
	 * step, these two open and close a capture window that spans the steps
	 * between them, so a DataExchange write can stimulate the DUT while the
	 * capture is live. */
	ACTION_GATT_MONITOR_START,
	ACTION_GATT_MONITOR_STOP,
};

struct action {
	enum action_kind kind;
	union {
		struct ble_advertise_params advertise;
		struct ble_connect_params connect;
		struct data_exchange_params data_exchange;
		/* ACTION_GATT_DISCOVER/ACTION_GATT_MONITOR_ALL/
		 * ACTION_GATT_MONITOR_START/ACTION_GATT_MONITOR_STOP carry no
		 * params. */
	};
};

/* Mirrors embarch-study-designer's `GattCharacteristicInfo`/`GattServiceInfo`
 * (src/gatt.rs, design.md §4.3a) -- `properties` is the raw ATT
 * characteristic-properties byte, passed through unchanged (that decision's
 * "raw, not symbolic" stance). UUID byte order matches every other UUID in
 * this header: big-endian, embarch-study-designer's own convention. */
#define BLE_MAX_DISCOVERED_SERVICES 8 /* mirrors limits::MAX_DISCOVERED_SERVICES */
#define BLE_MAX_CHARS_PER_SERVICE 16  /* mirrors limits::MAX_CHARS_PER_SERVICE */
#define BLE_MAX_GATT_ACTIVITY_RECORDS 32 /* mirrors limits::MAX_GATT_ACTIVITY_RECORDS */

struct ble_gatt_characteristic_info {
	uint8_t uuid[16];
	uint8_t properties;
};

struct ble_gatt_service_info {
	uint8_t uuid[16];
	struct ble_gatt_characteristic_info characteristics[BLE_MAX_CHARS_PER_SERVICE];
	uint8_t characteristics_len;
};

/* Mirrors `GattActivityRecord` (src/gatt.rs) -- `characteristic_index` indexes
 * into the same step's `gatt_services`, flattened service-then-characteristic
 * in discovery order (that type's own documented convention); this bridge
 * computes it directly against the `struct ble_gatt_service_info` array below,
 * so there is exactly one place that flattening happens. */
struct ble_gatt_activity_record {
	uint64_t rx_utc_ms;
	uint16_t characteristic_index;
	uint8_t payload[BLE_MAX_PAYLOAD_LEN];
	uint16_t payload_len;
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

	/* Populated by ACTION_GATT_DISCOVER/ACTION_GATT_MONITOR_ALL, mirroring
	 * `StepResult.gatt_services` (embarch-study-designer/src/result.rs, design.md
	 * §3 decisions 31/32) -- a borrow into a bridge-owned static buffer, same
	 * posture and same lifetime rule as `captured_data` above. NULL/0 for every
	 * other action kind. */
	const struct ble_gatt_service_info *gatt_services;
	size_t gatt_service_count;
	/* Populated only by ACTION_GATT_MONITOR_ALL, mirroring `StepResult.gatt_activity`.
	 * NULL/0 for every other action kind, including ACTION_GATT_DISCOVER. */
	const struct ble_gatt_activity_record *gatt_activity;
	size_t gatt_activity_count;
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

/* ---- GATT transcript (design.md §3 decision 36) ------------------------ */

/* Largest payload this bridge ever puts in a transcript entry -- one full ATT
 * MTU notification. Mirrors serial_protocol.h's
 * DBM_MAX_TRANSCRIPT_PAYLOAD_LEN; kept as its own constant here so this
 * header stays independent of the serial layer, the same way
 * BLE_MAX_PAYLOAD_LEN already mirrors DBM_MAX_PAYLOAD_LEN. */
#define BLE_MAX_TRANSCRIPT_PAYLOAD_LEN 244

/* Mirrors `GattDirection`/`GattEventKind` (embarch-study-designer
 * src/gatt.rs). Values must match those enums' declaration order --
 * main.c passes them straight through to the wire without remapping. */
enum ble_gatt_direction {
	BLE_GATT_DIR_OUT = 0,
	BLE_GATT_DIR_IN = 1,
	BLE_GATT_DIR_LOCAL = 2,
};

enum ble_gatt_event_kind {
	BLE_GATT_EVT_CONNECTED = 0,
	BLE_GATT_EVT_DISCONNECTED = 1,
	BLE_GATT_EVT_DISCOVERY_STARTED = 2,
	BLE_GATT_EVT_SERVICE_DISCOVERED = 3,
	BLE_GATT_EVT_CHARACTERISTIC_DISCOVERED = 4,
	BLE_GATT_EVT_SUBSCRIBED = 5,
	BLE_GATT_EVT_UNSUBSCRIBED = 6,
	BLE_GATT_EVT_WRITE_REQUEST = 7,
	BLE_GATT_EVT_WRITE_RESPONSE = 8,
	BLE_GATT_EVT_READ_REQUEST = 9,
	BLE_GATT_EVT_READ_RESPONSE = 10,
	BLE_GATT_EVT_NOTIFICATION = 11,
	BLE_GATT_EVT_INDICATION = 12,
	BLE_GATT_EVT_ERROR = 13,
};

/* One transcript entry, mirroring `GattTranscriptEntry` (src/gatt.rs, §4.3b).
 * Passed by pointer to the sink and copied there -- the bridge does not keep
 * it alive past the call. */
struct ble_transcript_entry {
	uint64_t rx_utc_ms;
	uint8_t direction; /* enum ble_gatt_direction */
	uint8_t kind;      /* enum ble_gatt_event_kind */
	bool has_service_uuid;
	uint8_t service_uuid[16];
	bool has_characteristic_uuid;
	uint8_t characteristic_uuid[16];
	uint8_t att_status;
	uint16_t payload_len;
	uint8_t payload[BLE_MAX_TRANSCRIPT_PAYLOAD_LEN];
};

/* Receives every GATT event this bridge observes, in either direction.
 *
 * Invoked from whichever context produced the event -- Zephyr's BT RX thread
 * for anything inbound, the caller's own thread for anything this bridge
 * initiated -- so a sink **must not block** and must not write to the link
 * UART directly (main.c's own sink enqueues instead, and a dedicated thread
 * drains it; see design.md §3 decision 36). With no sink registered, events
 * are simply not recorded: the transcript is an observability feature, never
 * a precondition for a step running. */
typedef void (*ble_transcript_sink)(const struct ble_transcript_entry *entry, void *user_data);
void ble_bridge_set_transcript_sink(ble_transcript_sink sink, void *user_data);

/* Receives a human-readable diagnostic line from the bridge.
 *
 * Unlike the transcript sink above, this is **only ever invoked from the
 * caller's own thread** (the dispatch loop, inside ble_bridge_execute), never
 * from Zephyr's BT RX thread -- so main.c's sink may write the link UART
 * directly. That restriction is the whole reason this is a separate hook
 * rather than a `BLE_GATT_EVT_*` transcript entry: the things worth reporting
 * here (what a failed scan actually saw) are collected in a callback but must
 * be *sent* from somewhere it's safe to block.
 *
 * Until this existed the bridge could only speak through an `Outcome`'s
 * 64-byte `fail_reason`, which is far too small for anything structured --
 * the first attempt at reporting a failed name match truncated the one name
 * that mattered. With no sink registered, lines are dropped; diagnostics are
 * never a precondition for a step running. */
/* Mirrors serial_protocol.h's DBM_MAX_LOG_LINE_LEN, which in turn mirrors
 * embarch-study-designer's limits::MAX_LOG_LINE_LEN -- duplicated rather than
 * included, to keep this BLE-facing header independent of the link protocol's
 * own header (nothing else here knows the wire format either). */
#define BLE_MAX_LOG_LINE_LEN 128

typedef void (*ble_log_sink)(const char *line, void *user_data);
void ble_bridge_set_log_sink(ble_log_sink sink, void *user_data);

/* True while a window opened by ACTION_GATT_MONITOR_START is still open --
 * main.c uses it to close an implicitly-left-open window when a study ends
 * (design.md §3 decision 36). */
bool ble_bridge_monitor_window_open(void);

/* embarch-dev-bench/design.md §3 decision 11: a fresh Hello unconditionally
 * clears dev-bench's in-RAM BT bonding table (Just Works pairing only, never
 * persisted to flash — not enabling CONFIG_BT_SETTINGS already keeps bonds
 * RAM-only; this clears them explicitly rather than letting them accumulate
 * across a session's repeated Hello handshakes). No-op for _stub, which has
 * no bonding table to clear. */
void ble_bridge_reset(void);

#endif /* EMBARCH_DEV_BENCH_BLE_BRIDGE_H_ */
