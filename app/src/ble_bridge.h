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

/* The `.eap` protocol manifest types (embarch-study-designer/design.md §3
 * decisions 58-62, §4.9). The one include this header has, and it is not the
 * link protocol's: eap.h is deliberately independent of serial_protocol.h for
 * exactly the reason BLE_MAX_PAYLOAD_LEN below is a duplicate rather than an
 * include -- nothing on this side of the boundary knows the wire format. */
#include "eap.h"

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
	/* embarch-study-designer/design.md §3 decisions 44/50. The first is
	 * the only action here that carries a field and does no GATT at all;
	 * the second is field-less and, uniquely among these, *drops the
	 * link* as a documented consequence (Zephyr's bt_unpair disconnects a
	 * peer whose keys it clears). */
	ACTION_BLE_SECURITY,
	ACTION_BLE_UNBOND,
	/* embarch-study-designer/design.md §3 decision 53 -- the same
	 * discovery-and-subscribe walk as ACTION_GATT_MONITOR_ALL/START,
	 * narrowed to the characteristics the study names. The first actions
	 * here to carry a *list*.
	 *
	 * A named characteristic the DUT doesn't have, or one that is neither
	 * notify- nor indicate-capable, **fails the step naming it** rather
	 * than being skipped: a study that named a characteristic has said it
	 * expects one, and a subscribe-to-everything walk's log-and-skip rule
	 * is right precisely because nothing there was named. */
	ACTION_GATT_MONITOR_SELECTED,
	ACTION_GATT_MONITOR_SELECTED_START,
	/* embarch-study-designer/design.md §3 decision 60 -- hand the link to a
	 * declared `.eap` state machine for the length of this step.
	 *
	 * **The only action here whose behavior is not fixed by this file.**
	 * Every other kind names a thing this bridge knows how to do; this one
	 * carries a manifest, and what it writes, what it waits for and when it
	 * gives up are all that manifest's. The bridge's job is the BLE half --
	 * discover the declared characteristics, subscribe to the ones frames
	 * arrive on, perform the writes the interpreter asks for, and feed
	 * arrivals and timer expiries back in. eap_interp.c owns the decisions. */
	ACTION_RUN_PROTOCOL,
};

/* Mirrors `GattTarget` (embarch-study-designer/src/gatt.rs) -- raw
 * big-endian UUIDs, this header's convention throughout. */
#define BLE_MAX_MONITOR_TARGETS 16 /* mirrors limits::MAX_MONITOR_TARGETS */

struct gatt_target {
	uint8_t service_uuid[16];
	uint8_t characteristic_uuid[16];
};

struct gatt_monitor_selected_params {
	struct gatt_target targets[BLE_MAX_MONITOR_TARGETS];
	size_t targets_len;
};

/* Mirrors `SecurityLevel` (embarch-study-designer/src/study.rs). Values are
 * the wire discriminants, NOT the spec's level numbers -- BLE_SECURITY_L1 is
 * 0. serial_protocol.h's `enum dbm_security_level` carries the same values
 * and `dbm_security_level_number()` is where the offset is undone; this
 * header keeps its own copy so it stays independent of the link protocol's,
 * the same way BLE_MAX_PAYLOAD_LEN already mirrors DBM_MAX_PAYLOAD_LEN. */
enum ble_security_level {
	BLE_SECURITY_L1 = 0,
	BLE_SECURITY_L2 = 1,
	BLE_SECURITY_L3 = 2,
	BLE_SECURITY_L4 = 3,
};

struct ble_set_security_params {
	uint8_t level; /* enum ble_security_level */
};

/* Mirrors `Action::RunProtocol { protocol, entry_state }`, with the index
 * already resolved: the caller (main.c) is the one holding the study, so it
 * does the `protocols[protocol]` lookup and the range check, and this side
 * receives the manifest itself.
 *
 * `def` is **borrowed** for the duration of one ble_bridge_execute() call,
 * the same contract `gatt_operation.write.payload` already has. That is
 * satisfied structurally rather than by care: it points into the decoded
 * `StudyStart` main.c holds for the whole study. */
struct run_protocol_params {
	const struct eap_protocol_def *def;
	uint8_t entry_state;
};

struct action {
	enum action_kind kind;
	union {
		struct ble_advertise_params advertise;
		struct ble_connect_params connect;
		struct data_exchange_params data_exchange;
		struct ble_set_security_params set_security;
		struct gatt_monitor_selected_params monitor_selected;
		struct run_protocol_params run_protocol;
		/* ACTION_GATT_DISCOVER/ACTION_GATT_MONITOR_ALL/
		 * ACTION_GATT_MONITOR_START/ACTION_GATT_MONITOR_STOP/
		 * ACTION_BLE_UNBOND carry no params. */
	};
};

/* Mirrors embarch-study-designer's `GattCharacteristicInfo`/`GattServiceInfo`
 * (src/gatt.rs, design.md §4.3a) -- `properties` is the raw ATT
 * characteristic-properties byte, passed through unchanged (that decision's
 * "raw, not symbolic" stance). UUID byte order matches every other UUID in
 * this header: big-endian, embarch-study-designer's own convention. */
#define BLE_MAX_DISCOVERED_SERVICES 8 /* mirrors limits::MAX_DISCOVERED_SERVICES */
#define BLE_MAX_CHARS_PER_SERVICE 16  /* mirrors limits::MAX_CHARS_PER_SERVICE */

struct ble_gatt_characteristic_info {
	uint8_t uuid[16];
	uint8_t properties;
};

struct ble_gatt_service_info {
	uint8_t uuid[16];
	struct ble_gatt_characteristic_info characteristics[BLE_MAX_CHARS_PER_SERVICE];
	uint8_t characteristics_len;
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
	/* `gatt_activity` was here, mirroring the `StepResult` field of the
	 * same name. Both are **retired** by
	 * embarch-study-designer/design.md §3 decision 54: a capped in-memory
	 * copy of a capture the tap pipeline already streams to Core uncapped.
	 * What a monitor step captured is now read out of the study's own
	 * `streams/` files, which is where all of it is rather than the first
	 * 32 records of it.
	 */

	/* The link's BLE security level when this action finished, mirroring
	 * `StepResult.security_level` (embarch-study-designer/src/result.rs,
	 * embarch-study-designer/design.md §3 decision 44).
	 *
	 * Set for **every** action kind, not just ACTION_BLE_SECURITY:
	 * whichever level the link was at is what makes a later step's failure
	 * legible ("disconnected during service discovery" at L1 and the same
	 * failure at L4 are different findings). `has_security_level == false`
	 * means there was no connection to ask about -- never "nobody
	 * looked". */
	bool has_security_level;
	uint8_t security_level; /* enum ble_security_level */

	/* What an ACTION_RUN_PROTOCOL step's state machine did, mirroring
	 * `StepResult.protocol: Option<ProtocolOutcome>`
	 * (embarch-study-designer/design.md §3 decision 62). `has_protocol ==
	 * false` for every other action kind, which is every action that
	 * existed before it.
	 *
	 * **Reported even when the step itself failed**, and separately from
	 * the step's own outcome above. The step can fail for a reason the
	 * machine never saw -- a dropped link, an exhausted `timeout_ms` -- and
	 * telling "the protocol reached its `failed` state" apart from "the
	 * protocol never finished" is the whole diagnostic value of recording a
	 * final state. There is deliberately no list of decoded values here:
	 * that is the shape decision 54 retired `gatt_activity` for, and
	 * decoded bytes reach a reader through the study's declared taps. */
	bool has_protocol;
	char protocol_final_state[EAP_MAX_STATE_NAME_LEN + 1];
	uint8_t protocol_outcome_tag; /* enum eap_outcome_tag */
	char protocol_fail_reason[OUTCOME_MAX_FAIL_REASON_LEN + 1];
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

/* Clears every bond in dev-bench's in-RAM BT bonding table
 * (embarch-dev-bench/design.md §3 decision 37 -- **a study is the bond's
 * lifetime**). main.c calls this at the end of every study so a second run
 * of a study behaves like the first, which is the failure mode study-scoped
 * bonding exists to avoid; ble_bridge_reset() below calls it too, since a
 * Hello is a hard reset.
 *
 * **This drops an active link.** Zephyr's bt_unpair disconnects a peer whose
 * keys it clears, and this waits for that disconnect rather than returning
 * while active_conn is still populated -- the same race ble_bridge_reset()
 * already had to fix once. No-op for _stub, which has no bonding table.
 *
 * Bonds were RAM-only before this and still are: CONFIG_BT_SETTINGS is
 * deliberately not enabled, so nothing survives a reboot either way. What
 * this adds is a *bounded* lifetime within one power cycle. */
void ble_bridge_clear_bonds(void);

/* embarch-dev-bench/design.md §3 decision 11: a fresh Hello unconditionally
 * clears dev-bench's in-RAM BT bonding table (never persisted to flash — not
 * enabling CONFIG_BT_SETTINGS already keeps bonds RAM-only; this clears them
 * explicitly rather than letting them accumulate across a session's repeated
 * Hello handshakes). No-op for _stub, which has no bonding table to clear.
 *
 * Decision 11's "Just Works pairing only" half is **superseded by decision
 * 37**: dev-bench now declares a DisplayYesNo-class IO capability and
 * auto-confirms, so an authored ACTION_BLE_SECURITY can reach an
 * authenticated key. Just Works is still what happens when the peer's own IO
 * capability forces it. */
void ble_bridge_reset(void);

#endif /* EMBARCH_DEV_BENCH_BLE_BRIDGE_H_ */
