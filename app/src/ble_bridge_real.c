/* Real Zephyr BT host calls, built by workspaces/nordic/ (embarch-dev-bench/design.md
 * §3 decision 16). Only calls stable, vendor-neutral Zephyr Bluetooth host
 * APIs (`bt_*`) per decision 3 -- nothing NCS-proprietary.
 *
 * Every ble_bridge_execute() call is synchronous from the caller's point of
 * view but asynchronous underneath: Zephyr's BT host reports connections, GATT
 * responses, and notifications on its own threads, so each action arms a
 * semaphore, issues the request, and waits on it against the step's own
 * deadline. `timeout_ms` therefore bounds the whole action -- discovery
 * included -- not just its final request, which is why `deadline` (not a
 * per-wait duration) is threaded through every helper below.
 *
 * One connection at a time (decision 15), one action at a time, so all the
 * Zephyr parameter structs Zephyr requires to outlive a call (discover/read/
 * write/subscribe params) are single static instances rather than per-call
 * allocations.
 *
 * Security IS handled here, as of decision 34 -- and this paragraph used to
 * say the opposite. It read: "NOT handled here, deliberately: elevating the
 * link to an encrypted/paired one. Zephyr's own ATT layer already re-runs a
 * request at higher security when a DUT answers with an authentication/
 * encryption ATT error, and decision 11's Just Works pairing needs no auth
 * callbacks, so an explicit bt_conn_set_security() call would only duplicate
 * that."
 *
 * Both halves of that turned out to be wrong for the case that matters. The
 * ATT-layer retry only fires for a DUT that answers with an ATT security
 * error; a DUT that *disconnects* instead reaches no retry at all. And Just
 * Works produces an **unauthenticated** key, which caps the link at Level 3
 * -- so no bt_conn_set_security(BT_SECURITY_L4) against that posture could
 * ever have succeeded, duplicate or not.
 *
 * What runs now (embarch-study-designer/design.md §3 decision 44,
 * embarch-dev-bench/design.md §3 decisions 34 and 37): auth callbacks that make this
 * bridge a DisplayYesNo-class device and auto-confirm the comparison value,
 * so LE Secure Connections Numeric Comparison gets selected and the
 * resulting key is authenticated. ACTION_BLE_SECURITY then asks for a
 * level and reports the level actually reached; ACTION_BLE_UNBOND drops the
 * bond.
 *
 * The IO-capability derivation this depends on was read out of this
 * workspace's own Zephyr (subsys/bluetooth/host/smp.c's get_io_capa) rather
 * than recalled: DisplayYesNo requires `passkey_display` **and**
 * `passkey_confirm` both non-NULL with LE SC available, and registering only
 * the confirm half yields NoInputNoOutput -- which the spec's own selection
 * matrix (gen_method_sc) then resolves to Just Works, silently
 * unauthenticated. That is why both are registered below and why
 * `passkey_entry` deliberately is not (it would make this KeyboardDisplay,
 * a different row of the same matrix).
 */
#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "ble_bridge.h"
#include "eap_interp.h"

/* How many characteristics ACTION_GATT_MONITOR_ALL can subscribe to
 * concurrently in one step -- a dev-bench-internal implementation cap, not
 * part of embarch-study-designer's own wire-type limits (those bound
 * `gatt_services`/`gatt_activity`'s *content*, design.md §3 decision 15's
 * update, not how many live subscriptions this bridge itself can juggle at
 * once). Sized to comfortably exceed any real DUT seen so far
 * (`reference-dut-fw` has 10 notify/indicate-capable characteristics
 * across its two services) with headroom, same placeholder-but-concrete
 * posture as every other size constant in this codebase -- not
 * BLE_MAX_DISCOVERED_SERVICES * BLE_MAX_CHARS_PER_SERVICE's own absolute
 * worst case (128), which would cost several extra KB of static RAM for a
 * count no real firmware plausibly reaches. A DUT that does exceed this is
 * handled the same way BLE_MAX_GATT_ACTIVITY_RECORDS' own overflow is
 * (design.md §3 decision 32's addendum): further characteristics are simply
 * not subscribed, not a hard failure. */
#define BLE_MAX_MONITOR_SUBSCRIPTIONS 32

/* ---- state ------------------------------------------------------------- */

/* Signalled by the connection callbacks; `conn_err` carries the HCI error (or
 * a synthesized one when connection creation itself failed to even start). */
static K_SEM_DEFINE(conn_sem, 0, 1);
/* Signalled when a discovery/read/write procedure completes. */
static K_SEM_DEFINE(gatt_sem, 0, 1);
/* Signalled when a subscribed characteristic pushes a value. Separate from
 * gatt_sem so a notification arriving mid-discovery can't be mistaken for that
 * discovery finishing. */
static K_SEM_DEFINE(notify_sem, 0, 1);
/* Signalled on disconnection. Lets the two places that wait for *absence* of a
 * link (a stream-capture window, dropping a wrong peer) block on an event
 * instead of polling shared state from another thread. */
static K_SEM_DEFINE(disconn_sem, 0, 1);
/* Signalled when a CCC write (subscribe or unsubscribe) has been answered, or
 * when the host reports the subscription dead. */
static K_SEM_DEFINE(ccc_sem, 0, 1);
/* Signalled by security_changed_cb -- a link's security level settling (or
 * failing to) is its own event, distinct from the connect that preceded it
 * and from any GATT procedure. */
static K_SEM_DEFINE(sec_sem, 0, 1);

static struct bt_conn *active_conn;
/* The reference bt_conn_le_create() handed back, held until the connect step
 * resolves — that's what makes cancelling a still-pending connection possible
 * (Zephyr cancels a create by disconnecting the connecting object). */
static struct bt_conn *pending_conn;
static uint8_t conn_err;
static bool advertising;
/* Set by disconnected_cb so a step waiting on a GATT response reports the lost
 * link immediately instead of sitting out its whole timeout. */
static bool link_lost;
/* ATT error code from the last completed GATT procedure; 0 means success. */
static uint8_t att_err;
/* ATT error from the last CCC (subscribe/unsubscribe) write. */
static uint8_t ccc_att_err;
/* Set when the host reports a subscription dead by calling notify with NULL
 * data — how a failed CCC discovery/write surfaces, since Zephyr doesn't route
 * that through the subscribe callback. */
static bool ccc_torn_down;

/* ---- security (design.md §3 decision 34) -------------------------------- */

/* What security_changed_cb last reported. `sec_level` is Zephyr's own
 * bt_security_t, not the wire enum -- the translation happens once, at the
 * outcome boundary. */
static bt_security_t sec_level;
static enum bt_security_err sec_err;
/* Set when pairing itself failed, as opposed to a security_changed that
 * simply landed lower than asked. Distinguishing the two is what lets a
 * fail_reason say "pairing failed" rather than only naming a level. */
static bool pairing_failed_seen;
static enum bt_security_err pairing_failed_reason;

static bt_addr_le_t scan_target;
static bool scan_target_set;
static bool scan_matched;

/* Captured value for a read/notify/indicate, borrowed by `struct outcome`
 * until the next execute/reset call (see ble_bridge.h). */
static uint8_t captured[BLE_MAX_PAYLOAD_LEN];
static size_t captured_len;

static ble_stream_sample_handler stream_handler;
static void *stream_user_data;
/* True only while a GATT_OP_STREAM_CAPTURE step is running, which is what
 * routes notifications to stream_handler instead of into `captured`. */
static bool streaming;

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_discover_params ccc_discover_params;
static struct bt_gatt_read_params read_params;
static struct bt_gatt_write_params write_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_uuid_128 discover_uuid;

/* Whether a subscription is believed live. Which characteristic it's on lives
 * in subscribe_params.value_handle, so the two can't disagree. */
static bool subscribed;

/* Handles for the (service, characteristic) pair the last DataExchange
 * resolved. Cached so a Study running several operations against the same
 * characteristic pays for discovery once; dropped on disconnect, since handles
 * are only meaningful within one connection. */
static struct {
	bool valid;
	uint8_t service_uuid[16];
	uint8_t characteristic_uuid[16];
	uint16_t value_handle;
	uint16_t service_end_handle;
} handle_cache;

/* Scratch for the discovery callbacks to hand results back to the waiter. */
static uint16_t found_service_start;
static uint16_t found_service_end;
static uint16_t found_value_handle;

/* ---- wildcard GATT discovery state (Action::GattDiscover/GattMonitorAll,
 * design.md §3 decisions 31/32) -------------------------------------------
 *
 * `discovered`/`discovered_len` mirror StepResult.gatt_services exactly
 * (ble_bridge.h's struct ble_gatt_service_info) -- `struct outcome` borrows
 * directly from this array, same lifetime rule as `captured` above. Handle
 * ranges and raw value handles are dev-bench-internal bookkeeping the wire
 * type itself has no room for (and no need of), so they live in parallel
 * arrays indexed the same way rather than growing the wire-shaped struct. */
static struct ble_gatt_service_info discovered[BLE_MAX_DISCOVERED_SERVICES];
static uint8_t discovered_len;
static struct {
	uint16_t start;
	uint16_t end;
} service_ranges[BLE_MAX_DISCOVERED_SERVICES];
static uint16_t char_value_handles[BLE_MAX_DISCOVERED_SERVICES][BLE_MAX_CHARS_PER_SERVICE];
/* Which service index discover_all_chars_cb is currently filling in --
 * bt_gatt_discover's own callback signature carries no caller context
 * pointer, so this is how run_gatt_discovery tells it. */
static uint8_t discovering_service_index;

static struct bt_gatt_discover_params all_services_params;
static struct bt_gatt_discover_params all_chars_params;

/* GattMonitorAll's own subscription set -- deliberately separate from
 * subscribe_params/ensure_subscribed above, which is sized for exactly one
 * concurrent subscription (every other GattOperation only ever needs one at
 * a time). One bt_gatt_subscribe_params per subscribed characteristic, since
 * Zephyr's host keeps a pointer to each for the subscription's whole
 * lifetime. */
static struct bt_gatt_subscribe_params monitor_subscribe_params[BLE_MAX_MONITOR_SUBSCRIPTIONS];
static struct bt_gatt_discover_params monitor_ccc_discover_params[BLE_MAX_MONITOR_SUBSCRIPTIONS];
static uint16_t monitor_char_index[BLE_MAX_MONITOR_SUBSCRIPTIONS];
static uint8_t monitor_subscribe_count;

/* `activity[]`/`activity_len` were here -- the fixed-size inline summary
 * `StepResult.gatt_activity` carried. Retired with that field
 * (embarch-study-designer/design.md §3 decision 54): 32 × 528 bytes of
 * static RAM holding the first 32 records of a capture the transcript tap
 * already streams to Core in full. Nothing read the truncated copy that
 * couldn't read the complete one. */

/* ---- GATT transcript (design.md §3 decision 36) ------------------------- */

static ble_transcript_sink transcript_sink;
static void *transcript_user_data;

/* True while a capture window opened by ACTION_GATT_MONITOR_START is still
 * armed -- i.e. between a GattMonitorStart and its GattMonitorStop, across
 * every step that runs in between. Exposed through
 * ble_bridge_monitor_window_open() so main.c can close a window a study left
 * open. */
static bool monitor_window_open;

/* The (service, characteristic) UUID pair the last resolve_handles() settled
 * on, or NULL/NULL when nothing is cached -- what every DataExchange-side
 * transcript entry is labelled with, since execute_read/execute_write are
 * handed only a raw value handle. */
static const uint8_t *cached_service_uuid(void)
{
	return handle_cache.valid ? handle_cache.service_uuid : NULL;
}

static const uint8_t *cached_characteristic_uuid(void)
{
	return handle_cache.valid ? handle_cache.characteristic_uuid : NULL;
}

/* Resolves a flattened characteristic index back to its (service,
 * characteristic) UUID pair, against `discovered`/`discovered_len`.
 *
 * "Flattened" is ble_bridge.h's own documented convention for
 * `ble_gatt_activity_record.characteristic_index`: service 0's characteristics
 * first, then service 1's, and so on, in discovery order. This is the one
 * place that flattening is inverted, so a transcript entry and an activity
 * record can never disagree about which characteristic an index names.
 *
 * Returns false with both outputs left NULL when the index is out of range --
 * a transcript entry with no UUIDs is still a truthful record of the event,
 * which is why callers `(void)` this rather than failing the step. */
static bool uuids_for_flat_index(uint16_t flat, const uint8_t **service_uuid,
				 const uint8_t **characteristic_uuid)
{
	uint16_t seen = 0;

	*service_uuid = NULL;
	*characteristic_uuid = NULL;
	for (uint8_t si = 0; si < discovered_len; si++) {
		uint8_t count = discovered[si].characteristics_len;

		if (flat < seen + count) {
			*service_uuid = discovered[si].uuid;
			*characteristic_uuid = discovered[si].characteristics[flat - seen].uuid;
			return true;
		}
		seen += count;
	}
	return false;
}

/* Hands one GATT event to the registered transcript sink.
 *
 * Called from both Zephyr's BT RX thread (anything inbound) and the dispatch
 * thread (anything this bridge initiates), so `entry` is a stack local rather
 * than a shared static -- the two would otherwise interleave and corrupt each
 * other's entry. ~290 bytes of stack per call, which is why the payload is
 * capped at BLE_MAX_TRANSCRIPT_PAYLOAD_LEN rather than BLE_MAX_PAYLOAD_LEN.
 *
 * A NULL uuid means "this event has no such UUID" (a connect, a discovery
 * starting) and is recorded as absent rather than as sixteen zero bytes,
 * which would be indistinguishable from a real all-zero UUID. Payload bytes
 * beyond the cap are truncated, not dropped: a truncated record still says
 * what happened and when. With no sink registered this is a no-op -- the
 * transcript is observability, never a precondition for a step running. */
static void transcript_emit(uint8_t direction, uint8_t kind, const uint8_t *service_uuid,
			    const uint8_t *characteristic_uuid, uint8_t att_status,
			    const void *payload, size_t payload_len)
{
	if (transcript_sink == NULL) {
		return;
	}

	struct ble_transcript_entry entry = {
		/* Same uptime-based convention as ble_gatt_activity_record's own
		 * rx_utc_ms: no Hello.host_utc_ms clock-offset tracking exists
		 * on this board yet, and Core restamps on receipt regardless. */
		.rx_utc_ms = (uint64_t)k_uptime_get(),
		.direction = direction,
		.kind = kind,
		.att_status = att_status,
	};

	if (service_uuid != NULL) {
		entry.has_service_uuid = true;
		memcpy(entry.service_uuid, service_uuid, 16);
	}
	if (characteristic_uuid != NULL) {
		entry.has_characteristic_uuid = true;
		memcpy(entry.characteristic_uuid, characteristic_uuid, 16);
	}
	if (payload != NULL && payload_len > 0) {
		size_t len = MIN(payload_len, (size_t)BLE_MAX_TRANSCRIPT_PAYLOAD_LEN);

		memcpy(entry.payload, payload, len);
		entry.payload_len = (uint16_t)len;
	}
	transcript_sink(&entry, transcript_user_data);
}

/* ---- small helpers ------------------------------------------------------ */

static struct outcome outcome_pass(void)
{
	struct outcome outcome = {.kind = OUTCOME_PASS};

	if (captured_len > 0) {
		outcome.captured_data = captured;
		outcome.captured_len = captured_len;
	}
	return outcome;
}

static struct outcome outcome_timed_out(void)
{
	return (struct outcome){.kind = OUTCOME_TIMED_OUT};
}

static struct outcome outcome_fail(const char *fmt, ...)
{
	struct outcome outcome = {.kind = OUTCOME_FAIL};
	va_list args;

	va_start(args, fmt);
	vsnprintk(outcome.fail_reason, sizeof(outcome.fail_reason), fmt, args);
	va_end(args);
	return outcome;
}

static int64_t deadline_from(uint32_t timeout_ms)
{
	return k_uptime_get() + (int64_t)timeout_ms;
}

static k_timeout_t remaining(int64_t deadline)
{
	int64_t left = deadline - k_uptime_get();

	return (left > 0) ? K_MSEC(left) : K_NO_WAIT;
}

/* embarch-study-designer stores UUIDs big-endian (src/ids.rs); Zephyr's
 * bt_uuid_create() wants little-endian. */
static void to_bt_uuid(const uint8_t be_bytes[16], struct bt_uuid_128 *out)
{
	uint8_t le_bytes[16];

	for (size_t i = 0; i < sizeof(le_bytes); i++) {
		le_bytes[i] = be_bytes[15 - i];
	}
	(void)bt_uuid_create(&out->uuid, le_bytes, sizeof(le_bytes));
}

/* The reverse of to_bt_uuid, for GattDiscover/GattMonitorAll's live results
 * (design.md §3 decisions 31/32): a discovered attribute's `bt_uuid` may be a
 * 16-, 32-, or 128-bit type (Zephyr's own GAP/GATT services are 16-bit; a
 * DUT's own custom services are typically 128-bit, per this crate's
 * "UUIDs are raw, not symbolic" stance, design.md §4.3) -- expanded here into
 * the Bluetooth Base UUID form (`0000xxxx-0000-1000-8000-00805F9B34FB`) for
 * 16-/32-bit types, matching Zephyr's own BT_UUID_16_TO_UUID_128 convention,
 * so `GattServiceInfo.uuid`/`GattCharacteristicInfo.uuid` are always a full
 * 16-byte value regardless of what the DUT actually declared on the wire. */
static void from_bt_uuid(const struct bt_uuid *uuid, uint8_t out_be[16])
{
	static const uint8_t bt_base_uuid[16] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
		0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB,
	};

	if (uuid->type == BT_UUID_TYPE_128) {
		const struct bt_uuid_128 *u128 = BT_UUID_128(uuid);

		for (size_t i = 0; i < 16; i++) {
			out_be[i] = u128->val[15 - i];
		}
		return;
	}

	memcpy(out_be, bt_base_uuid, sizeof(bt_base_uuid));
	if (uuid->type == BT_UUID_TYPE_16) {
		uint16_t val = BT_UUID_16(uuid)->val;

		/* A 16-bit UUID expands to 0000xxxx-0000-1000-8000-00805F9B34FB
		 * (Bluetooth Core Spec Vol 3, Part B) -- so its two bytes land at
		 * offsets 2 and 3 of the big-endian form, NOT 0 and 1. This wrote
		 * them at 0 and 1 until Milestone 6, which reported every 16-bit
		 * service and characteristic shifted two bytes left: the Device
		 * Information Service came back as `180a0000-0000-1000-8000-
		 * 00805f9b34fb` instead of `0000180a-...`.
		 *
		 * Not cosmetic. `Uuid::parse` in embarch-study-designer expands
		 * "180a" correctly, so a `DataExchange` authored against any
		 * 16-bit UUID could never match the same characteristic discovery
		 * had just reported -- the two representations disagreed, and the
		 * study simply failed to find a service that was plainly there in
		 * its own discovery output. 128-bit UUIDs were never affected
		 * (they take the byte-reversing branch above), which is why every
		 * custom-service study to date worked and this went unnoticed.
		 * The 32-bit branch below was always right: a 32-bit UUID really
		 * does occupy offsets 0..3.
		 */
		out_be[2] = (uint8_t)(val >> 8);
		out_be[3] = (uint8_t)val;
	} else if (uuid->type == BT_UUID_TYPE_32) {
		uint32_t val = BT_UUID_32(uuid)->val;

		out_be[0] = (uint8_t)(val >> 24);
		out_be[1] = (uint8_t)(val >> 16);
		out_be[2] = (uint8_t)(val >> 8);
		out_be[3] = (uint8_t)val;
	}
}

/* Same reversal for a device address (ble_bridge.h's note on byte order). */
static void to_bt_addr(const struct ble_connect_params *params, bt_addr_le_t *out)
{
	out->type = (params->target_address_kind == BLE_ADDR_RANDOM) ? BT_ADDR_LE_RANDOM
								    : BT_ADDR_LE_PUBLIC;
	for (size_t i = 0; i < sizeof(out->a.val); i++) {
		out->a.val[i] = params->target_address[5 - i];
	}
}

/* Releases the reference bt_conn_le_create() returned. `cancel` additionally
 * tears down a connection that may still be establishing (or just established),
 * which is what a connect step that timed out wants. */
static void release_pending_conn(bool cancel)
{
	if (pending_conn == NULL) {
		return;
	}
	if (cancel) {
		(void)bt_conn_disconnect(pending_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	bt_conn_unref(pending_conn);
	pending_conn = NULL;
}

/* Drops a GATT request the caller has given up waiting for, freeing its params
 * struct for reuse. No-op once the link is gone — the host already dropped
 * every pending request with it. */
static void abandon(void *params)
{
	if (active_conn != NULL) {
		bt_gatt_cancel(active_conn, params);
	}
}

static void capture_reset(void)
{
	captured_len = 0;
}

static void capture_append(const void *data, uint16_t len)
{
	size_t room = sizeof(captured) - captured_len;
	size_t copy = (len < room) ? len : room;

	/* A value longer than BLE_MAX_PAYLOAD_LEN is truncated rather than
	 * dropped: the crate's own `captured_data` is bounded by the same
	 * MAX_PAYLOAD_LEN, so there is nowhere to report the extra bytes anyway. */
	memcpy(&captured[captured_len], data, copy);
	captured_len += copy;
}

/* ---- connection callbacks ---------------------------------------------- */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	conn_err = err;
	advertising = false; /* a connectable advertiser stops on connection */

	if (err == 0 && active_conn == NULL) {
		active_conn = bt_conn_ref(conn);
	}
	k_sem_give(&conn_sem);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);

	if (active_conn == conn) {
		bt_conn_unref(active_conn);
		active_conn = NULL;
	}

	/* Handles and subscriptions belong to the connection that's now gone. */
	handle_cache.valid = false;
	subscribed = false;

	link_lost = true;
	k_sem_give(&gatt_sem);
	k_sem_give(&notify_sem);
	k_sem_give(&disconn_sem);
}

/* Reports the level the link actually settled at, whether or not that is the
 * level anything asked for (design.md §3 decision 34). `err` non-zero means
 * the elevation failed outright; `level` is then whatever the link kept. */
static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err)
{
	ARG_UNUSED(conn);

	sec_level = level;
	sec_err = err;
	k_sem_give(&sec_sem);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
};

/* ---- pairing callbacks (design.md §3 decision 34) ----------------------- */

/* **Every callback below runs on Zephyr's BT RX thread, so not one of them
 * logs.** ble_bridge.h's `ble_log_sink` contract is explicit that a log line
 * is only ever emitted from the caller's own thread, because main.c's sink
 * takes the link TX mutex and writes the UART -- blocking the BT RX thread on
 * that is the same class of defect CONFIG_BT_RX_STACK_SIZE and the transcript
 * queue already exist to avoid. So each of these records what it saw in a
 * plain static and returns; `execute_set_security` (dispatch thread) is what
 * turns the record into one diagnostic line.
 *
 * The set of non-NULL pointers here is not cosmetic -- Zephyr *derives* this
 * device's IO capability from exactly which of them are set
 * (subsys/bluetooth/host/smp.c's get_io_capa), and the derived capability is
 * half of what selects the pairing method. `passkey_display` +
 * `passkey_confirm` with LE SC available is DisplayYesNo, which against a
 * DisplayYesNo/KeyboardDisplay peer selects Numeric Comparison -- the only
 * auto-confirmable method the spec counts as authenticated, and therefore the
 * only route to Level 4.
 *
 * Deliberately absent: `passkey_entry`. Adding it would make this
 * KeyboardDisplay instead, a different row of gen_method_sc, and against a
 * DisplayOnly peer that row selects Passkey Entry -- a method needing the six
 * digits the *peer* is displaying, which an unattended bench cannot know.
 * Failing to reach the requested level is a far better outcome than stalling
 * on a prompt nobody can answer.
 *
 * `cancel` is required by Zephyr's own contract whenever any prompt callback
 * is provided, and is the only way this bridge would learn to stop waiting. */

/* Which pairing method actually ran, as observed from which callback fired --
 * the one runtime fact that says whether the derived IO capability landed
 * where this file intends. Reset by every ACTION_BLE_SECURITY. */
enum pairing_observed {
	PAIRING_OBSERVED_NONE = 0,
	/* passkey_confirm fired: LE SC Numeric Comparison, authenticated. */
	PAIRING_OBSERVED_NUMERIC_COMPARISON,
	/* pairing_confirm fired: Just Works, unauthenticated -- the peer's own
	 * IO capability forced it, which caps the link at Level 3. */
	PAIRING_OBSERVED_JUST_WORKS,
	/* passkey_display fired without a confirm: the peer wants Passkey
	 * Entry against a number it expects a human to read off this device. */
	PAIRING_OBSERVED_PASSKEY_DISPLAY,
};

static enum pairing_observed pairing_observed;
static unsigned int pairing_passkey;
static bool pairing_cancelled;
static bool pairing_complete_seen;
static bool pairing_bonded;

static void auth_passkey_display_cb(struct bt_conn *conn, unsigned int passkey)
{
	ARG_UNUSED(conn);

	/* Nothing to display to. This exists so the capability derivation lands
	 * on DisplayYesNo; the value is kept rather than dropped, since it is
	 * the one number a human debugging a pairing can compare against what
	 * the other end showed. */
	pairing_passkey = passkey;
	if (pairing_observed == PAIRING_OBSERVED_NONE) {
		pairing_observed = PAIRING_OBSERVED_PASSKEY_DISPLAY;
	}
}

/* **Auto-confirm, always.** No human input, ever -- an unattended bench has
 * nobody to ask, and the alternative is every security step timing out.
 *
 * This is what makes the resulting L4 "authenticated in the stack's
 * bookkeeping, with no real man-in-the-middle protection": the MITM flag is
 * set and the key is marked authenticated because a Numeric Comparison
 * completed, but nothing compared the numbers. Written here as well as in the
 * design decision so nobody reading only this file concludes otherwise. */
static void auth_passkey_confirm_cb(struct bt_conn *conn, unsigned int passkey)
{
	pairing_passkey = passkey;
	pairing_observed = PAIRING_OBSERVED_NUMERIC_COMPARISON;
	(void)bt_conn_auth_passkey_confirm(conn);
}

/* Registered for the same auto-answer reason: with this non-NULL, Zephyr
 * routes a Just Works pairing through it and *waits*. Leaving it NULL keeps
 * Just Works working on its own, but then a peer whose IO capability forces
 * Just Works and a peer needing an explicit accept would behave differently
 * for no reason a study author could see. It does not affect the capability
 * derivation -- get_io_capa never looks at it. */
static void auth_pairing_confirm_cb(struct bt_conn *conn)
{
	pairing_observed = PAIRING_OBSERVED_JUST_WORKS;
	(void)bt_conn_auth_pairing_confirm(conn);
}

static void auth_cancel_cb(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	pairing_cancelled = true;
}

static const struct bt_conn_auth_cb auth_callbacks = {
	.passkey_display = auth_passkey_display_cb,
	.passkey_confirm = auth_passkey_confirm_cb,
	.pairing_confirm = auth_pairing_confirm_cb,
	.cancel = auth_cancel_cb,
};

static void auth_pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);

	pairing_complete_seen = true;
	pairing_bonded = bonded;
}

/* Recorded rather than merely noted: a pairing that *failed* and a pairing
 * that succeeded at a lower level than requested both end with the step
 * failing, and they are different findings. */
static void auth_pairing_failed_cb(struct bt_conn *conn, enum bt_security_err reason)
{
	ARG_UNUSED(conn);

	pairing_failed_seen = true;
	pairing_failed_reason = reason;
	/* Wake a waiting security step immediately: bt_conn_set_security's
	 * completion is `security_changed`, which a failed pairing may never
	 * reach. */
	k_sem_give(&sec_sem);
}

static struct bt_conn_auth_info_cb auth_info_callbacks = {
	.pairing_complete = auth_pairing_complete_cb,
	.pairing_failed = auth_pairing_failed_cb,
};

/* ---- advertising ------------------------------------------------------- */

/* Starts connectable advertising. `params` may be NULL, meaning "whatever a
 * plain connectable advertiser needs" — used when a BleConnect step in the
 * peripheral role has to advertise but the Study never ran a BleAdvertise step
 * to say how.
 *
 * The Action's own local_name and service UUIDs are put on the air here (the
 * previous bring-up pass advertised CONFIG_BT_DEVICE_NAME regardless, and no
 * UUIDs at all). Service UUIDs go in the scan-response payload rather than the
 * advertising payload, because a 26-byte name (limits::MAX_LOCAL_NAME_LEN, sized
 * to exactly fill a legacy 31-byte advertising PDU alongside the flags) leaves
 * no room for them there; connectable-undirected advertising is scannable, so
 * the separate 31-byte scan response is available for free.
 *
 * Returns -EMSGSIZE if the Action asks for more 128-bit service UUIDs than a
 * legacy scan response can carry. Advertising a subset would be worse than
 * failing: a DUT scanning for one of the dropped UUIDs would never find the
 * bench, and the study would fail later for a reason nothing points at.
 * Carrying more than one needs extended advertising, which nothing in
 * embarch-study-designer's model asks for yet.
 */
#define ADV_LEGACY_PDU_LEN 31
/* One AD structure: length byte + type byte + n * 16 UUID bytes. */
#define ADV_MAX_UUID128 ((ADV_LEGACY_PDU_LEN - 2) / 16)

static int start_advertising(const struct ble_advertise_params *params)
{
	struct bt_data ad[2];
	size_t ad_len = 0;
	struct bt_data sd[1];
	size_t sd_len = 0;
	static uint8_t uuid_ad[ADV_MAX_UUID128 * 16];
	static const uint8_t flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
	uint16_t interval = BT_GAP_ADV_FAST_INT_MIN_2;
	const char *name = (params != NULL && params->has_local_name) ? params->local_name
								     : bt_get_name();
	size_t name_len = strlen(name);

	ad[ad_len++] = (struct bt_data)BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags));

	if (name_len > BLE_MAX_LOCAL_NAME_LEN) {
		/* Only reachable via the bt_get_name() fallback — a Study's own
		 * local_name is already bounded by limits::MAX_LOCAL_NAME_LEN. Flag it
		 * as shortened rather than passing a truncated name off as complete. */
		ad[ad_len++] = (struct bt_data)BT_DATA(BT_DATA_NAME_SHORTENED, name,
						       BLE_MAX_LOCAL_NAME_LEN);
	} else {
		ad[ad_len++] =
			(struct bt_data)BT_DATA(BT_DATA_NAME_COMPLETE, name, (uint8_t)name_len);
	}

	if (params != NULL && params->service_uuid_count > 0) {
		const uint8_t count = params->service_uuid_count;

		if (count > ADV_MAX_UUID128) {
			return -EMSGSIZE;
		}
		for (uint8_t i = 0; i < count; i++) {
			/* On-air 128-bit UUIDs are little-endian, same reversal
			 * to_bt_uuid does for the host API's benefit. */
			for (size_t b = 0; b < 16; b++) {
				uuid_ad[(i * 16) + b] = params->service_uuids[i][15 - b];
			}
		}
		sd[sd_len++] = (struct bt_data)BT_DATA(BT_DATA_UUID128_ALL, uuid_ad, count * 16);
	}

	if (params != NULL && params->adv_interval_ms > 0) {
		/* BT_GAP_MS_TO_ADV_INTERVAL's own documented range; a Study asking
		 * for something outside it gets clamped rather than rejected, since
		 * the interval only affects discovery latency, not correctness. */
		uint16_t ms = params->adv_interval_ms;

		if (ms < 20) {
			ms = 20;
		} else if (ms > 10240) {
			ms = 10240;
		}
		interval = BT_GAP_MS_TO_ADV_INTERVAL(ms);
	}

	struct bt_le_adv_param adv_param =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN, interval, interval, NULL);

	/* Stop first so a second BleAdvertise step with different parameters
	 * actually takes effect instead of returning -EALREADY. */
	(void)bt_le_adv_stop();

	int err = bt_le_adv_start(&adv_param, ad, ad_len, (sd_len > 0) ? sd : NULL, sd_len);

	if (err == 0) {
		advertising = true;
	}
	return err;
}

static struct outcome execute_advertise(const struct ble_advertise_params *params)
{
	int err = start_advertising(params);

	if (err == -EMSGSIZE) {
		return outcome_fail("%u service UUIDs exceed legacy advertising (max %u)",
				    params->service_uuid_count, (unsigned int)ADV_MAX_UUID128);
	}
	if (err != 0) {
		return outcome_fail("bt_le_adv_start failed (%d)", err);
	}
	return outcome_pass();
}

/* ---- connect ----------------------------------------------------------- */

static bool peer_matches(const struct ble_connect_params *params)
{
	bt_addr_le_t want;

	if (!params->has_target_address) {
		return true;
	}
	if (active_conn == NULL) {
		return false;
	}
	to_bt_addr(params, &want);
	return bt_addr_le_cmp(bt_conn_get_dst(active_conn), &want) == 0;
}


/* Diagnostic log sink (ble_bridge.h's `ble_log_sink`) and the one helper that
 * writes to it. Only ever called from the dispatch thread -- see that
 * typedef's own doc comment for why that restriction is the point. */
static ble_log_sink log_sink;
static void *log_sink_user;

void ble_bridge_set_log_sink(ble_log_sink sink, void *user_data)
{
	log_sink = sink;
	log_sink_user = user_data;
}

static void bridge_log(const char *fmt, ...)
{
	if (log_sink == NULL) {
		return;
	}

	/* Sized to the link's own log-line limit so the sink never has to
	 * truncate what it's handed. `static`, not a stack local: the dispatch
	 * thread is the only caller (the typedef's contract), and these call
	 * sites sit under an already-deep BLE call stack. */
	static char line[BLE_MAX_LOG_LINE_LEN + 1];
	va_list args;

	va_start(args, fmt);
	vsnprintk(line, sizeof(line), fmt, args);
	va_end(args);
	log_sink(line, log_sink_user);
}

/* Advertised-name filter (embarch-study-designer/design.md §3 decision 43).
 * `scan_name[0] == '\0'` means no name filter. */
static char scan_name[BLE_MAX_LOCAL_NAME_LEN + 1];

/* What this scan has seen, keyed by advertiser address.
 *
 * Address-keyed rather than a single "last name that matched" slot, which is
 * what this started as and what was wrong with it. A peripheral's name often
 * arrives in its scan response, which is not itself connectable, so the name
 * and the connectable advertisement are two separate callbacks in an order
 * this code does not control. Remembering per address means either order
 * works: whichever packet completes the pair triggers the connect, because
 * the entry records both facts independently.
 *
 * It also serves as the diagnostic for a scan that matched nothing. A bare
 * `TimedOut` cannot distinguish "the DUT is silent" from "the DUT is on the
 * air but advertises no name" -- and the second is invisible to a name
 * filter by construction, so a name-only report can never rule it out. Live
 * finding, Milestone 6: a five-minute scan for a DUT's configured
 * `CONFIG_BT_DEVICE_NAME` reported only two unrelated names, which left
 * exactly those two possibilities open and no way to choose between them. */
/* Raised from 8 -> 24 -> 256. The first real census hit 8 and said so ("more
 * than this firmware records"); the next found 12 distinct advertisers in
 * three minutes, so 24 was only one busy room away from truncating again. A
 * cap that silently truncates defeats the whole purpose of the diagnostic:
 * the advertiser that matters can be any of them, and "not in the list" has
 * to mean "not on the air", not "list was full".
 *
 * 256 is a deliberate over-provision at ~9 KB of static RAM on a board
 * already at ~96% SRAM (decision 27's own finding) -- affordable only because
 * this is a flat table of 35-byte entries rather than anything frame-sized
 * (contrast decision 29's ring buffer, which explicitly could not be sized to
 * a worst-case frame). The lookup is a linear scan per advertisement, which
 * is fine: it runs a few hundred byte-comparisons per packet, against a step
 * timeout measured in seconds. */
#define SCAN_SEEN_MAX 256
struct scan_seen_entry {
	bt_addr_le_t addr;
	/* Empty until a Local Name AD element is seen for this address. */
	char name[BLE_MAX_LOCAL_NAME_LEN + 1];
	/* Set once this address has sent a connectable advertisement --
	 * ADV_IND/ADV_DIRECT_IND. A scan response alone doesn't set it. */
	bool connectable;
};
static struct scan_seen_entry scan_seen[SCAN_SEEN_MAX];
static uint8_t scan_seen_len;
static bool scan_seen_overflowed;

/* Find-or-insert by address. NULL when the table is full (recorded as an
 * overflow so a report can say so rather than quietly under-listing). */
static struct scan_seen_entry *scan_seen_entry_for(const bt_addr_le_t *addr)
{
	for (uint8_t i = 0; i < scan_seen_len; i++) {
		if (bt_addr_le_cmp(&scan_seen[i].addr, addr) == 0) {
			return &scan_seen[i];
		}
	}
	if (scan_seen_len >= SCAN_SEEN_MAX) {
		scan_seen_overflowed = true;
		return NULL;
	}

	struct scan_seen_entry *entry = &scan_seen[scan_seen_len++];

	bt_addr_le_copy(&entry->addr, addr);
	entry->name[0] = '\0';
	entry->connectable = false;
	return entry;
}

/* bt_data_parse callback: copies any Local Name AD element into the
 * `struct scan_seen_entry *` passed as user_data. */
static bool name_ad_record(struct bt_data *data, void *user_data)
{
	struct scan_seen_entry *entry = user_data;

	if (data->type != BT_DATA_NAME_COMPLETE && data->type != BT_DATA_NAME_SHORTENED) {
		return true; /* keep parsing the remaining AD elements */
	}

	size_t len = MIN((size_t)data->data_len, (size_t)BLE_MAX_LOCAL_NAME_LEN);

	/* A complete name replaces a shortened one; a shortened one does not
	 * overwrite a complete one already recorded. Longest-wins is a good
	 * enough proxy and avoids tracking which kind produced the stored
	 * value. */
	if (len >= strlen(entry->name)) {
		memcpy(entry->name, data->data, len);
		entry->name[len] = '\0';
	}
	return true;
}

/* One line per advertiser seen, emitted through the bridge's log sink from
 * the dispatch thread once a name-filtered scan has given up. */
static void report_scan_seen(void)
{
	bridge_log("scan saw %u advertiser(s)%s:", (unsigned int)scan_seen_len,
		   scan_seen_overflowed ? " (more than this firmware records)" : "");
	for (uint8_t i = 0; i < scan_seen_len; i++) {
		char addr_str[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(&scan_seen[i].addr, addr_str, sizeof(addr_str));
		bridge_log("  %s %s name=%s", addr_str,
			   scan_seen[i].connectable ? "connectable" : "non-connectable",
			   scan_seen[i].name[0] != '\0' ? scan_seen[i].name : "(none advertised)");
	}
}

/* Comma-separated list of the names seen this scan, into a static buffer --
 * the compact form that fits an `Outcome`'s 64-byte `fail_reason`. The full
 * per-advertiser detail goes through `report_scan_seen` instead.
 *
 * The requested name is deliberately not repeated by the caller: the caller
 * already knows what it asked for, whereas the names actually on the air are
 * the part it cannot get any other way. Echoing both truncated the list
 * exactly where it mattered -- found the first time this ran on a real bench,
 * where the one interesting name got cut in half. */
static const char *scan_seen_names_summary(void)
{
	static char summary[OUTCOME_MAX_FAIL_REASON_LEN + 1];
	size_t used = 0;
	uint8_t named = 0;

	summary[0] = '\0';
	for (uint8_t i = 0; i < scan_seen_len; i++) {
		if (scan_seen[i].name[0] == '\0') {
			continue;
		}

		int written = snprintk(summary + used, sizeof(summary) - used, "%s'%s'",
				       (named == 0) ? "" : ", ", scan_seen[i].name);

		if (written < 0 || (size_t)written >= sizeof(summary) - used) {
			break; /* truncated -- the names that fit are still the useful part */
		}
		used += (size_t)written;
		named++;
	}
	if (named == 0) {
		return (scan_seen_len == 0) ? "(nothing advertising at all)"
					    : "(advertisers seen, none named)";
	}
	return summary;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	ARG_UNUSED(rssi);

	if (scan_matched) {
		return;
	}

	struct scan_seen_entry *entry = scan_seen_entry_for(addr);

	if (entry == NULL) {
		return; /* table full; nothing this call can usefully record or match */
	}

	/* Recorded for *every* advertisement type, scan responses included --
	 * that's usually where the name is. Done before any connectable-type
	 * filtering, which would otherwise discard the packet carrying it.
	 * `bt_data_parse` consumes the buffer it's given and this callback does
	 * not own `buf`, so it gets a copy. */
	if (buf != NULL) {
		struct net_buf_simple copy = *buf;

		bt_data_parse(&copy, name_ad_record, entry);
	}
	if (adv_type == BT_GAP_ADV_TYPE_ADV_IND || adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		entry->connectable = true;
	}

	/* Only a connectable advertiser can be connected to -- but the decision
	 * is made from the *entry*, not from this packet's own type, so a name
	 * arriving in a scan response can complete a match whose connectable
	 * advertisement already went by. */
	if (!entry->connectable) {
		return;
	}
	if (scan_target_set && bt_addr_le_cmp(addr, &scan_target) != 0) {
		return;
	}
	/* Both filters are ANDed: with a name set, only an address that
	 * advertised exactly that name is connected to. Exact match -- a loose
	 * one would reintroduce the failure decision 43 exists to remove, just
	 * less visibly. */
	if (scan_name[0] != '\0' && strcmp(entry->name, scan_name) != 0) {
		return;
	}

	scan_matched = true;

	/* Scanning must stop before initiating — stopping from inside this
	 * callback is Zephyr's own documented central pattern. */
	int err = bt_le_scan_stop();

	if (err != 0 && err != -EALREADY) {
		conn_err = BT_HCI_ERR_UNSPECIFIED;
		k_sem_give(&conn_sem);
		return;
	}

	struct bt_conn *conn = NULL;

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &conn);
	if (err != 0) {
		conn_err = BT_HCI_ERR_UNSPECIFIED;
		k_sem_give(&conn_sem);
		return;
	}
	/* Handed to connect_as_central, which releases it once the step resolves —
	 * connected_cb takes its own reference for active_conn regardless. */
	pending_conn = conn;
}

static struct outcome connect_as_central(const struct ble_connect_params *params,
					 int64_t deadline)
{
	scan_target_set = params->has_target_address;
	if (scan_target_set) {
		to_bt_addr(params, &scan_target);
	}
	if (params->has_target_name) {
		strncpy(scan_name, params->target_name, sizeof(scan_name) - 1);
		scan_name[sizeof(scan_name) - 1] = '\0';
	} else {
		scan_name[0] = '\0';
	}
	scan_seen_len = 0;
	scan_seen_overflowed = false;
	scan_matched = false;
	conn_err = 0;
	k_sem_reset(&conn_sem);

	/* Deliberately NOT BT_LE_SCAN_ACTIVE: that macro sets
	 * BT_LE_SCAN_OPT_FILTER_DUPLICATE, which reports each advertiser at most
	 * once per scan. That is fine for "connect to the first thing you see",
	 * but it breaks the name filter (design.md §3 decision 32): a name that
	 * arrives in a scan response has to be matched against a *connectable*
	 * advertisement from the same address, and with duplicate filtering
	 * there is no second advertisement to match it against -- the name is
	 * recorded and then nothing connectable ever arrives again.
	 *
	 * Same type (active, so scan responses are solicited at all) and same
	 * interval/window as BT_LE_SCAN_ACTIVE; only the duplicate-filter option
	 * is dropped. The cost is more callbacks per second, which is bounded by
	 * the step's own timeout and cheap -- scan_cb returns immediately once
	 * scan_matched, and scan_seen_names de-duplicates for itself. */
	static const struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	int err = bt_le_scan_start(&scan_param, scan_cb);

	if (err != 0 && err != -EALREADY) {
		return outcome_fail("bt_le_scan_start failed (%d)", err);
	}

	if (k_sem_take(&conn_sem, remaining(deadline)) != 0) {
		(void)bt_le_scan_stop();
		/* Cancel a connection still being established, so a DUT that answers
		 * just after the step gave up doesn't leave a link no step asked for
		 * (and no later step expects). Disconnecting a connecting object is
		 * Zephyr's documented way to cancel bt_conn_le_create. */
		release_pending_conn(true);
		if (scan_name[0] != '\0') {
			/* A name filter that matched nothing is reported as a
			 * Fail naming what *was* advertised, not as a bare
			 * TimedOut: "nothing called X appeared, but these did"
			 * is actionable. The per-advertiser detail (addresses,
			 * connectability, whether a name was advertised at all)
			 * goes through the log sink, because `fail_reason` is
			 * 64 bytes and cannot hold it. */
			report_scan_seen();
			return outcome_fail("no name match; on air: %s%s",
					    scan_seen_names_summary(),
					    scan_seen_overflowed ? ", ..." : "");
		}
		return outcome_timed_out();
	}

	release_pending_conn(false);

	if (conn_err != 0) {
		return outcome_fail("connection failed (HCI 0x%02x)", conn_err);
	}
	if (active_conn == NULL) {
		return outcome_fail("connected callback left no connection");
	}
	return outcome_pass();
}

static struct outcome connect_as_peripheral(const struct ble_connect_params *params,
					    int64_t deadline)
{
	while (true) {
		conn_err = 0;
		k_sem_reset(&conn_sem);

		if (!advertising) {
			int err = start_advertising(NULL);

			if (err != 0) {
				return outcome_fail("bt_le_adv_start failed (%d)", err);
			}
		}

		if (k_sem_take(&conn_sem, remaining(deadline)) != 0) {
			return outcome_timed_out();
		}
		if (conn_err != 0) {
			return outcome_fail("connection failed (HCI 0x%02x)", conn_err);
		}
		if (peer_matches(params)) {
			return outcome_pass();
		}

		/* Some other central connected while we were waiting for a specific
		 * DUT. Drop it and keep advertising for whatever budget is left,
		 * rather than reporting a Pass against the wrong peer. */
		k_sem_reset(&disconn_sem);
		(void)bt_conn_disconnect(active_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (k_sem_take(&disconn_sem, remaining(deadline)) != 0) {
			return outcome_timed_out();
		}
	}
}

static struct outcome execute_connect(const struct ble_connect_params *params, int64_t deadline)
{
	if (active_conn != NULL) {
		/* Already connected — a Study reconnecting the same link mid-run is
		 * a no-op, not an error (decision 15: one DUT at a time). */
		if (peer_matches(params)) {
			return outcome_pass();
		}
		return outcome_fail("already connected to a different peer");
	}

	if (params->role == BLE_ROLE_CENTRAL) {
		return connect_as_central(params, deadline);
	}
	return connect_as_peripheral(params, deadline);
}

/* ---- discovery --------------------------------------------------------- */

static uint8_t discover_service_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (attr != NULL) {
		const struct bt_gatt_service_val *service = attr->user_data;

		found_service_start = attr->handle + 1;
		found_service_end = service->end_handle;
	}
	k_sem_give(&gatt_sem);
	return BT_GATT_ITER_STOP;
}

static uint8_t discover_chrc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (attr != NULL) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		found_value_handle = chrc->value_handle;
	}
	k_sem_give(&gatt_sem);
	return BT_GATT_ITER_STOP;
}

/* Runs one bt_gatt_discover procedure and waits for it. Returns 0 on
 * completion (found or not — the caller checks its own `found_*` scratch), or a
 * negative errno / -ETIMEDOUT / -ENOTCONN. */
static int run_discovery(uint8_t type, const struct bt_uuid *uuid, uint16_t start_handle,
			 uint16_t end_handle, bt_gatt_discover_func_t func, int64_t deadline)
{
	discover_params.uuid = uuid;
	discover_params.func = func;
	discover_params.start_handle = start_handle;
	discover_params.end_handle = end_handle;
	discover_params.type = type;

	link_lost = false;
	k_sem_reset(&gatt_sem);

	int err = bt_gatt_discover(active_conn, &discover_params);

	if (err != 0) {
		return err;
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		/* Abandon the procedure rather than leaving it pending: the host would
		 * otherwise still own discover_params, making the next step's
		 * discovery fail with -EBUSY. Cancelling drives the callback with a
		 * synthetic error, which the next k_sem_reset() discards. */
		abandon(&discover_params);
		return -ETIMEDOUT;
	}
	if (link_lost) {
		return -ENOTCONN;
	}
	return 0;
}

/* Resolves (service_uuid, characteristic_uuid) to a value handle, reusing the
 * cache when the same pair was already resolved on this connection. */
static struct outcome resolve_handles(const struct data_exchange_params *params,
				      int64_t deadline)
{
	if (handle_cache.valid &&
	    memcmp(handle_cache.service_uuid, params->service_uuid, 16) == 0 &&
	    memcmp(handle_cache.characteristic_uuid, params->characteristic_uuid, 16) == 0) {
		return outcome_pass();
	}

	to_bt_uuid(params->service_uuid, &discover_uuid);
	found_service_start = 0;
	found_service_end = 0;

	int err = run_discovery(BT_GATT_DISCOVER_PRIMARY, &discover_uuid.uuid,
				BT_ATT_FIRST_ATTRIBUTE_HANDLE, BT_ATT_LAST_ATTRIBUTE_HANDLE,
				discover_service_cb, deadline);

	if (err == -ETIMEDOUT) {
		return outcome_timed_out();
	}
	if (err == -ENOTCONN) {
		return outcome_fail("disconnected during service discovery");
	}
	if (err != 0) {
		return outcome_fail("service discovery failed (%d)", err);
	}
	if (found_service_end == 0) {
		return outcome_fail("service not found on DUT");
	}

	uint16_t service_start = found_service_start;
	uint16_t service_end = found_service_end;

	to_bt_uuid(params->characteristic_uuid, &discover_uuid);
	found_value_handle = 0;

	err = run_discovery(BT_GATT_DISCOVER_CHARACTERISTIC, &discover_uuid.uuid, service_start,
			    service_end, discover_chrc_cb, deadline);

	if (err == -ETIMEDOUT) {
		return outcome_timed_out();
	}
	if (err == -ENOTCONN) {
		return outcome_fail("disconnected during characteristic discovery");
	}
	if (err != 0) {
		return outcome_fail("characteristic discovery failed (%d)", err);
	}
	if (found_value_handle == 0) {
		return outcome_fail("characteristic not found in service");
	}

	memcpy(handle_cache.service_uuid, params->service_uuid, 16);
	memcpy(handle_cache.characteristic_uuid, params->characteristic_uuid, 16);
	handle_cache.value_handle = found_value_handle;
	handle_cache.service_end_handle = service_end;
	handle_cache.valid = true;

	return outcome_pass();
}

/* Forward declaration: subscribe_cb is defined below (in the "GATT
 * operations" section), but execute_gatt_monitor_all -- which sits before
 * that section, alongside the rest of this file's discovery logic -- reuses
 * it for its own per-characteristic CCC-write callback (it only logs the ATT
 * result, no state specific to the single-subscription case below). */
static void subscribe_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_subscribe_params *params);

/* ---- wildcard discovery (Action::GattDiscover/GattMonitorAll) ---------- */

/* Unlike discover_service_cb/discover_chrc_cb above (a single targeted
 * result, BT_GATT_ITER_STOP as soon as one arrives), these iterate: Zephyr
 * keeps calling back with BT_GATT_ITER_CONTINUE-honoring callbacks until
 * either every matching attribute has been reported or the callback itself
 * stops early, finally calling back once more with `attr == NULL` to signal
 * "this discovery procedure is done." */
static uint8_t discover_all_services_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					 struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (attr == NULL) {
		k_sem_give(&gatt_sem);
		return BT_GATT_ITER_STOP;
	}
	if (discovered_len >= BLE_MAX_DISCOVERED_SERVICES) {
		/* Capacity reached -- stop discovering further services; what's
		 * already found stands (mirrors design.md §3 decision 32's own
		 * "log and skip rather than corrupt" precedent for gatt_activity). */
		k_sem_give(&gatt_sem);
		return BT_GATT_ITER_STOP;
	}

	const struct bt_gatt_service_val *service = attr->user_data;
	struct ble_gatt_service_info *info = &discovered[discovered_len];

	memset(info, 0, sizeof(*info));
	/* attr->uuid is the *declaration* attribute's own type UUID (always
	 * 0x2800, "Primary Service") for a BT_GATT_DISCOVER_PRIMARY callback
	 * -- identical for every service found. The service's actual UUID is
	 * service->uuid (struct bt_gatt_service_val, attr->user_data), which
	 * this function already fetches into `service` above but never used. */
	from_bt_uuid(service->uuid, info->uuid);
	service_ranges[discovered_len].start = attr->handle + 1;
	service_ranges[discovered_len].end = service->end_handle;
	discovered_len++;

	transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_SERVICE_DISCOVERED, info->uuid, NULL, 0,
			NULL, 0);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_all_chars_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	struct ble_gatt_service_info *info = &discovered[discovering_service_index];

	if (attr == NULL) {
		k_sem_give(&gatt_sem);
		return BT_GATT_ITER_STOP;
	}
	if (info->characteristics_len >= BLE_MAX_CHARS_PER_SERVICE) {
		k_sem_give(&gatt_sem);
		return BT_GATT_ITER_STOP;
	}

	const struct bt_gatt_chrc *chrc = attr->user_data;
	uint8_t char_idx = info->characteristics_len;
	struct ble_gatt_characteristic_info *cinfo = &info->characteristics[char_idx];

	from_bt_uuid(chrc->uuid, cinfo->uuid);
	cinfo->properties = chrc->properties;
	char_value_handles[discovering_service_index][char_idx] = chrc->value_handle;
	info->characteristics_len = char_idx + 1;

	/* `att_status` carries the raw ATT properties byte here rather than an
	 * error code -- the one field on the entry with room for it, and the
	 * fact a reader most wants next to a discovered characteristic
	 * (is it writable? does it notify?). Documented rather than adding a
	 * field used by exactly one event kind. */
	transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_CHARACTERISTIC_DISCOVERED, info->uuid,
			cinfo->uuid, chrc->properties, NULL, 0);

	return BT_GATT_ITER_CONTINUE;
}

/* Walks every primary service, then every characteristic within each,
 * populating `discovered`/`discovered_len` (design.md §4.3a's
 * "GattDiscover"/"GattMonitorAll share one discovery walk" framing). Two
 * discovery passes per service is unavoidable: Zephyr can't be told
 * "discover primary services AND their characteristics" in one procedure,
 * and a nested bt_gatt_discover() call from inside a discovery callback
 * isn't safe -- so this runs the wildcard service pass to completion first,
 * then a wildcard characteristic pass per discovered service afterward,
 * bounded throughout by the same `deadline` decision 16's own doc comment
 * already establishes for every action in this file. */
static struct outcome run_gatt_discovery(int64_t deadline)
{
	transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_DISCOVERY_STARTED, NULL, NULL, 0, NULL,
			0);

	discovered_len = 0;
	memset(discovered, 0, sizeof(discovered));

	all_services_params.uuid = NULL; /* wildcard: every primary service */
	all_services_params.func = discover_all_services_cb;
	all_services_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	all_services_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	all_services_params.type = BT_GATT_DISCOVER_PRIMARY;

	link_lost = false;
	k_sem_reset(&gatt_sem);

	int err = bt_gatt_discover(active_conn, &all_services_params);

	if (err != 0) {
		return outcome_fail("primary service discovery failed to start (%d)", err);
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		abandon(&all_services_params);
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected during service discovery");
	}

	for (uint8_t i = 0; i < discovered_len; i++) {
		discovering_service_index = i;

		all_chars_params.uuid = NULL; /* wildcard: every characteristic */
		all_chars_params.func = discover_all_chars_cb;
		all_chars_params.start_handle = service_ranges[i].start;
		all_chars_params.end_handle = service_ranges[i].end;
		all_chars_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		k_sem_reset(&gatt_sem);
		err = bt_gatt_discover(active_conn, &all_chars_params);
		if (err != 0) {
			return outcome_fail("characteristic discovery failed to start (%d)", err);
		}
		if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
			abandon(&all_chars_params);
			return outcome_timed_out();
		}
		if (link_lost) {
			return outcome_fail("disconnected during characteristic discovery");
		}
	}

	return outcome_pass();
}

static struct outcome execute_gatt_discover(int64_t deadline)
{
	if (active_conn == NULL) {
		return outcome_fail("no active connection -- run a BleConnect step first");
	}

	struct outcome result = run_gatt_discovery(deadline);

	if (result.kind != OUTCOME_PASS) {
		return result;
	}
	result.gatt_services = discovered;
	result.gatt_service_count = discovered_len;
	return result;
}

/* service/characteristic index, flattened service-then-characteristic in
 * discovery order -- the exact convention design.md §4.3a documents for
 * `GattActivityRecord.characteristic_index`, computed here in the one place
 * both this bridge and any consumer need to agree on it. */
static uint16_t flat_characteristic_index(uint8_t service_idx, uint8_t char_idx)
{
	uint16_t flat = 0;

	for (uint8_t s = 0; s < service_idx; s++) {
		flat += discovered[s].characteristics_len;
	}
	return flat + char_idx;
}

/* GattMonitorAll's own notify callback -- deliberately not notify_cb (below):
 * that one serves the single shared subscribe_params ensure_subscribed()
 * manages for GATT_OP_NOTIFY/INDICATE/SUBSCRIBE/STREAM_CAPTURE, whereas this
 * step subscribes to many characteristics concurrently, each with its own
 * `struct bt_gatt_subscribe_params` in monitor_subscribe_params -- `params`
 * is one of that array's elements, so pointer arithmetic recovers which one
 * fired without a second lookup table keyed by handle. */
static uint8_t monitor_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
				 const void *data, uint16_t length)
{
	ARG_UNUSED(conn);

	if (data == NULL) {
		/* This one subscription was torn down (disconnect, or the server
		 * clearing it) -- nothing to record; the rest keep running. */
		return BT_GATT_ITER_STOP;
	}
	ptrdiff_t idx_for_transcript = params - monitor_subscribe_params;
	uint16_t flat_for_transcript =
		(idx_for_transcript >= 0 && (size_t)idx_for_transcript < monitor_subscribe_count)
			? monitor_char_index[idx_for_transcript]
			: 0;
	const uint8_t *tr_service_uuid = NULL;
	const uint8_t *tr_char_uuid = NULL;

	(void)uuids_for_flat_index(flat_for_transcript, &tr_service_uuid, &tr_char_uuid);
	/* **The only sink for a captured notification, as of schema v14.** It
	 * used to be one of two, the other being a 32-record inline summary
	 * that stopped growing mid-capture (design.md §3 decision 54 retired
	 * it). The transcript is bounded only by the study's own duration, and
	 * main.c fans one entry out to the declared transcript tap and to any
	 * GattNotify tap that named this characteristic
	 * (embarch-study-designer/design.md §3 decision 55) -- so
	 * "exhaustive" is now true of every route a record can take, rather
	 * than of one of two. */
	transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_NOTIFICATION, tr_service_uuid,
			tr_char_uuid, 0, data, length);

	return BT_GATT_ITER_CONTINUE;
}

/* Which target in `params` a discovered characteristic matches, or -1.
 * A NULL/empty target list means "everything", which is what
 * ACTION_GATT_MONITOR_ALL/START pass. */
static int target_index_for(const struct gatt_monitor_selected_params *params,
			     const uint8_t *service_uuid, const uint8_t *char_uuid)
{
	if (params == NULL || params->targets_len == 0) {
		return 0; /* unfiltered: everything matches */
	}
	for (size_t t = 0; t < params->targets_len; t++) {
		if (memcmp(params->targets[t].service_uuid, service_uuid, 16) == 0 &&
		    memcmp(params->targets[t].characteristic_uuid, char_uuid, 16) == 0) {
			return (int)t;
		}
	}
	return -1;
}

/* Discovery + subscribe, shared by all four monitor actions -- the
 * unfiltered pair (design.md §3 decision 36) pass `params == NULL`, the
 * selective pair (that doc's decision 53) pass their target list. Leaves
 * every subscription armed; the caller decides whether to tear them down at
 * the end of its own step (MonitorAll/Selected) or leave them live across
 * the steps that follow (MonitorStart/SelectedStart).
 *
 * **The two differ in what a characteristic that isn't subscribed means.**
 * Unfiltered, skipping one is routine -- nothing named it. Filtered, every
 * target is something the study said it expects, so a target that matched no
 * discovered characteristic, or matched one that can neither notify nor
 * indicate, fails the step naming it. Left as a skip it would be a study
 * that passes having captured nothing, which is the failure decisions 34, 36,
 * 53 and 54 were each opened by. */
static struct outcome monitor_subscribe_matching(int64_t deadline,
						  const struct gatt_monitor_selected_params *params)
{
	if (active_conn == NULL) {
		return outcome_fail("no active connection -- run a BleConnect step first");
	}

	struct outcome discover_result = run_gatt_discovery(deadline);

	if (discover_result.kind != OUTCOME_PASS) {
		return discover_result;
	}

	monitor_subscribe_count = 0;

	/* Per-target: did it match a discovered characteristic at all, and was
	 * that characteristic subscribable. Both reported, because they are
	 * different mistakes -- a typo'd UUID and a read-only characteristic
	 * need different fixes. */
	bool target_found[BLE_MAX_MONITOR_TARGETS] = {false};
	bool target_subscribable[BLE_MAX_MONITOR_TARGETS] = {false};

	for (uint8_t s = 0; s < discovered_len; s++) {
		struct ble_gatt_service_info *service = &discovered[s];

		for (uint8_t c = 0; c < service->characteristics_len; c++) {
			uint8_t props = service->characteristics[c].properties;
			bool notify = (props & BT_GATT_CHRC_NOTIFY) != 0;
			bool indicate = (props & BT_GATT_CHRC_INDICATE) != 0;
			int target = target_index_for(params, service->uuid,
						       service->characteristics[c].uuid);

			if (target < 0) {
				continue; /* not named by this step */
			}
			if (params != NULL && params->targets_len > 0) {
				target_found[target] = true;
				target_subscribable[target] = notify || indicate;
			}
			if (!notify && !indicate) {
				continue;
			}
			if (monitor_subscribe_count >= BLE_MAX_MONITOR_SUBSCRIPTIONS) {
				/* BLE_MAX_MONITOR_SUBSCRIPTIONS's own doc comment:
				 * log-and-skip further characteristics, not a failure.
				 * Recorded in the transcript so a reader can see that
				 * a characteristic was deliberately not subscribed
				 * rather than silently producing nothing. */
				transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR,
						service->uuid,
						service->characteristics[c].uuid, 0,
						"subscription limit reached", 26);
				continue;
			}

			struct bt_gatt_subscribe_params *sp =
				&monitor_subscribe_params[monitor_subscribe_count];

			memset(sp, 0, sizeof(*sp));
			sp->notify = monitor_notify_cb;
			sp->subscribe = subscribe_cb; /* shared: only logs the ATT result */
			sp->value_handle = char_value_handles[s][c];
			sp->value = indicate ? BT_GATT_CCC_INDICATE : BT_GATT_CCC_NOTIFY;
			sp->ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
			sp->end_handle = service_ranges[s].end;
			sp->disc_params = &monitor_ccc_discover_params[monitor_subscribe_count];

			ccc_att_err = 0;
			ccc_torn_down = false;
			k_sem_reset(&ccc_sem);

			int err = bt_gatt_subscribe(active_conn, sp);

			if (err != 0 && err != -EALREADY) {
				/* This one characteristic's CCC write couldn't even
				 * start -- move on to the next rather than failing
				 * the whole step over one characteristic. */
				transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR,
						service->uuid,
						service->characteristics[c].uuid, 0,
						"bt_gatt_subscribe failed", 24);
				continue;
			}
			if (err == 0 && k_sem_take(&ccc_sem, remaining(deadline)) != 0) {
				abandon(sp);
				transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR,
						service->uuid,
						service->characteristics[c].uuid, 0,
						"CCC write timed out", 19);
				continue;
			}

			transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_SUBSCRIBED, service->uuid,
					service->characteristics[c].uuid, 0, NULL, 0);

			monitor_char_index[monitor_subscribe_count] =
				flat_characteristic_index(s, c);
			monitor_subscribe_count++;
		}
	}

	if (params != NULL && params->targets_len > 0) {
		for (size_t t = 0; t < params->targets_len; t++) {
			if (!target_found[t]) {
				return outcome_fail(
					"target %u (%02x%02x..%02x%02x) is not on this DUT",
					(unsigned int)t + 1,
					params->targets[t].characteristic_uuid[0],
					params->targets[t].characteristic_uuid[1],
					params->targets[t].characteristic_uuid[14],
					params->targets[t].characteristic_uuid[15]);
			}
			if (!target_subscribable[t]) {
				return outcome_fail(
					"target %u (%02x%02x..%02x%02x) can neither notify nor indicate",
					(unsigned int)t + 1,
					params->targets[t].characteristic_uuid[0],
					params->targets[t].characteristic_uuid[1],
					params->targets[t].characteristic_uuid[14],
					params->targets[t].characteristic_uuid[15]);
			}
		}
	}

	return outcome_pass();
}

/* Tears down every subscription monitor_subscribe_all armed. Fire-and-forget:
 * by the time this runs the owning step's deadline has usually passed, so
 * there is no budget left to wait out each CCC-clear write's response. */
static void monitor_unsubscribe_all(void)
{
	for (uint8_t i = 0; i < monitor_subscribe_count; i++) {
		const uint8_t *service_uuid = NULL;
		const uint8_t *char_uuid = NULL;

		(void)uuids_for_flat_index(monitor_char_index[i], &service_uuid, &char_uuid);
		(void)bt_gatt_unsubscribe(active_conn, &monitor_subscribe_params[i]);
		transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_UNSUBSCRIBED, service_uuid,
				char_uuid, 0, NULL, 0);
	}
	monitor_subscribe_count = 0;
}

/* Fills in the discovery field every monitor action reports.
 *
 * It used to fill two -- `gatt_activity` was the other, retired at schema
 * v14 (embarch-study-designer/design.md §3 decision 54). What a monitor step
 * captured is read out of the study's `streams/` files now, which is where
 * all of it is rather than the first 32 records of it. */
static struct outcome monitor_result(void)
{
	struct outcome result = outcome_pass();

	result.gatt_services = discovered;
	result.gatt_service_count = discovered_len;
	return result;
}

/* Shared by ACTION_GATT_MONITOR_ALL and ACTION_GATT_MONITOR_SELECTED --
 * `params` is NULL for the first and the step's target list for the second,
 * and that is the *only* difference between them (design.md §3 decision
 * 53). */
static struct outcome execute_gatt_monitor_window(
	int64_t deadline, const struct gatt_monitor_selected_params *params)
{
	struct outcome subscribed_result = monitor_subscribe_matching(deadline, params);

	if (subscribed_result.kind != OUTCOME_PASS) {
		return subscribed_result;
	}

	/* Capture window: whatever's left of the step's own timeout_ms after
	 * discovery+subscribe -- no separate duration field, same "the step's
	 * own budget is the window" precedent as GATT_OP_STREAM_CAPTURE
	 * (design.md §3 decisions 20/21). Ends on the deadline (the normal
	 * case) or early if the DUT drops the link. */
	link_lost = false;
	k_sem_reset(&disconn_sem);
	bool dropped = k_sem_take(&disconn_sem, remaining(deadline)) == 0;

	/* Unsubscribe everything this step subscribed, regardless of outcome --
	 * a later step shouldn't keep receiving this step's notifications.
	 * This is exactly the behaviour ACTION_GATT_MONITOR_START exists to
	 * opt out of (design.md §3 decision 36). */
	monitor_unsubscribe_all();

	if (dropped) {
		return outcome_fail("disconnected during GATT monitor capture");
	}

	return monitor_result();
}

/* design.md §3 decision 36 (and 53's selective half). Subscribes and returns
 * immediately, leaving the window open: the step costs only
 * discovery+subscribe time, not its whole timeout_ms, because the capture
 * happens during the steps that follow rather than during this one. */
static struct outcome execute_gatt_monitor_window_start(
	int64_t deadline, const struct gatt_monitor_selected_params *params)
{
	if (monitor_window_open) {
		/* Re-arming over a live window would silently orphan the first
		 * set of subscriptions in Zephyr's host. Close it first. */
		monitor_unsubscribe_all();
	}

	struct outcome subscribed_result = monitor_subscribe_matching(deadline, params);

	if (subscribed_result.kind != OUTCOME_PASS) {
		monitor_window_open = false;
		return subscribed_result;
	}

	monitor_window_open = true;

	/* Reports what it subscribed to, so the step's own result is useful on
	 * its own. What the window actually captures lands in the study's
	 * `streams/` files as it arrives, not in this step's result. */
	struct outcome result = outcome_pass();

	result.gatt_services = discovered;
	result.gatt_service_count = discovered_len;
	return result;
}

/* design.md §3 decision 36. A Stop with no open window is a no-op Pass, not a
 * Fail: a study that ends without one still has its window closed for it
 * (main.c), so an explicit-but-redundant Stop is a harmless authoring
 * pattern, not an error worth aborting a study over. */
static struct outcome execute_gatt_monitor_stop(void)
{
	if (!monitor_window_open) {
		return outcome_pass();
	}

	monitor_unsubscribe_all();
	monitor_window_open = false;
	return monitor_result();
}

/* ---- GATT operations --------------------------------------------------- */

static uint8_t read_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
		       const void *data, uint16_t length)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (err != 0) {
		att_err = err;
		k_sem_give(&gatt_sem);
		return BT_GATT_ITER_STOP;
	}
	if (data == NULL) {
		/* Zephyr signals "read complete" with a NULL data callback. */
		k_sem_give(&gatt_sem);
		return BT_GATT_ITER_STOP;
	}
	capture_append(data, length);
	return BT_GATT_ITER_CONTINUE;
}

static void write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	att_err = err;
	k_sem_give(&gatt_sem);
}

static void subscribe_cb(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_subscribe_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	ccc_att_err = err;
	k_sem_give(&ccc_sem);
}

static uint8_t notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			 const void *data, uint16_t length)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (data == NULL) {
		/* The subscription is gone: a CCC descriptor that couldn't be
		 * discovered or written, the server clearing it, or the host dropping
		 * it on disconnection. Also how a failed bt_gatt_subscribe reports
		 * itself, hence the ccc_sem wake-up. */
		subscribed = false;
		ccc_torn_down = true;
		k_sem_give(&ccc_sem);
		return BT_GATT_ITER_STOP;
	}

	if (streaming) {
		if (stream_handler != NULL) {
			stream_handler(data, length, stream_user_data);
		}
		/* Recorded even though the bytes also go to the stream handler:
		 * "exhaustive" means the transcript accounts for every
		 * notification, including the ones another channel consumes. */
		transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_NOTIFICATION, cached_service_uuid(),
				cached_characteristic_uuid(), 0, data, length);
		return BT_GATT_ITER_CONTINUE;
	}

	transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_NOTIFICATION, cached_service_uuid(),
			cached_characteristic_uuid(), 0, data, length);

	capture_reset();
	capture_append(data, length);
	k_sem_give(&notify_sem);
	return BT_GATT_ITER_CONTINUE;
}

static struct outcome execute_read(uint16_t value_handle, int64_t deadline)
{
	read_params.func = read_cb;
	read_params.handle_count = 1;
	read_params.single.handle = value_handle;
	read_params.single.offset = 0;

	att_err = 0;
	link_lost = false;
	capture_reset();
	k_sem_reset(&gatt_sem);

	transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_READ_REQUEST, cached_service_uuid(),
			cached_characteristic_uuid(), 0, NULL, 0);

	int err = bt_gatt_read(active_conn, &read_params);

	if (err != 0) {
		transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR, cached_service_uuid(),
				cached_characteristic_uuid(), 0, "bt_gatt_read failed", 19);
		return outcome_fail("bt_gatt_read failed (%d)", err);
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		abandon(&read_params);
		transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR, cached_service_uuid(),
				cached_characteristic_uuid(), 0, "read timed out", 14);
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected during read");
	}
	if (att_err != 0) {
		transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_ERROR, cached_service_uuid(),
				cached_characteristic_uuid(), att_err, NULL, 0);
		return outcome_fail("read rejected (ATT 0x%02x)", att_err);
	}
	/* The value itself, not just "a read happened" -- `captured` holds
	 * whatever read_cb appended. */
	transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_READ_RESPONSE, cached_service_uuid(),
			cached_characteristic_uuid(), 0, captured, captured_len);
	return outcome_pass();
}

static struct outcome execute_write(uint16_t value_handle, const uint8_t *payload,
				    size_t payload_len, int64_t deadline)
{
	write_params.func = write_cb;
	write_params.handle = value_handle;
	write_params.offset = 0;
	write_params.data = payload;
	write_params.length = (uint16_t)payload_len;

	att_err = 0;
	link_lost = false;
	capture_reset();
	k_sem_reset(&gatt_sem);

	/* The stimulus itself, recorded before it goes out -- design.md §3
	 * decision 36's "record what dev-bench sent, not only what it
	 * received". Without this a transcript shows a DUT's response with
	 * nothing explaining what provoked it. */
	transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_WRITE_REQUEST, cached_service_uuid(),
			cached_characteristic_uuid(), 0, payload, payload_len);

	int err = bt_gatt_write(active_conn, &write_params);

	if (err != 0) {
		transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR, cached_service_uuid(),
				cached_characteristic_uuid(), 0, "bt_gatt_write failed", 20);
		return outcome_fail("bt_gatt_write failed (%d)", err);
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		abandon(&write_params);
		transcript_emit(BLE_GATT_DIR_LOCAL, BLE_GATT_EVT_ERROR, cached_service_uuid(),
				cached_characteristic_uuid(), 0, "write timed out", 15);
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected during write");
	}
	if (att_err != 0) {
		transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_ERROR, cached_service_uuid(),
				cached_characteristic_uuid(), att_err, NULL, 0);
		return outcome_fail("write rejected (ATT 0x%02x)", att_err);
	}
	transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_WRITE_RESPONSE, cached_service_uuid(),
			cached_characteristic_uuid(), 0, NULL, 0);
	return outcome_pass();
}

/* Subscribes to `value_handle` for notifications or indications, reusing an
 * existing subscription on the same handle. Only one subscription exists at a
 * time — there's one static subscribe_params, matching one-DUT-at-a-time.
 *
 * Returns only once the CCC write has actually been answered, so a Notify step
 * can't start waiting before the DUT has been told to push anything.
 *
 * subscribe_params is never memset: after bt_gatt_unsubscribe() the host still
 * holds this pointer for the in-flight CCC write and dereferences
 * `params->notify` when the response lands (Zephyr's `gatt_write_ccc_rsp`), so
 * zeroing it would be a NULL call. Fields are overwritten in place instead, and
 * `flags`/`node` are left entirely to the host.
 */
static struct outcome ensure_subscribed(uint16_t value_handle, uint16_t ccc_value,
					int64_t deadline)
{
	if (subscribed && subscribe_params.value_handle == value_handle &&
	    subscribe_params.value == ccc_value) {
		return outcome_pass();
	}

	if (subscribe_params.value_handle != 0) {
		/* Unconditional, not `if (subscribed)`: the host keeps a bonded
		 * peer's subscription across a reconnect even though
		 * disconnected_cb cleared our own flag. Unsubscribing one that isn't
		 * (or is no longer) in the host's list just returns an error worth
		 * ignoring — but when it *does* succeed, the CCC-clearing write has
		 * to complete before this params struct is reused. */
		ccc_att_err = 0;
		ccc_torn_down = false;
		k_sem_reset(&ccc_sem);
		subscribed = false;

		if (bt_gatt_unsubscribe(active_conn, &subscribe_params) == 0 &&
		    k_sem_take(&ccc_sem, remaining(deadline)) != 0) {
			abandon(&subscribe_params);
			return outcome_timed_out();
		}
	}

	subscribe_params.notify = notify_cb;
	subscribe_params.subscribe = subscribe_cb;
	subscribe_params.value_handle = value_handle;
	subscribe_params.value = ccc_value;
	/* Let the host find the CCC descriptor itself (CONFIG_BT_GATT_AUTO_DISCOVER_CCC)
	 * rather than adding a third discovery pass here; it needs the enclosing
	 * service's end handle to bound that search. */
	subscribe_params.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
	subscribe_params.end_handle = handle_cache.service_end_handle;
	subscribe_params.disc_params = &ccc_discover_params;

	ccc_att_err = 0;
	ccc_torn_down = false;
	k_sem_reset(&ccc_sem);

	int err = bt_gatt_subscribe(active_conn, &subscribe_params);

	if (err != 0 && err != -EALREADY) {
		return outcome_fail("bt_gatt_subscribe failed (%d)", err);
	}

	if (err == 0 && k_sem_take(&ccc_sem, remaining(deadline)) != 0) {
		abandon(&subscribe_params);
		return outcome_timed_out();
	}
	if (ccc_torn_down) {
		/* Zephyr reports a failed subscription (no CCC descriptor found, or a
		 * rejected write) by calling notify with NULL data. */
		return outcome_fail("subscribe failed — no writable CCC on that characteristic");
	}
	if (ccc_att_err != 0) {
		return outcome_fail("CCC write rejected (ATT 0x%02x)", ccc_att_err);
	}

	subscribed = true;
	return outcome_pass();
}

/* Waits for one pushed value from an already-subscribed characteristic.
 * `op_deadline` is the GattOperation's own timeout, which the crate defines
 * independently of the step's timeout_ms — whichever expires first wins.
 *
 * Waits for a value pushed after the step began: ble_bridge_execute() resets
 * both the capture buffer and notify_sem on entry, so a notification that
 * arrived between steps (e.g. after an earlier Subscribe step) is not replayed
 * here. Continuous capture is GATT_OP_STREAM_CAPTURE's job, not this one's.
 */
static struct outcome await_notification(int64_t op_deadline, int64_t step_deadline)
{
	int64_t deadline = (op_deadline < step_deadline) ? op_deadline : step_deadline;

	if (k_sem_take(&notify_sem, remaining(deadline)) != 0) {
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected while awaiting notification");
	}
	return outcome_pass();
}

static struct outcome execute_data_exchange(const struct data_exchange_params *params,
					    int64_t deadline)
{
	if (active_conn == NULL) {
		return outcome_fail("no active connection — run a BleConnect step first");
	}

	struct outcome resolved = resolve_handles(params, deadline);

	if (resolved.kind != OUTCOME_PASS) {
		return resolved;
	}

	const uint16_t value_handle = handle_cache.value_handle;

	switch (params->operation.kind) {
	case GATT_OP_READ:
		return execute_read(value_handle, deadline);

	case GATT_OP_WRITE:
		return execute_write(value_handle, params->operation.write.payload,
				     params->operation.write.payload_len, deadline);

	case GATT_OP_NOTIFY:
	case GATT_OP_INDICATE: {
		const bool indicate = params->operation.kind == GATT_OP_INDICATE;
		const uint32_t op_timeout_ms = indicate ? params->operation.indicate.timeout_ms
							: params->operation.notify.timeout_ms;
		struct outcome sub =
			ensure_subscribed(value_handle,
					  indicate ? BT_GATT_CCC_INDICATE : BT_GATT_CCC_NOTIFY,
					  deadline);

		if (sub.kind != OUTCOME_PASS) {
			return sub;
		}
		return await_notification(deadline_from(op_timeout_ms), deadline);
	}

	case GATT_OP_SUBSCRIBE:
		/* Enable notifications without waiting for one; the subscription
		 * deliberately outlives this step so a later Notify/Indicate or
		 * StreamCapture step on the same characteristic reuses it. */
		return ensure_subscribed(value_handle, BT_GATT_CCC_NOTIFY, deadline);

	case GATT_OP_STREAM_CAPTURE: {
		/* Continuous capture for the whole step (no separate duration field
		 * in the crate's type — the step's timeout_ms is the window). Values
		 * go to stream_handler as they arrive rather than into `captured`. */
		struct outcome sub = ensure_subscribed(value_handle, BT_GATT_CCC_NOTIFY, deadline);

		if (sub.kind != OUTCOME_PASS) {
			return sub;
		}

		capture_reset();
		link_lost = false;
		k_sem_reset(&disconn_sem);
		streaming = true;
		/* The window ends either when the step's budget runs out (the normal
		 * case, a timeout on this wait) or early if the DUT drops the link. */
		const bool dropped = k_sem_take(&disconn_sem, remaining(deadline)) == 0;

		streaming = false;

		if (dropped) {
			return outcome_fail("disconnected during stream capture");
		}
		/* Running the window to completion is the outcome; whether any
		 * samples arrived is a host-side question
		 * (embarch-study-designer/design.md §3 decision 19). */
		return outcome_pass();
	}

	default:
		return outcome_fail("unknown GATT operation kind");
	}
}

/* ---- security (design.md §3 decisions 34/37) ---------------------------- */

/* Translates the wire's SecurityLevel discriminant into Zephyr's own
 * bt_security_t. Two enums with the same *order* and different *bases*, so
 * this is a switch rather than an offset -- an offset would keep compiling
 * the day either side grows a variant somewhere other than the end. */
static bool to_bt_security(uint8_t level, bt_security_t *out)
{
	switch (level) {
	case BLE_SECURITY_L1:
		*out = BT_SECURITY_L1;
		return true;
	case BLE_SECURITY_L2:
		*out = BT_SECURITY_L2;
		return true;
	case BLE_SECURITY_L3:
		*out = BT_SECURITY_L3;
		return true;
	case BLE_SECURITY_L4:
		*out = BT_SECURITY_L4;
		return true;
	default:
		return false;
	}
}

/* The inverse, for reporting. Anything Zephyr reports that this enum has no
 * name for (BT_SECURITY_L0 on an invalid connection, or a future level) is
 * *not* reported at all rather than reported as the nearest thing -- a
 * `StepResult` claiming a level the link never had is worse than one saying
 * nothing. */
static bool from_bt_security(bt_security_t level, uint8_t *out)
{
	switch (level) {
	case BT_SECURITY_L1:
		*out = BLE_SECURITY_L1;
		return true;
	case BT_SECURITY_L2:
		*out = BLE_SECURITY_L2;
		return true;
	case BT_SECURITY_L3:
		*out = BLE_SECURITY_L3;
		return true;
	case BT_SECURITY_L4:
		*out = BLE_SECURITY_L4;
		return true;
	default:
		return false;
	}
}

/* Note there is deliberately no wire-enum-to-level-number helper in this
 * file: every message below prints a `bt_security_t`, whose own enumerators
 * already *are* the spec's level numbers (BT_SECURITY_L4 == 4). Only the wire
 * enum carries the off-by-one, and the two functions above are the only place
 * it is crossed. */

static const char *pairing_method_name(void)
{
	switch (pairing_observed) {
	case PAIRING_OBSERVED_NUMERIC_COMPARISON:
		return "LE SC Numeric Comparison (authenticated)";
	case PAIRING_OBSERVED_JUST_WORKS:
		return "Just Works (UNauthenticated -- the peer's IO capability forced it)";
	case PAIRING_OBSERVED_PASSKEY_DISPLAY:
		return "Passkey Display (the peer wants a human to read the number)";
	default:
		return "none (no pairing ran -- keys were already in place)";
	}
}

/* Elevates the link to `params->level`, answering the pairing prompts itself,
 * and **fails the step if the level actually reached is lower than asked
 * for**.
 *
 * That strictness is the point. `bt_conn_set_security` returning 0 only means
 * the request was accepted; the level the link settles at depends on the
 * peer's IO capability and on whether pairing completed at all. A step named
 * "establish L4" that passed at L2 would be the silent degradation this whole
 * decision exists to refuse. A study that wants the attempt without the
 * abort sets `Step.continue_on_fail` -- an existing knob, deliberately reused
 * instead of adding a second one here.
 *
 * `timeout_ms` bounds the elevation, and `delay_before_ms` says when it
 * starts, so a DUT enforcing its own post-connect security deadline is
 * authorable with no new timing field.
 *
 * **BLE_SECURITY_L1 needs no special case**, and deliberately gets none.
 * embarch-study-designer/design.md §3 decision 44 makes L1 the honest way
 * for a study to say "this DUT needs no security" rather than omitting the
 * step; a connected LE link is already at L1, so the already-at-or-above
 * check below passes it on the first iteration without a branch of its own.
 *
 * **-EBUSY is not a failure, and finding that out cost a hardware run.** The
 * first real study against the DUT failed this step with
 * `bt_conn_set_security(L4) failed (-16)` -- and the *next* step then ran at
 * L4. Zephyr returns -EBUSY when a security procedure is already in flight
 * (`smp.c`'s `SMP_FLAG_PAIRING`/`SMP_FLAG_ENC_PENDING`), which is exactly
 * what a DUT that issues its own Security Request immediately after connect
 * produces. Failing on it reported "could not establish security" about a
 * link that was seconds away from being secured, which is worse than useless
 * -- it points at the wrong side. So -EBUSY means *wait for the one already
 * running and judge what it reaches*, and only then, if it settled below what
 * was asked, ask again on this step's own behalf. Two attempts, not a retry
 * loop: `bt_conn_set_security` resets `conn->required_sec_level` on error, so
 * the in-flight procedure targets whatever the peer asked for rather than
 * this study's level, and one further request after it completes is all that
 * is needed to close that gap. */
static struct outcome execute_set_security(const struct ble_set_security_params *params,
					    int64_t deadline)
{
	bt_security_t want;

	if (active_conn == NULL) {
		return outcome_fail("no connection to secure; connect first");
	}
	if (!to_bt_security(params->level, &want)) {
		return outcome_fail("unknown security level %u", (unsigned int)params->level);
	}

	pairing_observed = PAIRING_OBSERVED_NONE;
	pairing_passkey = 0;
	pairing_cancelled = false;
	pairing_complete_seen = false;
	pairing_bonded = false;
	pairing_failed_seen = false;
	sec_err = BT_SECURITY_ERR_SUCCESS;

	for (int attempt = 0; attempt < 2; attempt++) {
		/* Armed **before** the level is read, not after. Reading first
		 * and resetting second loses a security_changed that lands
		 * between the two -- and this step's whole job is to be running
		 * at exactly the moment one does. */
		k_sem_reset(&sec_sem);

		bt_security_t already = bt_conn_get_security(active_conn);

		if (already >= want) {
			/* Idempotent, and this covers three real cases rather
			 * than one: a repeated step, a *bonded reconnect* where
			 * encryption came back from the bond with no pairing to
			 * run (and therefore no security_changed to wait for),
			 * and a peer whose own Security Request got there first.
			 * Waiting anyway would burn the whole timeout and then
			 * fail a link that is already correct. */
			bridge_log("security at L%u (asked for L%u)", (unsigned int)already,
				   (unsigned int)want);
			return outcome_pass();
		}

		int err = bt_conn_set_security(active_conn, want);
		bool in_flight_was_someone_elses = false;

		if (err == -EBUSY) {
			/* A procedure is already running -- see this function's
			 * own comment. Wait for it rather than reporting a
			 * failure about it. */
			in_flight_was_someone_elses = true;
			bridge_log("security already in flight; waiting for it (asked for L%u)",
				   (unsigned int)want);
		} else if (err != 0) {
			return outcome_fail("bt_conn_set_security(L%u) failed (%d)",
					     (unsigned int)want, err);
		}

		/* Two things can end this wait: security_changed (the elevation
		 * settled, at whatever level) or pairing_failed (it never
		 * will). disconnected_cb gives neither, so `link_lost` is
		 * checked on the way out -- a DUT that drops the link rather
		 * than answering is exactly the behaviour this action was added
		 * to make legible. */
		if (k_sem_take(&sec_sem, remaining(deadline)) != 0) {
			if (link_lost) {
				return outcome_fail("disconnected while establishing security");
			}
			return outcome_timed_out();
		}
		if (link_lost) {
			return outcome_fail("disconnected while establishing security");
		}

		bridge_log("pairing method: %s", pairing_method_name());
		if (pairing_observed == PAIRING_OBSERVED_NUMERIC_COMPARISON ||
		    pairing_observed == PAIRING_OBSERVED_PASSKEY_DISPLAY) {
			bridge_log("passkey was %06u (auto-confirmed, never compared)",
				   pairing_passkey);
		}
		if (pairing_complete_seen) {
			bridge_log("pairing complete, bonded=%d", (int)pairing_bonded);
		}

		if (pairing_failed_seen) {
			return outcome_fail("pairing failed (security err %d), link stayed at L%u",
					     (int)pairing_failed_reason,
					     (unsigned int)bt_conn_get_security(active_conn));
		}
		if (pairing_cancelled) {
			return outcome_fail("pairing was cancelled by the stack");
		}

		/* Asked of the connection, not taken from the callback's
		 * argument: the callback reports one transition, and what the
		 * step must report is where the link ended up. */
		bt_security_t reached = bt_conn_get_security(active_conn);

		if (sec_err != BT_SECURITY_ERR_SUCCESS) {
			return outcome_fail("security error %d, link at L%u", (int)sec_err,
					     (unsigned int)reached);
		}
		if (reached >= want) {
			return outcome_pass();
		}
		if (!in_flight_was_someone_elses) {
			/* Our own request completed and landed short. Asking
			 * again cannot help -- nothing about either peer's
			 * capabilities changed -- so this is the honest stop. */
			return outcome_fail("reached L%u, study asked for L%u",
					     (unsigned int)reached, (unsigned int)want);
		}
		/* The procedure we waited on was not ours and settled below
		 * what this study needs. Now that it is done, ask on our own
		 * behalf -- the next iteration does exactly that. */
	}

	return outcome_fail("stuck at L%u after the in-flight procedure settled; asked for L%u",
			     (unsigned int)bt_conn_get_security(active_conn), (unsigned int)want);
}

/* Drops the bond, inside the study (embarch-study-designer/design.md §3
 * decision 50; this firmware's half is design.md §3 decision 37).
 *
 * **This disconnects.** Zephyr's bt_unpair disconnects a peer whose keys it
 * clears -- dev-bench does not choose that, and it is right: a link whose
 * keys just went away is not a link. So this waits for the disconnect rather
 * than returning with `active_conn` still populated, which is precisely the
 * race ble_bridge_reset() already had to fix once (the next BleConnect
 * failing with "already connected to a different peer", passing on retry).
 *
 * Clears *every* bond rather than only the connected peer's: dev-bench holds
 * one connection at a time (design.md §3 decision 15), so the two are the
 * same set in practice, and "clear the table" is the thing a study author
 * actually means by "drop the bond". */
static struct outcome execute_unbond(void)
{
	bool was_connected = active_conn != NULL;

	if (was_connected) {
		k_sem_reset(&disconn_sem);
	}

	int err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);

	if (err != 0) {
		return outcome_fail("bt_unpair failed (%d)", err);
	}

	if (was_connected) {
		/* A fixed bound, not the step's deadline: the unbond itself has
		 * already happened by here, and a local disconnect that takes
		 * longer than two seconds is a stuck link, not a slow one.
		 * Timing out leaves the stale conn and the next connect fails
		 * as it would have anyway -- no worse than not waiting. */
		if (k_sem_take(&disconn_sem, K_MSEC(2000)) != 0) {
			bridge_log("unbond: peer did not disconnect within 2s");
		}
	}

	/* Handles and any cached discovery belonged to the link that just
	 * went away. */
	handle_cache.valid = false;
	subscribed = false;

	bridge_log("bond table cleared%s", was_connected ? " (link dropped)" : "");
	return outcome_pass();
}

/* ---- public API -------------------------------------------------------- */

int ble_bridge_init(void)
{
	int err = bt_conn_cb_register(&conn_callbacks);

	if (err != 0) {
		return err;
	}

	/* Registered before bt_enable, so no connection can ever exist without
	 * them: Zephyr latches `bt_auth` into an SMP context the first time it
	 * needs a capability, and a peer that connected and initiated pairing
	 * before this ran would have latched NULL -- NoInputNoOutput, Just
	 * Works, unauthenticated, with nothing in the resulting failure
	 * pointing at registration order (design.md §3 decision 34). */
	err = bt_conn_auth_cb_register(&auth_callbacks);
	if (err != 0) {
		return err;
	}
	err = bt_conn_auth_info_cb_register(&auth_info_callbacks);
	if (err != 0) {
		return err;
	}

	return bt_enable(NULL);
}

/* The action switch itself, split out of ble_bridge_execute so the
 * security-level stamp below it runs for every kind without each arm having
 * to remember it. */
/* ---- Action::RunProtocol (embarch-study-designer/design.md §3 decision 60)
 *
 * **The BLE half only.** eap_interp.c owns every decision this loop acts on:
 * what to write, how long to wait, whether an arriving frame advances the
 * machine, and what the run ended as. What lives here is the part that
 * needs a radio -- resolve the manifest's declared characteristics against
 * the DUT's real GATT table, subscribe to the ones frames arrive on, perform
 * the writes the interpreter asks for, and hand arrivals and timer expiries
 * back to it.
 *
 * That split is the whole reason the semantics are testable on a host
 * (app/tests/serial_protocol drives eap_interp.c with no DUT in the room) and
 * it is also why this function has no protocol knowledge of its own to get
 * wrong.
 */

/* One arriving notification, on its way from Zephyr's BT RX thread to the
 * dispatch thread the interpreter runs on.
 *
 * A queue rather than a direct call because the two really are different
 * threads (ble_bridge.h's own sink contract says a notify callback must not
 * block) and the interpreter's next step can be a write, which does. Sized to
 * one ATT MTU per record, the same bound `struct ble_transcript_entry` uses
 * and for the same reason: this bridge never sees an inbound value larger
 * than one. */
struct protocol_notification {
	uint8_t source;
	uint16_t len;
	uint8_t payload[BLE_MAX_TRANSCRIPT_PAYLOAD_LEN];
};

/* Four slots, matching the transcript queue's own posture: deep enough that a
 * burst arriving while the interpreter is inside a blocking write is not
 * lost, shallow enough to cost ~1 KB on a board that has overflowed twice.
 * An overflow is **counted and reported**, never silently dropped -- a
 * protocol run that missed a frame reached the wrong state, and a result that
 * did not say so would be the silently-incomplete capture this suite keeps
 * arriving at from other directions. */
#define PROTOCOL_NOTIFY_QUEUE_SLOTS 4

K_MSGQ_DEFINE(protocol_notify_q, sizeof(struct protocol_notification),
	      PROTOCOL_NOTIFY_QUEUE_SLOTS, 4);

static struct bt_gatt_subscribe_params protocol_subscribe_params[EAP_MAX_SOURCES_PER_PROTOCOL];
static struct bt_gatt_discover_params protocol_ccc_discover_params[EAP_MAX_SOURCES_PER_PROTOCOL];
/* Which manifest source each armed subscription belongs to. The notify
 * callback recovers its slot by pointer arithmetic into
 * protocol_subscribe_params (the same trick monitor_notify_cb uses), and this
 * turns that slot back into the index the interpreter speaks in. */
static uint8_t protocol_subscribe_source[EAP_MAX_SOURCES_PER_PROTOCOL];
static uint8_t protocol_subscribe_count;
/* Per-source GATT state, indexed by the manifest's own source index. */
static uint16_t protocol_source_handle[EAP_MAX_SOURCES_PER_PROTOCOL];
static uint16_t protocol_source_service_end[EAP_MAX_SOURCES_PER_PROTOCOL];
static uint8_t protocol_source_props[EAP_MAX_SOURCES_PER_PROTOCOL];
static uint32_t protocol_notifications_dropped;

static uint8_t protocol_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
				  const void *data, uint16_t length)
{
	ARG_UNUSED(conn);

	if (data == NULL) {
		/* This one subscription was torn down (disconnect, or the
		 * server clearing it). Nothing to enqueue; the run's own step
		 * timeout is what notices that frames stopped arriving. */
		return BT_GATT_ITER_STOP;
	}

	ptrdiff_t slot = params - protocol_subscribe_params;

	if (slot < 0 || (size_t)slot >= protocol_subscribe_count) {
		return BT_GATT_ITER_CONTINUE;
	}

	static struct protocol_notification note;

	note.source = protocol_subscribe_source[slot];
	note.len = (length > sizeof(note.payload)) ? (uint16_t)sizeof(note.payload) : length;
	memcpy(note.payload, data, note.len);

	if (k_msgq_put(&protocol_notify_q, &note, K_NO_WAIT) != 0) {
		protocol_notifications_dropped++;
	}
	return BT_GATT_ITER_CONTINUE;
}

/* Locate every declared source in the DUT's real GATT table.
 *
 * A source the DUT does not have **fails the step naming it**, and is not
 * skipped: a manifest that declares a characteristic has said it expects one,
 * which is the same rule a selective monitor's targets follow (§3 decision
 * 53) and the opposite of the subscribe-to-everything walk's log-and-skip --
 * right there precisely because nothing was named.
 */
static struct outcome protocol_resolve_sources(const struct eap_protocol_def *def,
					       int64_t deadline)
{
	struct outcome discover_result = run_gatt_discovery(deadline);

	if (discover_result.kind != OUTCOME_PASS) {
		return discover_result;
	}

	for (uint8_t i = 0; i < def->sources_len; i++) {
		bool found = false;

		for (uint8_t s = 0; s < discovered_len && !found; s++) {
			if (memcmp(discovered[s].uuid, def->sources[i].service_uuid, 16) != 0) {
				continue;
			}
			for (uint8_t c = 0; c < discovered[s].characteristics_len; c++) {
				if (memcmp(discovered[s].characteristics[c].uuid,
					   def->sources[i].characteristic_uuid, 16) != 0) {
					continue;
				}
				protocol_source_handle[i] = char_value_handles[s][c];
				protocol_source_service_end[i] = service_ranges[s].end;
				protocol_source_props[i] =
					discovered[s].characteristics[c].properties;
				found = true;
				break;
			}
		}
		if (!found) {
			return outcome_fail(
				"protocol source %u (%02x%02x..%02x%02x) is not on this DUT",
				(unsigned int)i, def->sources[i].characteristic_uuid[0],
				def->sources[i].characteristic_uuid[1],
				def->sources[i].characteristic_uuid[14],
				def->sources[i].characteristic_uuid[15]);
		}
	}
	return outcome_pass();
}

/* Whether any frame reads from this source -- i.e. whether the machine can
 * ever be moved by something arriving on it. A source only ever written to
 * (a control point) is deliberately not subscribed: subscribing to it would
 * be this bridge deciding something the manifest did not say. */
static bool protocol_source_is_read(const struct eap_protocol_def *def, uint8_t source)
{
	for (uint8_t f = 0; f < def->frames_len; f++) {
		if (def->frames[f].source == source) {
			return true;
		}
	}
	return false;
}

static struct outcome protocol_subscribe_sources(const struct eap_protocol_def *def,
						 int64_t deadline)
{
	protocol_subscribe_count = 0;

	for (uint8_t i = 0; i < def->sources_len; i++) {
		if (!protocol_source_is_read(def, i)) {
			continue;
		}

		bool notify = (protocol_source_props[i] & BT_GATT_CHRC_NOTIFY) != 0;
		bool indicate = (protocol_source_props[i] & BT_GATT_CHRC_INDICATE) != 0;

		if (!notify && !indicate) {
			return outcome_fail(
				"protocol source %u (%02x%02x..%02x%02x) can neither notify nor indicate",
				(unsigned int)i, def->sources[i].characteristic_uuid[0],
				def->sources[i].characteristic_uuid[1],
				def->sources[i].characteristic_uuid[14],
				def->sources[i].characteristic_uuid[15]);
		}

		struct bt_gatt_subscribe_params *sp =
			&protocol_subscribe_params[protocol_subscribe_count];

		memset(sp, 0, sizeof(*sp));
		sp->notify = protocol_notify_cb;
		sp->subscribe = subscribe_cb; /* shared: only logs the ATT result */
		sp->value_handle = protocol_source_handle[i];
		sp->value = indicate ? BT_GATT_CCC_INDICATE : BT_GATT_CCC_NOTIFY;
		sp->ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
		sp->end_handle = protocol_source_service_end[i];
		sp->disc_params = &protocol_ccc_discover_params[protocol_subscribe_count];

		ccc_att_err = 0;
		ccc_torn_down = false;
		k_sem_reset(&ccc_sem);

		int err = bt_gatt_subscribe(active_conn, sp);

		if (err != 0 && err != -EALREADY) {
			return outcome_fail("subscribe to protocol source %u failed (%d)",
					    (unsigned int)i, err);
		}
		if (err == 0 && k_sem_take(&ccc_sem, remaining(deadline)) != 0) {
			abandon(sp);
			return outcome_timed_out();
		}
		if (ccc_torn_down) {
			return outcome_fail("protocol source %u has no writable CCC",
					    (unsigned int)i);
		}
		if (ccc_att_err != 0) {
			return outcome_fail("protocol source %u CCC write rejected (ATT 0x%02x)",
					    (unsigned int)i, ccc_att_err);
		}

		transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_SUBSCRIBED,
				def->sources[i].service_uuid,
				def->sources[i].characteristic_uuid, 0, NULL, 0);

		protocol_subscribe_source[protocol_subscribe_count] = i;
		protocol_subscribe_count++;
	}
	return outcome_pass();
}

/* Fire-and-forget, the same posture monitor_unsubscribe_all takes and for the
 * same reason: by the time this runs the step's deadline has usually passed,
 * so there is no budget left to wait out each CCC-clear write's response.
 *
 * Unconditional at the end of a `RunProtocol` step, unlike a monitor
 * *window*: a protocol run is scoped to one step by construction (§3 decision
 * 60 -- it "hands the link to a declared state machine for the length of one
 * step"), so leaving its subscriptions armed would be capturing into nothing
 * for the rest of the study. */
static void protocol_unsubscribe_all(const struct eap_protocol_def *def)
{
	for (uint8_t i = 0; i < protocol_subscribe_count; i++) {
		uint8_t src = protocol_subscribe_source[i];

		(void)bt_gatt_unsubscribe(active_conn, &protocol_subscribe_params[i]);
		transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_UNSUBSCRIBED,
				def->sources[src].service_uuid,
				def->sources[src].characteristic_uuid, 0, NULL, 0);
	}
	protocol_subscribe_count = 0;
}

/* One `on_enter` write. `with_response` selects the ATT operation and nothing
 * else -- **no transition is ever triggered by a write's own response**,
 * whichever it is (§3 decision 60): on the DUT this was designed against a
 * control-point write's response confirms only that the write was accepted,
 * and the authoritative answer arrives later on a different characteristic.
 * An acknowledged write is still *waited* for here, so two writes are never
 * in flight at once; notifications arriving during that wait queue up rather
 * than being lost. */
static struct outcome protocol_write(const struct eap_protocol_def *def,
				     const struct eap_step *step, int64_t deadline)
{
	uint8_t source = step->source;

	if (source >= def->sources_len) {
		return outcome_fail("protocol write names source %u, which does not exist",
				    (unsigned int)source);
	}

	transcript_emit(BLE_GATT_DIR_OUT, BLE_GATT_EVT_WRITE_REQUEST,
			def->sources[source].service_uuid,
			def->sources[source].characteristic_uuid, 0, step->payload,
			step->payload_len);

	if (!step->with_response) {
		int err = bt_gatt_write_without_response(active_conn,
							 protocol_source_handle[source],
							 step->payload, (uint16_t)step->payload_len,
							 false);

		if (err != 0) {
			return outcome_fail("protocol write failed (%d)", err);
		}
		return outcome_pass();
	}

	write_params.func = write_cb;
	write_params.handle = protocol_source_handle[source];
	write_params.offset = 0;
	write_params.data = step->payload;
	write_params.length = (uint16_t)step->payload_len;

	att_err = 0;
	link_lost = false;
	k_sem_reset(&gatt_sem);

	int err = bt_gatt_write(active_conn, &write_params);

	if (err != 0) {
		return outcome_fail("protocol write failed (%d)", err);
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		abandon(&write_params);
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected during protocol write");
	}
	if (att_err != 0) {
		return outcome_fail("protocol write rejected (ATT 0x%02x)", att_err);
	}
	return outcome_pass();
}

/* Copy an interpreter verdict into the step's own `struct outcome`.
 *
 * The step outcome and the protocol outcome are set from the same verdict
 * here because on a *clean* finish they agree by construction. They are two
 * fields rather than one precisely for the cases where they do not: a run cut
 * short by a lost link or an exhausted `timeout_ms` reports a step failure
 * whose protocol half still names the state the machine was sitting in. */
static void protocol_outcome_into(struct outcome *out, const struct eap_step *done)
{
	out->has_protocol = true;
	strncpy(out->protocol_final_state, done->final_state,
		sizeof(out->protocol_final_state) - 1);
	out->protocol_final_state[sizeof(out->protocol_final_state) - 1] = '\0';
	out->protocol_outcome_tag = done->outcome.tag;
	strncpy(out->protocol_fail_reason, done->outcome.fail_reason,
		sizeof(out->protocol_fail_reason) - 1);
	out->protocol_fail_reason[sizeof(out->protocol_fail_reason) - 1] = '\0';
}

/* The state's own deadline, or the step's when the state declared none. A
 * state with no `on_timeout` is not a state that can hang forever -- the
 * step's `timeout_ms` still bounds the whole run, which is exactly what
 * `Run::abandon` reports as `TimedOut`. */
static int64_t protocol_state_deadline(const struct eap_step *step, int64_t step_deadline)
{
	if (!step->has_deadline) {
		return step_deadline;
	}

	int64_t state_deadline = deadline_from(step->deadline_ms);

	return (state_deadline < step_deadline) ? state_deadline : step_deadline;
}

static struct outcome execute_run_protocol(const struct run_protocol_params *params,
					   int64_t deadline)
{
	const struct eap_protocol_def *def = params->def;

	if (active_conn == NULL) {
		return outcome_fail("no active connection -- run a BleConnect step first");
	}
	if (def == NULL || params->entry_state >= def->states_len) {
		/* Unreachable in practice: `validate_protocol` and Core's
		 * pre-flight both range-check `entry_state`, and main.c checks
		 * it again before dispatch. Named here anyway rather than left
		 * to a raw array subscript, which is §3 decision 18's rule. */
		return outcome_fail("protocol entry state %u does not exist",
				    (unsigned int)params->entry_state);
	}

	struct outcome resolved = protocol_resolve_sources(def, deadline);

	if (resolved.kind != OUTCOME_PASS) {
		return resolved;
	}

	k_msgq_purge(&protocol_notify_q);
	protocol_notifications_dropped = 0;

	struct outcome subscribed_result = protocol_subscribe_sources(def, deadline);

	if (subscribed_result.kind != OUTCOME_PASS) {
		protocol_unsubscribe_all(def);
		return subscribed_result;
	}

	struct eap_run run;
	struct eap_step step;

	(void)eap_run_start(&run, def, params->entry_state);
	eap_run_enter(&run, &step);

	struct outcome result = outcome_pass();

	for (;;) {
		if (step.kind == EAP_STEP_DONE) {
			if (step.outcome.tag == EAP_OUTCOME_PASS) {
				result = outcome_pass();
			} else if (step.outcome.tag == EAP_OUTCOME_TIMED_OUT) {
				result = outcome_timed_out();
			} else {
				result = outcome_fail("%s", step.outcome.fail_reason);
			}
			protocol_outcome_into(&result, &step);
			break;
		}

		if (step.kind == EAP_STEP_WRITE) {
			struct outcome write_result = protocol_write(def, &step, deadline);

			if (write_result.kind != OUTCOME_PASS) {
				/* The BLE half failed, not the machine. Report
				 * the failure as the step's, and the state the
				 * machine was sitting in as the protocol's --
				 * which is the pair that makes "the write was
				 * rejected in state `pumping`" readable at all. */
				struct eap_step where;

				eap_run_abandon(&run, &where);
				result = write_result;
				protocol_outcome_into(&result, &where);
				result.protocol_outcome_tag = EAP_OUTCOME_FAIL;
				break;
			}
		}

		/* Wait for whichever comes first: a frame, this state's own
		 * deadline, or the step's. */
		int64_t state_deadline = protocol_state_deadline(&step, deadline);
		struct protocol_notification note;

		if (k_msgq_get(&protocol_notify_q, &note, remaining(state_deadline)) == 0) {
			struct eap_event ev = {
				.kind = EAP_EVENT_NOTIFY,
				.source = note.source,
				.payload = note.payload,
				.payload_len = note.len,
			};

			transcript_emit(BLE_GATT_DIR_IN, BLE_GATT_EVT_NOTIFICATION,
					def->sources[note.source].service_uuid,
					def->sources[note.source].characteristic_uuid, 0,
					note.payload, note.len);
			eap_run_on_event(&run, &ev, &step);
			continue;
		}

		if (k_uptime_get() >= deadline) {
			/* The **step's** own timeout_ms ran out before the
			 * machine reached a terminal state -- the one way
			 * `Outcome::TimedOut` is produced, and the one outcome
			 * no manifest can declare. */
			eap_run_abandon(&run, &step);
			continue;
		}

		struct eap_event ev = {.kind = EAP_EVENT_TIMEOUT};

		eap_run_on_event(&run, &ev, &step);
	}

	protocol_unsubscribe_all(def);

	if (protocol_notifications_dropped > 0) {
		/* A run that missed a frame reached the wrong state, so this is
		 * said out loud rather than left for a reader to infer from a
		 * surprising `final_state`. */
		bridge_log("protocol run dropped %u notifications (queue full) -- its final state "
			   "is NOT trustworthy",
			   (unsigned int)protocol_notifications_dropped);
	}
	return result;
}

static struct outcome dispatch_action(const struct action *action, int64_t deadline)
{
	switch (action->kind) {
	case ACTION_BLE_ADVERTISE:
		return execute_advertise(&action->advertise);
	case ACTION_BLE_CONNECT:
		return execute_connect(&action->connect, deadline);
	case ACTION_DATA_EXCHANGE:
		return execute_data_exchange(&action->data_exchange, deadline);
	case ACTION_GATT_DISCOVER:
		return execute_gatt_discover(deadline);
	case ACTION_GATT_MONITOR_ALL:
		return execute_gatt_monitor_window(deadline, NULL);
	case ACTION_GATT_MONITOR_START:
		return execute_gatt_monitor_window_start(deadline, NULL);
	case ACTION_GATT_MONITOR_SELECTED:
		return execute_gatt_monitor_window(deadline, &action->monitor_selected);
	case ACTION_GATT_MONITOR_SELECTED_START:
		return execute_gatt_monitor_window_start(deadline, &action->monitor_selected);
	case ACTION_GATT_MONITOR_STOP:
		return execute_gatt_monitor_stop();
	case ACTION_BLE_SECURITY:
		return execute_set_security(&action->set_security, deadline);
	case ACTION_BLE_UNBOND:
		return execute_unbond();
	case ACTION_RUN_PROTOCOL:
		return execute_run_protocol(&action->run_protocol, deadline);
	default:
		return outcome_fail("unknown action kind");
	}
}

struct outcome ble_bridge_execute(const struct action *action, uint32_t timeout_ms)
{
	const int64_t deadline = deadline_from(timeout_ms);

	/* Each action reports only what it observed itself: no captured bytes and
	 * no already-pending notification carry over from a previous step.
	 * Note what is deliberately *not* reset here: the monitor
	 * subscriptions, which by design span steps once a window is open
	 * (design.md §3 decision 36) -- monitor_subscribe_matching clears them
	 * when a new window opens instead. */
	capture_reset();
	link_lost = false;
	k_sem_reset(&notify_sem);

	struct outcome outcome = dispatch_action(action, deadline);

	/* One place, for every action kind (ble_bridge.h's `struct outcome`
	 * contract). Deliberately after the dispatch rather than inside each
	 * arm: a per-arm stamp is a stamp somebody forgets to add to the next
	 * arm, and the whole value of reporting this on every step is that it
	 * is reported on every step.
	 *
	 * Read from the live connection at this moment, not cached: for
	 * ACTION_BLE_SECURITY that is the level it just reached, and for
	 * every other action it is the level the action ran *at* -- which is
	 * what makes "disconnected during service discovery" at L1 legible as
	 * a different finding from the same failure at L4. No connection means
	 * nothing to report, and says so. */
	if (active_conn != NULL) {
		uint8_t wire_level;

		if (from_bt_security(bt_conn_get_security(active_conn), &wire_level)) {
			outcome.has_security_level = true;
			outcome.security_level = wire_level;
		}
	}
	return outcome;
}

void ble_bridge_set_stream_handler(ble_stream_sample_handler handler, void *user_data)
{
	stream_handler = handler;
	stream_user_data = user_data;
}

void ble_bridge_set_transcript_sink(ble_transcript_sink sink, void *user_data)
{
	transcript_sink = sink;
	transcript_user_data = user_data;
}

bool ble_bridge_monitor_window_open(void)
{
	return monitor_window_open;
}

void ble_bridge_reset(void)
{
	(void)bt_le_scan_stop();
	(void)bt_le_adv_stop();
	release_pending_conn(true);
	advertising = false;
	streaming = false;

	if (subscribe_params.value_handle != 0 && active_conn != NULL) {
		(void)bt_gatt_unsubscribe(active_conn, &subscribe_params);
	}
	subscribed = false;
	memset(&subscribe_params, 0, sizeof(subscribe_params));

	/* GattMonitorAll's own subscription set (design.md §3 decision 32) --
	 * torn down here too, before the disconnect below, same reasoning as
	 * the single subscribe_params case just above. */
	if (active_conn != NULL) {
		for (uint8_t i = 0; i < monitor_subscribe_count; i++) {
			(void)bt_gatt_unsubscribe(active_conn, &monitor_subscribe_params[i]);
		}
	}
	monitor_subscribe_count = 0;
	/* A Hello is a hard reset (design.md §3 decision 12/16) -- any window
	 * left open by a previous study dies with it. */
	monitor_window_open = false;
	discovered_len = 0;

	if (active_conn != NULL) {
		/* disconnected_cb drops the reference and clears active_conn; don't
		 * unref here as well.
		 *
		 * And *wait* for it. bt_conn_disconnect only requests the
		 * disconnect -- active_conn stays set until disconnected_cb runs
		 * on the BT RX thread. Returning before that made a `Hello`'s
		 * "hard reset" contract a lie: the very next study's BleConnect
		 * would find active_conn still populated and fail with "already
		 * connected to a different peer", which is exactly what happened
		 * running two back-to-back studies against different addresses in
		 * Milestone 6 -- the first attempt failed, the identical retry
		 * passed, which is the signature of a race rather than a
		 * configuration problem.
		 *
		 * A fixed bound rather than a caller-supplied deadline: this
		 * function is `void` and is a reset, not a step, so it has no
		 * budget to draw from. Two seconds is far longer than a local
		 * disconnect needs; timing out anyway leaves the stale conn and
		 * the next connect fails as before, which is no worse than the
		 * unconditional behavior this replaces. */
		k_sem_reset(&disconn_sem);
		if (bt_conn_disconnect(active_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN) == 0) {
			(void)k_sem_take(&disconn_sem, K_MSEC(2000));
		}
	}

	handle_cache.valid = false;
	capture_reset();

	ble_bridge_clear_bonds();
}

void ble_bridge_clear_bonds(void)
{
	bool was_connected = active_conn != NULL;

	if (was_connected) {
		k_sem_reset(&disconn_sem);
	}

	(void)bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);

	/* Same wait, and the same reason, as execute_unbond's: bt_unpair
	 * disconnects a peer whose keys it clears, and returning while
	 * `active_conn` is still populated is the race a Hello's own
	 * "hard reset" contract already got caught lying about once. Called
	 * from ble_bridge_reset() *after* its own disconnect, where there is
	 * usually nothing left to wait for -- and from main.c at the end of
	 * every study, where there usually is. */
	if (was_connected) {
		(void)k_sem_take(&disconn_sem, K_MSEC(2000));
	}

	handle_cache.valid = false;
	subscribed = false;
}
