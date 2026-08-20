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
 * NOT handled here, deliberately: elevating the link to an encrypted/paired
 * one. Zephyr's own ATT layer already re-runs a request at higher security
 * when a DUT answers with an authentication/encryption ATT error, and
 * decision 11's Just Works pairing needs no auth callbacks, so an explicit
 * bt_conn_set_security() call would only duplicate that.
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

#include "ble_bridge.h"

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

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
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

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	ARG_UNUSED(rssi);
	ARG_UNUSED(buf);

	if (scan_matched) {
		return;
	}
	/* Only a connectable advertiser can be connected to; skip scan responses
	 * and non-connectable beacons rather than failing on them. */
	if (adv_type != BT_GAP_ADV_TYPE_ADV_IND && adv_type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}
	if (scan_target_set && bt_addr_le_cmp(addr, &scan_target) != 0) {
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
	scan_matched = false;
	conn_err = 0;
	k_sem_reset(&conn_sem);

	int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);

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
		return BT_GATT_ITER_CONTINUE;
	}

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

	int err = bt_gatt_read(active_conn, &read_params);

	if (err != 0) {
		return outcome_fail("bt_gatt_read failed (%d)", err);
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		abandon(&read_params);
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected during read");
	}
	if (att_err != 0) {
		return outcome_fail("read rejected (ATT 0x%02x)", att_err);
	}
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

	int err = bt_gatt_write(active_conn, &write_params);

	if (err != 0) {
		return outcome_fail("bt_gatt_write failed (%d)", err);
	}
	if (k_sem_take(&gatt_sem, remaining(deadline)) != 0) {
		abandon(&write_params);
		return outcome_timed_out();
	}
	if (link_lost) {
		return outcome_fail("disconnected during write");
	}
	if (att_err != 0) {
		return outcome_fail("write rejected (ATT 0x%02x)", att_err);
	}
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
		 * samples arrived is Core-side post-hoc validation's question
		 * (embarch-study-designer/design.md §3 decision 19). */
		return outcome_pass();
	}

	default:
		return outcome_fail("unknown GATT operation kind");
	}
}

/* ---- public API -------------------------------------------------------- */

int ble_bridge_init(void)
{
	int err = bt_conn_cb_register(&conn_callbacks);

	if (err != 0) {
		return err;
	}
	return bt_enable(NULL);
}

struct outcome ble_bridge_execute(const struct action *action, uint32_t timeout_ms)
{
	const int64_t deadline = deadline_from(timeout_ms);

	/* Each action reports only what it observed itself: no captured bytes and
	 * no already-pending notification carry over from a previous step. */
	capture_reset();
	link_lost = false;
	k_sem_reset(&notify_sem);

	switch (action->kind) {
	case ACTION_BLE_ADVERTISE:
		return execute_advertise(&action->advertise);
	case ACTION_BLE_CONNECT:
		return execute_connect(&action->connect, deadline);
	case ACTION_DATA_EXCHANGE:
		return execute_data_exchange(&action->data_exchange, deadline);
	default:
		return outcome_fail("unknown action kind");
	}
}

void ble_bridge_set_stream_handler(ble_stream_sample_handler handler, void *user_data)
{
	stream_handler = handler;
	stream_user_data = user_data;
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

	if (active_conn != NULL) {
		/* disconnected_cb drops the reference and clears active_conn; don't
		 * unref here as well. */
		(void)bt_conn_disconnect(active_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}

	handle_cache.valid = false;
	capture_reset();

	bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
}
