/* Real Zephyr BT host calls, built by workspaces/nordic/ (embarch-dev-bench/design.md
 * §3 decision 16). Only calls stable, vendor-neutral Zephyr Bluetooth host
 * APIs (`bt_*`) per decision 3 -- nothing NCS-proprietary.
 *
 * This bring-up pass only wires up enough to run main.c's fixed demo sequence
 * (BleAdvertise) end to end on real hardware -- BleConnect/DataExchange, and
 * honoring a Step's own timeout_ms via a real async wait, are deferred along
 * with the rest of real Study execution (decision 20; see main.c's
 * run_demo_sequence comment). advertise's local_name/service_uuids aren't
 * wired into the advertising data yet either: BT_LE_ADV_CONN_NAME advertises
 * whatever CONFIG_BT_DEVICE_NAME is set to, not the Action's own local_name.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "ble_bridge.h"

int ble_bridge_init(void)
{
	return bt_enable(NULL);
}

static struct outcome outcome_pass(void)
{
	return (struct outcome){.kind = OUTCOME_PASS};
}

static struct outcome outcome_fail(const char *reason)
{
	struct outcome outcome = {.kind = OUTCOME_FAIL};

	strncpy(outcome.fail_reason, reason, OUTCOME_MAX_FAIL_REASON_LEN);
	outcome.fail_reason[OUTCOME_MAX_FAIL_REASON_LEN] = '\0';
	return outcome;
}

static struct outcome execute_advertise(const struct ble_advertise_params *params)
{
	(void)params;

	int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);

	if (err != 0 && err != -EALREADY) {
		return outcome_fail("bt_le_adv_start failed");
	}
	return outcome_pass();
}

struct outcome ble_bridge_execute(const struct action *action, uint32_t timeout_ms)
{
	(void)timeout_ms;

	switch (action->kind) {
	case ACTION_BLE_ADVERTISE:
		return execute_advertise(&action->advertise);
	case ACTION_BLE_CONNECT:
	case ACTION_DATA_EXCHANGE:
		return outcome_fail("not implemented in this bring-up pass");
	default:
		return outcome_fail("unknown action kind");
	}
}

void ble_bridge_reset(void)
{
	bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
}
