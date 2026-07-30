/* No-BLE-host stub, built by workspaces/native_sim/ (embarch-dev-bench/design.md
 * §3 decision 16). Every action reports a canned Pass outcome — this exists so
 * the serial/dispatch flow (main.c) is exercisable on a host machine with no
 * radio at all, not to model real BLE behavior.
 */
#include "ble_bridge.h"

int ble_bridge_init(void)
{
	return 0;
}

struct outcome ble_bridge_execute(const struct action *action, uint32_t timeout_ms)
{
	(void)action;
	(void)timeout_ms;
	return (struct outcome){.kind = OUTCOME_PASS};
}

void ble_bridge_reset(void)
{
}
