/* No-BLE-host stub, built by workspaces/native_sim/ (embarch-dev-bench/design.md
 * §3 decision 16). Every action reports a canned Pass outcome — this exists so
 * the serial/dispatch flow (main.c) is exercisable on a host machine with no
 * radio at all, not to model real BLE behavior.
 *
 * Kept in signature-sync with ble_bridge_real.c by hand, which is decision 16's
 * accepted cost for keeping the real BLE-host code free of stub-only branches.
 */
#include <stddef.h>

#include "ble_bridge.h"

/* Registered but never invoked: with no radio there are no notifications to
 * stream. Stored anyway so the setter behaves identically on both sides of the
 * decision-16 split. */
static ble_stream_sample_handler stream_handler;
static void *stream_user_data;

int ble_bridge_init(void)
{
	return 0;
}

struct outcome ble_bridge_execute(const struct action *action, uint32_t timeout_ms)
{
	(void)action;
	(void)timeout_ms;
	/* No captured_data either: a canned Pass has no DUT bytes behind it, and
	 * inventing some would let a native_sim run look like it exchanged real
	 * data. */
	return (struct outcome){.kind = OUTCOME_PASS};
}

void ble_bridge_set_stream_handler(ble_stream_sample_handler handler, void *user_data)
{
	stream_handler = handler;
	stream_user_data = user_data;
}

void ble_bridge_reset(void)
{
	(void)stream_handler;
	(void)stream_user_data;
}
