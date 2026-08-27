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

/* Same posture for the GATT transcript (design.md §3 decision 36): stored so
 * the setter behaves identically on both sides of the decision-16 split, never
 * invoked, because a canned Pass observed no GATT traffic to transcribe.
 * Emitting invented entries here would let a native_sim run produce a
 * transcript indistinguishable from a real capture -- exactly the failure this
 * stub's "no captured_data either" rule already guards against. */
static ble_transcript_sink transcript_sink;
static ble_log_sink log_sink;
static void *transcript_user_data;

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
	 * data.
	 *
	 * And no `security_level` (embarch-study-designer/design.md §3 decision
	 * 44), for exactly the
	 * same reason and more sharply: there is no link here, so there is no
	 * level. A stub that reported L4 would let a native_sim run produce a
	 * StepResult indistinguishable from a real authenticated pairing --
	 * the one claim this suite must never manufacture. `has_security_level`
	 * stays false, which reads as "there was no connection to ask about",
	 * which is the truth.
	 *
	 * And no `protocol` either (embarch-study-designer/design.md §3
	 * decision 62), for the same reason a third time and the sharpest one
	 * yet: an ACTION_RUN_PROTOCOL step here runs no state machine, so there
	 * is no state it stopped in. `has_protocol` stays false, which reads as
	 * "no protocol ran", which is the truth. Reporting a `done` here would
	 * let a native_sim run produce a StepResult indistinguishable from a
	 * real handshake completing against a real DUT -- and the interpreter's
	 * own semantics are covered where they can be covered honestly, by
	 * app/tests/serial_protocol driving eap_interp.c directly. */
	return (struct outcome){.kind = OUTCOME_PASS};
}

void ble_bridge_set_stream_handler(ble_stream_sample_handler handler, void *user_data)
{
	stream_handler = handler;
	stream_user_data = user_data;
}

void ble_bridge_set_log_sink(ble_log_sink sink, void *user_data)
{
	log_sink = sink;
	(void)user_data;
	(void)log_sink;
}

void ble_bridge_set_transcript_sink(ble_transcript_sink sink, void *user_data)
{
	transcript_sink = sink;
	transcript_user_data = user_data;
}

/* The stub has no radio and therefore no notification queue to overflow, so
 * it accepts the registration and never calls it -- the same posture every
 * other sink here takes. */
void ble_bridge_set_drop_sink(ble_drop_sink sink, void *user_data)
{
	(void)sink;
	(void)user_data;
}

bool ble_bridge_monitor_window_open(void)
{
	/* No window is ever opened here, so main.c never has one to close. */
	return false;
}

void ble_bridge_clear_bonds(void)
{
	/* No bonding table without a BT host. */
}

void ble_bridge_reset(void)
{
	(void)stream_handler;
	(void)stream_user_data;
	(void)transcript_sink;
	(void)transcript_user_data;
}
