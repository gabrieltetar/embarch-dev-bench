/* The `.eap` protocol interpreter -- embarch-study-designer/design.md §3
 * decisions 58-62, §4.9.
 *
 * # This is the executor, and the Rust one is the specification
 *
 * That doc's decision 60 puts the **real** interpreter here rather than in
 * Core: the loop closes against the DUT's own BLE connection interval, Core
 * still sends nothing after `StudyStart`, and main.c's receive-then-run model
 * is unchanged. What lives in `embarch-study-designer/src/eap_interp.rs` is
 * the *reference* -- the semantics this file has to agree with, executable so
 * a cross-language test can pin them. Where the two disagree, that crate is
 * right, exactly as it is for the wire encoding.
 *
 * The division is the one §3 decisions 31/32 already set: the wire types and
 * the reference semantics live in the crate, live BLE dispatch lives here.
 *
 * # Events in, instructions out -- and no radio anywhere
 *
 * Nothing in this module opens a connection, subscribes to anything, or knows
 * what time it is. A caller feeds it arrivals and timer expiries and reads
 * back the writes it wants performed. That is deliberate and it is what makes
 * the semantics testable on a host: every branch below -- the
 * `remember`-before-guards ordering, first-guard-wins, saturating `+`, an
 * unresolvable operand making a guard false and a write not happen, `retry`
 * re-running `on_enter` rather than waiting longer -- is exercised by
 * app/tests/serial_protocol without a DUT in the room.
 *
 * # One behavior that is easy to get wrong, because it already was once
 *
 * An `on_event` arm with **no** `otherwise` consumes the frame, applies its
 * `remember`s, and stays in the state *without re-entering it*: no `on_enter`
 * re-send, and the timeout keeps counting from the original entry.
 * `otherwise: goto <this state>` is a different thing -- it re-enters, which
 * re-sends the flow-control ack and restarts the stall watchdog. A
 * flow-controlled pump loop needs the second, and the first draft of the
 * crate's own fixture wrote the first: it consumed every chunk correctly,
 * acked none of them, and stalled at the watchdog. See
 * embarch-decision-reversals.md row 66.
 */
#ifndef EMBARCH_DEV_BENCH_EAP_INTERP_H_
#define EMBARCH_DEV_BENCH_EAP_INTERP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "eap.h"

/* Mirrors `Outcome` (embarch-study-designer/src/result.rs). `TimedOut` is the
 * one a manifest cannot declare and only a run can produce. */
enum eap_outcome_tag {
	EAP_OUTCOME_PASS = 0,
	EAP_OUTCOME_FAIL = 1,
	EAP_OUTCOME_TIMED_OUT = 2,
};

/* Largest `write` payload a manifest can assemble: every field of an
 * `on_enter` write at its widest. Sized from the grammar rather than borrowed
 * from the link protocol's own DBM_MAX_PAYLOAD_LEN, which would be 512 bytes
 * of stack for a
 * control-point opcode and an argument. */
#define EAP_MAX_WRITE_PAYLOAD_LEN (EAP_MAX_WRITE_FIELDS * 8)

/* Something that happened, from the interpreter's point of view. */
enum eap_event_kind {
	/* A notification arrived on the characteristic bound to this source
	 * index, carrying these bytes. */
	EAP_EVENT_NOTIFY = 0,
	/* The current state's `on_timeout` deadline expired. */
	EAP_EVENT_TIMEOUT = 1,
};

struct eap_event {
	uint8_t kind;   /* enum eap_event_kind */
	uint8_t source; /* index into eap_protocol_def.sources; NOTIFY only */
	const uint8_t *payload;
	size_t payload_len;
};

/* What the caller should do next. */
enum eap_step_kind {
	/* Perform a GATT write of `payload` to `source`, then wait. */
	EAP_STEP_WRITE = 0,
	/* Wait for the next event. */
	EAP_STEP_WAIT = 1,
	/* The machine reached a terminal state, or was abandoned. */
	EAP_STEP_DONE = 2,
};

/* One instruction.
 *
 * **A write's own ATT response is never fed back in.** There is no event for
 * one, deliberately (design.md §3 decision 60): on the DUT this was designed
 * against, a control-point write's response confirms only that the write was
 * *accepted*, and the authoritative answer arrives later as an independent
 * notification on a different characteristic. This says what to send; only a
 * notification or a timeout moves the machine.
 */
struct eap_step {
	uint8_t kind; /* enum eap_step_kind */

	/* EAP_STEP_WRITE */
	uint8_t source;
	bool with_response;
	uint8_t payload[EAP_MAX_WRITE_PAYLOAD_LEN];
	size_t payload_len;

	/* EAP_STEP_WRITE and EAP_STEP_WAIT: the state's own timeout, if it
	 * declared one. A write arms it too -- a write is not a state of its
	 * own, it is what entering a state does before it starts waiting. */
	bool has_deadline;
	uint32_t deadline_ms;

	/* EAP_STEP_DONE -- `ProtocolOutcome { final_state, outcome }`. */
	char final_state[EAP_MAX_STATE_NAME_LEN + 1];
	struct {
		uint8_t tag; /* enum eap_outcome_tag */
		char fail_reason[EAP_MAX_FAIL_REASON_LEN + 1];
	} outcome;
};

/* A protocol run in progress. Borrows `def` for its whole lifetime -- which
 * is one `Action::RunProtocol` step, and `def` lives in the decoded
 * `struct dbm_study_start` that outlives every step. */
struct eap_run {
	const struct eap_protocol_def *def;
	int64_t session[EAP_MAX_SESSION_VARS];
	uint8_t state;
	/* Retries already spent in the current state, reset on every entry --
	 * which is why `eap_run_enter` and the internal re-entry an
	 * `on_timeout` retry performs are not the same call. */
	uint8_t retries;
	bool finished;
};

/* Start a run at `entry_state`, initialising every session variable to its
 * declared value. Returns 0, or -1 if `entry_state` is out of range -- which
 * `validate_protocol` and Core's pre-flight both check before this can be
 * reached, so it is belt and braces rather than the real defence. */
int eap_run_start(struct eap_run *run, const struct eap_protocol_def *def, uint8_t entry_state);

/* What to do on entering the current state. Call once after `eap_run_start`;
 * the machine calls it for itself on every transition. */
void eap_run_enter(struct eap_run *run, struct eap_step *out);

/* Feed one event and get the next instruction. */
void eap_run_on_event(struct eap_run *run, const struct eap_event *ev, struct eap_step *out);

/* End the run because the *step's* own `timeout_ms` expired before the
 * machine reached a terminal state. Reports `TimedOut` against whatever state
 * it was sitting in -- the one outcome no manifest can declare. */
void eap_run_abandon(struct eap_run *run, struct eap_step *out);

/* The state the run is currently in, as an index into `def->states`. */
uint8_t eap_run_state(const struct eap_run *run);

/* Session variables, in declaration order.
 *
 * Exposed for tests and for offline replay, and **not** reported in
 * `ProtocolOutcome` -- design.md §3 decision 62 kept decoded values out of a
 * result on purpose, and a session variable is a decoded value that survived.
 * Mirrors `Run::session()` in the Rust reference, which exists for the same
 * two callers and neither of them is the wire. */
const int64_t *eap_run_session(const struct eap_run *run);

#endif /* EMBARCH_DEV_BENCH_EAP_INTERP_H_ */
