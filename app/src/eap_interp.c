/* The `.eap` protocol interpreter -- embarch-study-designer/design.md §3
 * decisions 58-62, §4.9.
 *
 * See eap_interp.h for what this is and what it deliberately is not. The
 * short version: `embarch-study-designer/src/eap_interp.rs` is the executable
 * specification and this file has to agree with it, the same way
 * serial_protocol.c's postcard walker has to agree with that crate's own
 * encoding.
 *
 * No Zephyr, no BLE, no I/O -- events in, instructions out. That is what
 * makes the semantics testable on any host (app/tests/serial_protocol)
 * instead of only against a DUT.
 */

#include "eap_interp.h"

#include <string.h>

/* ---- `ScalarType`, the two directions ------------------------------------
 *
 * Mirrors `embarch-study-designer/src/decoder.rs`'s `ScalarType::read_i64`
 * and `write_i64`. The same 18-variant enum decision 52's `StructLayout`
 * uses, which is the whole point of `ScalarRead` reusing it: a frame lowered
 * into a layout for rendering reads its bytes through the identical rules a
 * guard does.
 */

static size_t scalar_width(uint8_t ty)
{
	switch (ty) {
	case EAP_SCALAR_U8:
	case EAP_SCALAR_I8:
		return 1;
	case EAP_SCALAR_U16LE:
	case EAP_SCALAR_U16BE:
	case EAP_SCALAR_I16LE:
	case EAP_SCALAR_I16BE:
		return 2;
	case EAP_SCALAR_U32LE:
	case EAP_SCALAR_U32BE:
	case EAP_SCALAR_I32LE:
	case EAP_SCALAR_I32BE:
	case EAP_SCALAR_F32LE:
	case EAP_SCALAR_F32BE:
		return 4;
	case EAP_SCALAR_U64LE:
	case EAP_SCALAR_U64BE:
	case EAP_SCALAR_I64LE:
	case EAP_SCALAR_I64BE:
	case EAP_SCALAR_F64LE:
	case EAP_SCALAR_F64BE:
		return 8;
	default:
		return 0;
	}
}

static bool scalar_is_big_endian(uint8_t ty)
{
	switch (ty) {
	case EAP_SCALAR_U16BE:
	case EAP_SCALAR_I16BE:
	case EAP_SCALAR_U32BE:
	case EAP_SCALAR_I32BE:
	case EAP_SCALAR_U64BE:
	case EAP_SCALAR_I64BE:
		return true;
	default:
		return false;
	}
}

static bool scalar_is_signed(uint8_t ty)
{
	switch (ty) {
	case EAP_SCALAR_I8:
	case EAP_SCALAR_I16LE:
	case EAP_SCALAR_I16BE:
	case EAP_SCALAR_I32LE:
	case EAP_SCALAR_I32BE:
	case EAP_SCALAR_I64LE:
	case EAP_SCALAR_I64BE:
		return true;
	default:
		return false;
	}
}

static bool scalar_is_integer(uint8_t ty)
{
	switch (ty) {
	case EAP_SCALAR_F32LE:
	case EAP_SCALAR_F32BE:
	case EAP_SCALAR_F64LE:
	case EAP_SCALAR_F64BE:
		return false;
	default:
		return ty <= EAP_SCALAR_I64BE;
	}
}

/* Read one scalar out of a payload. Returns false when the payload is too
 * short to contain it -- a real runtime case (a truncated notification), and
 * deliberately **not** a zero: the caller turns an unresolvable read into a
 * false guard and a write that does not happen. */
static bool scalar_read_i64(uint8_t ty, const uint8_t *bytes, size_t len, int64_t *out)
{
	size_t w = scalar_width(ty);

	if (w == 0 || !scalar_is_integer(ty) || len < w) {
		return false;
	}

	uint64_t v = 0;

	for (size_t i = 0; i < w; i++) {
		size_t src = scalar_is_big_endian(ty) ? i : (w - 1 - i);

		v = (v << 8) | bytes[src];
	}

	if (scalar_is_signed(ty) && w < 8) {
		uint64_t sign_bit = (uint64_t)1 << ((w * 8) - 1);

		if (v & sign_bit) {
			v |= ~(((uint64_t)1 << (w * 8)) - 1);
		}
		*out = (int64_t)v;
	} else if (ty == EAP_SCALAR_U64LE || ty == EAP_SCALAR_U64BE) {
		/* Saturated at i64::MAX, matching `read_i64`'s own `.min()`:
		 * the expression set is `i64`, and a `u64` past its ceiling has
		 * no honest representation in it. */
		*out = (v > (uint64_t)INT64_MAX) ? INT64_MAX : (int64_t)v;
	} else {
		*out = (int64_t)v;
	}
	return true;
}

/* Write one scalar into a `write` payload, truncating to the declared width.
 * The truncation is deliberate and matches what a C firmware assembling the
 * same packet would do with a cast. */
static bool scalar_write_i64(uint8_t ty, int64_t value, uint8_t *out, size_t out_cap, size_t *n)
{
	size_t w = scalar_width(ty);

	if (w == 0 || !scalar_is_integer(ty) || out_cap < w) {
		return false;
	}

	uint64_t raw = (uint64_t)value;

	for (size_t i = 0; i < w; i++) {
		size_t shift = scalar_is_big_endian(ty) ? ((w - 1 - i) * 8) : (i * 8);

		out[i] = (uint8_t)(raw >> shift);
	}
	*n = w;
	return true;
}

/* ---- The expression set (eap.rs's `eval_*`) ----------------------------- */

/* Resolve one operand against the current run state.
 *
 * `frame`/`payload` are the frame that triggered the current event, absent
 * for an `on_enter` write. Returns false for a reference `validate_protocol`
 * should already have rejected, or for a payload too short to contain a
 * declared field -- the latter is a real runtime case and is deliberately not
 * a zero.
 */
static bool eval_operand(const struct eap_operand *op, const struct eap_run *run,
			 const struct eap_frame *frame, const uint8_t *payload,
			 size_t payload_len, int64_t *out)
{
	switch (op->kind) {
	case EAP_OP_LITERAL:
		*out = op->literal;
		return true;
	case EAP_OP_SESSION:
		if (op->index >= run->def->session_len) {
			return false;
		}
		*out = run->session[op->index];
		return true;
	case EAP_OP_FIELD: {
		if (frame == NULL || op->index >= frame->fields_len) {
			return false;
		}

		const struct eap_scalar_read *read = &frame->fields[op->index];

		if (read->offset > payload_len) {
			return false;
		}
		return scalar_read_i64(read->ty, payload + read->offset,
				       payload_len - read->offset, out);
	}
	case EAP_OP_SPAN_LEN: {
		if (frame == NULL || op->index >= frame->spans_len) {
			return false;
		}

		const struct eap_span_read *span = &frame->spans[op->index];

		if (span->offset > payload_len) {
			return false;
		}

		size_t available = payload_len - span->offset;

		if (!span->has_len) {
			*out = (int64_t)available;
			return true;
		}
		/* A declared fixed length longer than what arrived is a short
		 * payload, not a silently shorter span. */
		if ((size_t)span->len > available) {
			return false;
		}
		*out = (int64_t)span->len;
		return true;
	}
	default:
		return false;
	}
}

/* `Add` **saturates**. A wrapping counter would be a plausible wrong number;
 * a saturated one stops a pump loop's guard from ever passing again, which is
 * a stall the step timeout catches and reports. */
static bool eval_expr(const struct eap_expr *e, const struct eap_run *run,
		      const struct eap_frame *frame, const uint8_t *payload,
		      size_t payload_len, int64_t *out)
{
	int64_t a;

	if (!eval_operand(&e->a, run, frame, payload, payload_len, &a)) {
		return false;
	}
	if (e->kind == EAP_EXPR_TERM) {
		*out = a;
		return true;
	}

	int64_t b;

	if (!eval_operand(&e->b, run, frame, payload, payload_len, &b)) {
		return false;
	}
	if (b > 0 && a > INT64_MAX - b) {
		*out = INT64_MAX;
	} else if (b < 0 && a < INT64_MIN - b) {
		*out = INT64_MIN;
	} else {
		*out = a + b;
	}
	return true;
}

/* An operand that cannot be resolved makes the guard **false**, never true: a
 * comparison against a field a truncated payload did not carry must not be
 * the thing that advances a state machine. */
static bool eval_condition(const struct eap_condition *c, const struct eap_run *run,
			   const struct eap_frame *frame, const uint8_t *payload,
			   size_t payload_len)
{
	int64_t l;
	int64_t r;

	if (!eval_operand(&c->lhs, run, frame, payload, payload_len, &l) ||
	    !eval_operand(&c->rhs, run, frame, payload, payload_len, &r)) {
		return false;
	}
	switch (c->op) {
	case EAP_CMP_EQ:
		return l == r;
	case EAP_CMP_NE:
		return l != r;
	case EAP_CMP_LT:
		return l < r;
	case EAP_CMP_LE:
		return l <= r;
	case EAP_CMP_GT:
		return l > r;
	case EAP_CMP_GE:
		return l >= r;
	default:
		return false;
	}
}

/* Whether `payload` carries this match's magic at its declared offset.
 *
 * A payload too short to contain the match does **not** match -- never a
 * partial hit, because a truncated notification and a different format are
 * different facts. */
static bool frame_match(const struct eap_frame *f, const uint8_t *payload, size_t payload_len)
{
	if (!f->has_select) {
		return true;
	}
	if ((size_t)f->select_offset + f->select_len > payload_len) {
		return false;
	}
	return memcmp(payload + f->select_offset, f->select_eq, f->select_len) == 0;
}

/* First matching `select_if` wins; an unguarded frame matches anything. That
 * ordering is the only format-versioning mechanism a manifest has, which is
 * why `validate_protocol` refuses an unguarded frame declared ahead of a
 * guarded sibling on the same source. */
static bool select_frame(const struct eap_protocol_def *def, uint8_t source,
			 const uint8_t *payload, size_t payload_len, uint8_t *out)
{
	for (uint8_t i = 0; i < def->frames_len; i++) {
		if (def->frames[i].source != source) {
			continue;
		}
		if (frame_match(&def->frames[i], payload, payload_len)) {
			*out = i;
			return true;
		}
	}
	return false;
}

/* Assemble a `write` payload from its typed fields (design.md §3 decision
 * 61).
 *
 * Returns false if any operand fails to resolve -- the write is **not** sent
 * with a zero substituted in, because a control-point opcode carrying a
 * silently-wrong argument is worse than a step that fails saying so. */
static bool encode_write(const struct eap_write *w, const struct eap_run *run,
			 const struct eap_frame *frame, const uint8_t *payload,
			 size_t payload_len, uint8_t *out, size_t out_cap, size_t *out_len)
{
	size_t n = 0;

	for (uint8_t i = 0; i < w->fields_len; i++) {
		int64_t v;
		size_t written;

		if (!eval_operand(&w->fields[i].value, run, frame, payload, payload_len, &v)) {
			return false;
		}
		if (!scalar_write_i64(w->fields[i].ty, v, out + n, out_cap - n, &written)) {
			return false;
		}
		n += written;
	}
	*out_len = n;
	return true;
}

/* ---- The machine ------------------------------------------------------- */

static void set_outcome(const struct eap_run *run, struct eap_step *step, uint8_t outcome_tag,
			const char *fail_reason)
{
	step->kind = EAP_STEP_DONE;
	step->outcome.tag = outcome_tag;
	step->outcome.fail_reason[0] = '\0';
	if (fail_reason != NULL) {
		strncpy(step->outcome.fail_reason, fail_reason,
			sizeof(step->outcome.fail_reason) - 1);
		step->outcome.fail_reason[sizeof(step->outcome.fail_reason) - 1] = '\0';
	}
	strncpy(step->final_state, run->def->states[run->state].name,
		sizeof(step->final_state) - 1);
	step->final_state[sizeof(step->final_state) - 1] = '\0';
}

/* `"protocol reached terminal state <name>"` -- the same sentence
 * `eap_interp.rs`'s `fail_reason` builds, so a run that failed reads
 * identically whichever interpreter produced it. No comma and no quote, the
 * rule every string this suite renders into a CSV column follows. */
static void terminal_fail_reason(const char *state, char *out, size_t out_cap)
{
	static const char prefix[] = "protocol reached terminal state ";
	size_t n = sizeof(prefix) - 1;

	if (n >= out_cap) {
		n = out_cap - 1;
	}
	memcpy(out, prefix, n);

	size_t room = out_cap - 1 - n;
	size_t sl = strlen(state);

	if (sl > room) {
		sl = room;
	}
	memcpy(out + n, state, sl);
	out[n + sl] = '\0';
}

static void wait_step(const struct eap_state *st, struct eap_step *out)
{
	out->kind = EAP_STEP_WAIT;
	out->has_deadline = st->has_on_timeout;
	out->deadline_ms = st->has_on_timeout ? st->on_timeout.after_ms : 0;
}

/* Entering the current state: its `on_enter` write if it has one, otherwise a
 * wait. Split from `eap_run_enter` because an `on_timeout` retry re-runs this
 * **without** resetting the retry counter -- which is what makes `retry` mean
 * "send it again" rather than "wait longer". */
static void on_entered(struct eap_run *run, struct eap_step *out)
{
	const struct eap_state *st = &run->def->states[run->state];

	memset(out, 0, sizeof(*out));

	if (st->kind == EAP_STATE_TERMINAL) {
		run->finished = true;
		if (st->terminal == EAP_TERMINAL_PASS) {
			set_outcome(run, out, EAP_OUTCOME_PASS, NULL);
		} else {
			char reason[EAP_MAX_FAIL_REASON_LEN + 1];

			terminal_fail_reason(st->name, reason, sizeof(reason));
			set_outcome(run, out, EAP_OUTCOME_FAIL, reason);
		}
		return;
	}

	if (!st->has_on_enter) {
		wait_step(st, out);
		return;
	}

	size_t payload_len = 0;

	if (!encode_write(&st->on_enter, run, NULL, NULL, 0, out->payload, sizeof(out->payload),
			  &payload_len)) {
		/* An unresolvable operand does not become a zero: the write is
		 * not sent, and the run ends saying so. */
		run->finished = true;
		set_outcome(run, out, EAP_OUTCOME_FAIL, "protocol write operand did not resolve");
		return;
	}
	out->kind = EAP_STEP_WRITE;
	out->source = st->on_enter.source;
	out->payload_len = payload_len;
	out->with_response = st->on_enter.with_response;
	/* The caller arms this state's own deadline alongside the write: a
	 * `Write` is not a state of its own, it is what entering this state
	 * does before it starts waiting. */
	out->has_deadline = st->has_on_timeout;
	out->deadline_ms = st->has_on_timeout ? st->on_timeout.after_ms : 0;
}

int eap_run_start(struct eap_run *run, const struct eap_protocol_def *def, uint8_t entry_state)
{
	if (def == NULL || entry_state >= def->states_len) {
		return -1;
	}
	memset(run, 0, sizeof(*run));
	run->def = def;
	run->state = entry_state;
	for (uint8_t i = 0; i < def->session_len; i++) {
		run->session[i] = def->session_initial[i];
	}
	return 0;
}

void eap_run_enter(struct eap_run *run, struct eap_step *out)
{
	run->retries = 0;
	on_entered(run, out);
}

uint8_t eap_run_state(const struct eap_run *run)
{
	return run->state;
}

const int64_t *eap_run_session(const struct eap_run *run)
{
	return run->session;
}

static void on_timeout(struct eap_run *run, struct eap_step *out)
{
	const struct eap_state *st = &run->def->states[run->state];

	if (!st->has_on_timeout) {
		/* No declared timeout means this state has no deadline of its
		 * own; the step's own `timeout_ms` still bounds the whole run. */
		memset(out, 0, sizeof(*out));
		out->kind = EAP_STEP_WAIT;
		return;
	}
	if (run->retries < st->on_timeout.retry) {
		run->retries++;
		/* Re-run `on_enter` and wait again -- **not** wait longer. */
		on_entered(run, out);
		return;
	}
	run->state = st->on_timeout.goto_state;
	eap_run_enter(run, out);
}

static void on_notify(struct eap_run *run, uint8_t source, const uint8_t *payload,
		      size_t payload_len, struct eap_step *out)
{
	const struct eap_state *st = &run->def->states[run->state];
	uint8_t fi;

	/* A frame no arm of this state names is **ignored**, not an error: two
	 * states legitimately care about different subsets of what a DUT is
	 * sending, and a machine that failed on the first unrelated
	 * notification could not survive a real connection. */
	if (!select_frame(run->def, source, payload, payload_len, &fi)) {
		memset(out, 0, sizeof(*out));
		wait_step(st, out);
		return;
	}

	const struct eap_event_arm *arm = NULL;

	for (uint8_t i = 0; i < st->on_event_len; i++) {
		if (st->on_event[i].frame == fi) {
			arm = &st->on_event[i];
			break;
		}
	}
	if (arm == NULL) {
		memset(out, 0, sizeof(*out));
		wait_step(st, out);
		return;
	}

	const struct eap_frame *frame = &run->def->frames[fi];

	/* `remember`s apply **before** the guards, in declaration order, so
	 * `remember received = received + len(chunk.payload)` followed by
	 * `when received >= expect_total` compares the value *including* this
	 * frame -- which is what an author writing those two lines together
	 * means. */
	for (uint8_t i = 0; i < arm->remember_len; i++) {
		int64_t v;

		if (!eval_expr(&arm->remember[i].value, run, frame, payload, payload_len, &v)) {
			continue;
		}
		if (arm->remember[i].var < run->def->session_len) {
			run->session[arm->remember[i].var] = v;
		}
	}

	bool have_target = false;
	uint8_t target = 0;

	for (uint8_t i = 0; i < arm->when_len; i++) {
		if (eval_condition(&arm->when[i].cond, run, frame, payload, payload_len)) {
			target = arm->when[i].goto_state;
			have_target = true;
			break; /* first match wins */
		}
	}
	if (!have_target && arm->has_otherwise) {
		target = arm->otherwise;
		have_target = true;
	}

	if (!have_target) {
		/* No guard matched and no `otherwise`: the frame is consumed,
		 * its `remember`s stand, and the machine stays put **without
		 * re-entering** -- no `on_enter` re-send, and the timeout keeps
		 * running from the original entry. A pump loop wants the
		 * opposite and says so with `otherwise: goto <itself>`, which
		 * re-sends the flow-control ack and restarts the watchdog.
		 * Both behaviors are real and the author says which
		 * (embarch-decision-reversals.md row 66). */
		memset(out, 0, sizeof(*out));
		wait_step(st, out);
		return;
	}

	run->state = target;
	eap_run_enter(run, out);
}

void eap_run_on_event(struct eap_run *run, const struct eap_event *ev, struct eap_step *out)
{
	if (run->finished) {
		memset(out, 0, sizeof(*out));
		set_outcome(run, out, EAP_OUTCOME_PASS, NULL);
		return;
	}
	if (run->def->states[run->state].kind != EAP_STATE_ACTIVE) {
		on_entered(run, out);
		return;
	}
	if (ev->kind == EAP_EVENT_TIMEOUT) {
		on_timeout(run, out);
	} else {
		on_notify(run, ev->source, ev->payload, ev->payload_len, out);
	}
}

void eap_run_abandon(struct eap_run *run, struct eap_step *out)
{
	run->finished = true;
	memset(out, 0, sizeof(*out));
	set_outcome(run, out, EAP_OUTCOME_TIMED_OUT, NULL);
}
