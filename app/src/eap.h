/* The `.eap` protocol manifest wire types -- `embarch-study-designer` decisions 58-62, §4.9.
 *
 * Mirrors `embarch-study-designer/src/eap.rs` field for field, and is a
 * separate header for the reason that crate keeps `eap.rs` separate from
 * `eap_interp.rs`: three unrelated things need these types and none of them
 * needs the other two. serial_protocol.c decodes them off the wire,
 * eap_interp.c executes them, and ble_bridge carries one into a
 * `RunProtocol` action -- and ble_bridge.h deliberately knows nothing about
 * the link protocol (see its own note on why BLE_MAX_PAYLOAD_LEN is a
 * duplicate rather than an include).
 *
 * **Two deliberate departures from a mechanical transliteration**, both of
 * which are the point rather than incidental:
 *
 *  - *Names are dropped*, all of them but `eap_state.name`. Every
 *    cross-reference inside a resolved `ProtocolDef` is already an index --
 *    a `goto` is a states index, an `on_event` is a frames index, a `write`
 *    is a sources index -- so this firmware compares no strings, and the one
 *    string it keeps is the one `ProtocolOutcome.final_state` reports back.
 *    Same reasoning, and same ~2.5 KB, as `struct dbm_stream_tap` keeping 12
 *    bytes and discarding `name`/`encoding`. The decoder still *walks* every
 *    discarded name, because postcard carries no per-field length.
 *
 *  - *`struct eap_operand` is packed.* Its `literal` is an `int64_t` because
 *    the crate's `Operand::Literal` is an `i64`, and an operand is the
 *    most-multiplied thing in a manifest: two per condition, two per
 *    expression, one per write field, times arms, times states, times
 *    protocols, times the two `struct dev_bench_message` statics. Naturally
 *    aligned it is 16 bytes, six of them padding; packed it is 10, and that
 *    single attribute takes a two-protocol `StudyStart` from ~25 KB of SRAM
 *    to ~13 KB. The cost is byte-wise loads of `literal` on a target without
 *    misaligned access, which is nothing on a path that runs once per
 *    notification.
 */
#ifndef EMBARCH_DEV_BENCH_EAP_H_
#define EMBARCH_DEV_BENCH_EAP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- `.eap` protocol manifests (`embarch-study-designer` decisions 58-62,
 *      §4.9) ---------------------------------------------------------------
 *
 * **These bound a value this firmware executes, not one it walks past**, and
 * that is what makes them different from every other constant this firmware
 * mirrors.
 * A `ProtocolDef` is the guard-reachable half of an `.eap` manifest (that
 * doc's decision 59 split) and rides in `StudyStart` all the way here, so
 * every constant below costs real ESP32-C5 SRAM twice over: `struct
 * dbm_study_start` is the largest member of `struct dev_bench_message`'s
 * union, and main.c holds two of those (`tx_scratch` and the dispatch loop's
 * `msg`).
 *
 * Every count below is the **crate's own** value except
 * EAP_MAX_EVENT_ARMS_PER_STATE -- see that constant for why it is the one
 * worth a dev-bench-internal cap, and serial_protocol.h's own
 * DBM_MAX_STEPS_PER_STUDY for the precedent such a cap follows: rejected
 * outright and named, never silently truncated.
 *
 * How many protocols one study may carry, and how many bytes their span may
 * occupy on the wire, are bounds on the *message* rather than on the grammar
 * and live in serial_protocol.h with the rest of those.
 */
#define EAP_MAX_SOURCES_PER_PROTOCOL 6
#define EAP_MAX_FRAMES_PER_PROTOCOL 8
#define EAP_MAX_FRAME_FIELDS 8
#define EAP_MAX_FRAME_SPANS 4
#define EAP_MAX_SESSION_VARS 6
#define EAP_MAX_STATES_PER_PROTOCOL 12
/* limits::MAX_STATE_NAME_LEN -- the one `.eap` string this firmware keeps,
 * because `ProtocolOutcome.final_state` reports it back (that doc's decision
 * 62). Every other name in a manifest -- protocol, source, frame, field,
 * span, session variable -- is walked past and discarded, exactly as `struct
 * dbm_stream_tap` discards `name`/`encoding`: every cross-reference inside a
 * resolved `ProtocolDef` is already an index, so this firmware compares no
 * strings and needs none of them. */
#define EAP_MAX_STATE_NAME_LEN 24
/* **The one cap below the crate's own** (4 there). An event arm is the
 * heaviest thing a state holds -- two `remember`s and two guarded `goto`s,
 * each carrying operands -- and it is multiplied by
 * EAP_MAX_STATES_PER_PROTOCOL, by DBM_MAX_PROTOCOLS_PER_STUDY, and again by
 * the two `struct dev_bench_message` statics, so the crate's 4 costs ~9 KB of
 * SRAM more than 2 does. Both worked protocols in that doc's §4.9 use **one**
 * arm per state; two is double the largest real case, and a study declaring
 * more is refused by name at decode rather than quietly losing a transition
 * (which would be a state machine that runs and branches wrongly -- strictly
 * worse than one that does not run). Revisit this number, not the crate's own
 * MAX_EVENT_ARMS_PER_STATE, if a real manifest needs a third arm. */
#define EAP_MAX_EVENT_ARMS_PER_STATE 2
#define EAP_MAX_GUARDS_PER_ARM 2
#define EAP_MAX_REMEMBER_PER_ARM 2
#define EAP_MAX_WRITE_FIELDS 6
#define EAP_MAX_SELECT_MATCH_LEN 8
/* limits::MAX_FAIL_REASON_LEN. Duplicated from serial_protocol.h's
 * DBM_MAX_FAIL_REASON_LEN rather than included, the same way
 * BLE_MAX_PAYLOAD_LEN duplicates DBM_MAX_PAYLOAD_LEN: this header is one of
 * three that must not depend on the link protocol. Both mirror the one crate
 * constant, which is the definition. */
#define EAP_MAX_FAIL_REASON_LEN 64

/* `ScalarType` (embarch-study-designer/src/decoder.rs) -- the discriminants,
 * which is what crosses the wire inside a `ScalarRead`/`WriteField`. Float
 * widths are listed because the enum has them and a manifest could name one;
 * they are refused at parse time host-side, and refused again here rather
 * than trusted, since the expression set is integer-only. */
enum eap_scalar_type {
	EAP_SCALAR_U8 = 0,
	EAP_SCALAR_I8 = 1,
	EAP_SCALAR_U16LE = 2,
	EAP_SCALAR_U16BE = 3,
	EAP_SCALAR_I16LE = 4,
	EAP_SCALAR_I16BE = 5,
	EAP_SCALAR_U32LE = 6,
	EAP_SCALAR_U32BE = 7,
	EAP_SCALAR_I32LE = 8,
	EAP_SCALAR_I32BE = 9,
	EAP_SCALAR_U64LE = 10,
	EAP_SCALAR_U64BE = 11,
	EAP_SCALAR_I64LE = 12,
	EAP_SCALAR_I64BE = 13,
	EAP_SCALAR_F32LE = 14,
	EAP_SCALAR_F32BE = 15,
	EAP_SCALAR_F64LE = 16,
	EAP_SCALAR_F64BE = 17,
};

/* `Operand` (eap.rs) -- the four forms, and adding a fifth is a decision. */
enum eap_operand_kind {
	EAP_OP_LITERAL = 0,
	EAP_OP_FIELD = 1,
	EAP_OP_SESSION = 2,
	EAP_OP_SPAN_LEN = 3,
};

struct eap_operand {
	uint8_t kind;  /* enum eap_operand_kind */
	uint8_t index; /* field / session-variable / span index; unused for a literal */
	int64_t literal;
} __attribute__((packed));

/* The packing above is load-bearing for SRAM, not cosmetic -- assert it took
 * rather than trusting a compiler that might ignore the attribute. */
_Static_assert(sizeof(struct eap_operand) == 10, "eap_operand must stay packed");

/* `Expr` (eap.rs) -- one level, exactly one arithmetic operation, and `Add`
 * saturates. `b` is unused for a Term. */
enum eap_expr_kind {
	EAP_EXPR_TERM = 0,
	EAP_EXPR_ADD = 1,
};

struct eap_expr {
	uint8_t kind; /* enum eap_expr_kind */
	struct eap_operand a;
	struct eap_operand b;
};

/* `CompareOp` (eap.rs). Six comparisons and no boolean connectives: `a && b`
 * is two states, `!a` is swapping `when` and `otherwise`. */
enum eap_compare_op {
	EAP_CMP_EQ = 0,
	EAP_CMP_NE = 1,
	EAP_CMP_LT = 2,
	EAP_CMP_LE = 3,
	EAP_CMP_GT = 4,
	EAP_CMP_GE = 5,
};

struct eap_condition {
	struct eap_operand lhs;
	struct eap_operand rhs;
	uint8_t op; /* enum eap_compare_op */
};

/* `Remember` -- `remember <var> = <expr>`, applied **before** this arm's
 * guards are evaluated. */
struct eap_remember {
	struct eap_expr value;
	uint8_t var; /* session-variable index */
};

/* `GuardedGoto` -- `when <cond>: goto <state>`. First match wins. */
struct eap_guarded_goto {
	struct eap_condition cond;
	uint8_t goto_state;
};

/* `EventArm` -- `on_event <frame>: [remember …] [when …] [otherwise …]`.
 *
 * **`has_otherwise == false` is not the same as `otherwise` pointing at this
 * state, and getting that wrong is the one mistake this type exists to make
 * hard** (`embarch-study-designer` decision 60, and
 * embarch-decision-reversals.md row 66). With no `otherwise`, the frame is
 * consumed, its `remember`s stand, and the machine stays put *without
 * re-entering*: no `on_enter` re-send, and the timeout keeps counting from
 * the original entry. `otherwise: goto <this state>` re-enters -- re-sending
 * the flow-control ack and restarting the stall watchdog -- which is what a
 * pump loop needs and what the first draft of the crate's own fixture got
 * wrong (it consumed every chunk, acked none, and stalled). */
struct eap_event_arm {
	struct eap_remember remember[EAP_MAX_REMEMBER_PER_ARM];
	struct eap_guarded_goto when[EAP_MAX_GUARDS_PER_ARM];
	uint8_t frame; /* index into eap_protocol_def.frames */
	uint8_t remember_len;
	uint8_t when_len;
	uint8_t otherwise; /* state index, valid only when has_otherwise */
	bool has_otherwise;
};

/* `WriteField`/`WriteAction` (`embarch-study-designer` decision 61) -- a write's payload
 * is assembled from the same typed vocabulary a decode reads, so it can carry
 * a session variable or a field of the frame that triggered this event.
 * `ty` is a `ScalarType` discriminant (embarch-study-designer/src/decoder.rs),
 * shared with decision 52's `StructLayout` on purpose. */
struct eap_write_field {
	struct eap_operand value;
	uint8_t ty; /* ScalarType discriminant */
};

struct eap_write {
	struct eap_write_field fields[EAP_MAX_WRITE_FIELDS];
	uint8_t source; /* index into eap_protocol_def.sources */
	uint8_t fields_len;
	/* An acknowledged `Write Request` rather than a `Write Command`.
	 * **Never a transition trigger either way** -- a control-point write's
	 * response confirms only that the write was accepted, and the
	 * authoritative answer arrives later on a different characteristic
	 * (`embarch-study-designer` decision 60). This selects the ATT operation, nothing
	 * more. */
	bool with_response;
};

/* `TimeoutArm` -- `on_timeout <ms> [retry <n>]: goto <state>`.
 *
 * `retry` re-runs `on_enter` and waits again; it does **not** mean "wait
 * longer". `retry: 0` takes `goto` on the first expiry, which is what a stall
 * watchdog wants. Measured from the state's entry, or from the last retry. */
struct eap_timeout_arm {
	uint32_t after_ms;
	uint8_t retry;
	uint8_t goto_state;
};

/* `StateKind`/`TerminalOutcome` (eap.rs). `Outcome::TimedOut` is deliberately
 * absent: a manifest cannot declare it, only a run can produce it. */
enum eap_state_kind {
	EAP_STATE_ACTIVE = 0,
	EAP_STATE_TERMINAL = 1,
};

enum eap_terminal_outcome {
	EAP_TERMINAL_PASS = 0,
	EAP_TERMINAL_FAIL = 1,
};

struct eap_state {
	/* The one `.eap` name this firmware keeps -- see the section comment
	 * above. NUL-terminated. */
	char name[EAP_MAX_STATE_NAME_LEN + 1];
	uint8_t kind;     /* enum eap_state_kind */
	uint8_t terminal; /* enum eap_terminal_outcome, valid when kind == TERMINAL */
	/* Everything below is valid only for an ACTIVE state. */
	struct eap_write on_enter;
	struct eap_event_arm on_event[EAP_MAX_EVENT_ARMS_PER_STATE];
	struct eap_timeout_arm on_timeout;
	uint8_t on_event_len;
	bool has_on_enter;
	bool has_on_timeout;
};

/* `ScalarRead` and `SpanRead`, minus their names. A span's *bytes* never
 * reach an expression -- only its `len()` does, which is what a
 * flow-controlled pump loop counts. `has_len == false` means "the rest of the
 * payload". */
struct eap_scalar_read {
	uint16_t offset;
	uint8_t ty; /* ScalarType discriminant */
};

struct eap_span_read {
	uint16_t offset;
	uint16_t len;
	bool has_len;
};

/* `FrameDef` -- a frame shape the machine can dispatch on.
 *
 * `select_len` is `eq.len()` by construction; the crate does not carry a
 * separate `len` for exactly the reason it would admit a manifest where the
 * two disagree. A payload too short to contain the match **does not match**:
 * a truncated notification and a different format are different facts. */
struct eap_frame {
	uint8_t select_eq[EAP_MAX_SELECT_MATCH_LEN];
	struct eap_scalar_read fields[EAP_MAX_FRAME_FIELDS];
	struct eap_span_read spans[EAP_MAX_FRAME_SPANS];
	uint16_t select_offset;
	uint8_t source; /* index into eap_protocol_def.sources */
	uint8_t select_len;
	uint8_t fields_len;
	uint8_t spans_len;
	bool has_select;
};

/* `ProtocolSource` minus its alias -- the characteristic this protocol writes
 * to or reads frames from. Raw big-endian UUID bytes, this header's
 * convention throughout. Both are kept, unlike `struct dbm_stream_tap`'s
 * routing-only characteristic: a `RunProtocol` step has to *discover* within
 * a service before it can subscribe, exactly as DataExchange does. */
struct eap_source {
	uint8_t service_uuid[16];
	uint8_t characteristic_uuid[16];
};

/* One `protocol <name> { … }` block, resolved and ready to execute. */
struct eap_protocol_def {
	struct eap_state states[EAP_MAX_STATES_PER_PROTOCOL];
	struct eap_frame frames[EAP_MAX_FRAMES_PER_PROTOCOL];
	struct eap_source sources[EAP_MAX_SOURCES_PER_PROTOCOL];
	/* Session variables are integers only (`embarch-study-designer` decision 60): the
	 * draft's byte-span accumulator is gone with `++`, because on this
	 * bench there is nowhere to put those bytes and nowhere they are
	 * needed -- the chunks stream out on their own tap as they arrive, so
	 * the machine only has to count them. */
	int64_t session_initial[EAP_MAX_SESSION_VARS];
	uint8_t sources_len;
	uint8_t frames_len;
	uint8_t session_len;
	uint8_t states_len;
};

#endif /* EMBARCH_DEV_BENCH_EAP_H_ */
