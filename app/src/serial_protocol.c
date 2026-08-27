#include "serial_protocol.h"

#include <string.h>

/* ---- postcard-compatible primitives -------------------------------------
 *
 * postcard encodes unsigned integers (including enum discriminants, which
 * serde treats as u32) as ULEB128 varints, `bool` as a single 0/1 byte,
 * `f32`/`f64` as raw little-endian bytes (no varint), and `&str`/`String` as
 * a ULEB128 byte-length prefix followed by raw UTF-8 bytes. This assumes a
 * little-endian target for float encoding, true for both this firmware's
 * targets (Cortex-M33 in Zephyr's default configuration, and native_sim's
 * host x86/x86_64).
 */

static size_t pc_write_varint(uint64_t value, uint8_t *out)
{
	size_t n = 0;

	while (value >= 0x80) {
		out[n++] = (uint8_t)((value & 0x7F) | 0x80);
		value >>= 7;
	}
	out[n++] = (uint8_t)(value & 0x7F);
	return n;
}

static int pc_read_varint(const uint8_t *in, size_t in_len, size_t *pos, uint64_t *out)
{
	uint64_t result = 0;
	int shift = 0;

	while (1) {
		if (*pos >= in_len) {
			return -1;
		}
		uint8_t byte = in[(*pos)++];

		result |= (uint64_t)(byte & 0x7F) << shift;
		if (!(byte & 0x80)) {
			break;
		}
		shift += 7;
		if (shift >= 64) {
			return -1;
		}
	}
	*out = result;
	return 0;
}

static int pc_write_bytes(const uint8_t *bytes, size_t len, uint8_t *out, size_t out_cap,
			   size_t *pos)
{
	uint8_t len_buf[10];
	size_t len_n = pc_write_varint((uint64_t)len, len_buf);

	if (*pos + len_n + len > out_cap) {
		return -1;
	}
	memcpy(out + *pos, len_buf, len_n);
	*pos += len_n;
	memcpy(out + *pos, bytes, len);
	*pos += len;
	return 0;
}

/* Reads a length-prefixed string into `out` (capacity `out_cap`, including
 * room for the NUL this function appends). Rejects a wire string that
 * wouldn't fit rather than truncating it silently. */
static int pc_read_str(const uint8_t *in, size_t in_len, size_t *pos, char *out, size_t out_cap)
{
	uint64_t len;

	if (pc_read_varint(in, in_len, pos, &len) != 0) {
		return -1;
	}
	if (len >= out_cap || *pos + len > in_len) {
		return -1;
	}
	memcpy(out, in + *pos, (size_t)len);
	out[len] = '\0';
	*pos += (size_t)len;
	return 0;
}

/* `pc_write_f32`/`pc_read_f32` were here, for the retired `Sample`-carrying
 * stream messages (embarch-study-designer schema v8, that doc's §3 decision
 * 39). Removed with them: the wire now carries arrival-stamped bytes, and
 * this firmware assigns no meaning -- and no numeric type -- to a stream
 * payload at all.
 */
static int pc_skip_len_prefixed(const uint8_t *in, size_t in_len, size_t *pos, size_t elem_size)
{
	uint64_t len;

	if (pc_read_varint(in, in_len, pos, &len) != 0) {
		return -1;
	}
	size_t n = (size_t)len * elem_size;

	if (*pos + n > in_len) {
		return -1;
	}
	*pos += n;
	return 0;
}

/* Reads one postcard-encoded `StreamTap` (embarch-study-designer
 * src/streams.rs, design.md §4.8), advancing *pos past it and filling `out`
 * with the part this node acts on. Returns 0 on success, -1 on a malformed
 * or unrecognized shape.
 *
 * Every field is walked, because postcard carries no per-field length and
 * `scope` cannot be reached without parsing `name`, `source` and `encoding`
 * first. Only `id`, the source tag and the scope are **kept** -- see `struct
 * dbm_stream_tap` for why an encoding stored here would be this firmware
 * holding the one kind of knowledge decision 39 took away from it.
 *
 * An unknown variant tag is a hard error rather than a skip: postcard
 * carries no field names and no per-variant length, so a tag this decoder
 * predates leaves everything after it unwalkable. The Hello/HelloAck
 * schema-version handshake is what makes that acceptable -- a peer that
 * would send one has already been refused.
 */
static int pc_read_stream_tap(const uint8_t *raw, size_t raw_len, size_t *pos,
			      struct dbm_stream_tap *out)
{
	uint64_t tag;
	uint64_t scratch;

	/* id: u8 -- a raw byte, not a varint (same as StreamOpen's own id). */
	if (*pos >= raw_len) {
		return -1;
	}
	out->id = raw[*pos];
	(*pos)++;
	/* Cleared rather than left as whatever the previous study's tap at
	 * this index held: a stale UUID here would route a notification into a
	 * tap that isn't a GattNotify one at all. */
	memset(out->characteristic_uuid, 0, sizeof(out->characteristic_uuid));

	/* name: heapless::String -- length-prefixed bytes. */
	if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
		return -1;
	}

	/* source: StreamSource */
	if (pc_read_varint(raw, raw_len, pos, &tag) != 0) {
		return -1;
	}
	out->source_tag = (uint8_t)tag;
	switch (tag) {
	case 0: /* GattNotify { service_uuid, characteristic_uuid } */
		if (*pos + 32 > raw_len) {
			return -1;
		}
		/* The service UUID is walked past; the characteristic is kept,
		 * because this node routes notifications to this tap's id and
		 * a notification identifies itself by characteristic (design.md
		 * §3 decision 55). Addressing, not meaning -- see `struct
		 * dbm_stream_tap`. */
		memcpy(out->characteristic_uuid, raw + *pos + 16, 16);
		*pos += 32;
		break;
	case 1: /* PowerFrontEnd { sample_hz } */
		if (pc_read_varint(raw, raw_len, pos, &scratch) != 0) {
			return -1;
		}
		break;
	case 2: /* GattTranscript */
	case 3: /* DevBenchLog */
		break;
	case 4: /* Signal { name } */
		if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
			return -1;
		}
		break;
	default:
		return -1;
	}

	/* encoding: StreamEncoding */
	if (pc_read_varint(raw, raw_len, pos, &tag) != 0) {
		return -1;
	}
	switch (tag) {
	case 0: /* Raw */
	case 1: /* Text */
	case 3: /* GattTranscript */
	case 4: /* OutpostTrace -- a unit variant as of schema v11 */
		break;
	case 2: /* Samples { layout, unit, channel_id } */
		if (pc_read_varint(raw, raw_len, pos, &scratch) != 0) {
			return -1; /* layout: SampleLayout */
		}
		if (pc_read_varint(raw, raw_len, pos, &scratch) != 0) {
			return -1; /* unit: Unit */
		}
		if (*pos >= raw_len) {
			return -1;
		}
		(*pos)++; /* channel_id: u8 */
		break;
	case 5: /* Struct { decoder } -- schema v14 */
		/* Walked past, never kept: `decoder` indexes `Study.decoders`,
		 * a host-only field this node never receives, and what a
		 * payload means is exactly the knowledge decision 39 took away
		 * from this firmware. One raw u8, not a varint -- same shape as
		 * `Samples`' own channel_id. */
		if (*pos >= raw_len) {
			return -1;
		}
		(*pos)++;
		break;
	default:
		return -1;
	}

	/* scope: StreamScope */
	if (pc_read_varint(raw, raw_len, pos, &tag) != 0) {
		return -1;
	}
	out->scope_tag = (uint8_t)tag;
	out->scope_from = 0;
	out->scope_to = 0;
	switch (tag) {
	case 0: /* WholeStudy */
		break;
	case 1: /* Steps { from, to } */
		if (pc_read_varint(raw, raw_len, pos, &scratch) != 0) {
			return -1;
		}
		out->scope_from = (uint32_t)scratch;
		if (pc_read_varint(raw, raw_len, pos, &scratch) != 0) {
			return -1;
		}
		out->scope_to = (uint32_t)scratch;
		break;
	default:
		return -1;
	}

	return 0;
}

/* ---- `.eap` protocol manifests (embarch-study-designer/design.md §3
 *      decisions 58-62, §4.9) ----------------------------------------------
 *
 * The reference these have to agree with is `embarch-study-designer/src/eap.rs`
 * (the wire types) and `src/eap_interp.rs` (the semantics). Nothing here
 * infers a shape: every read below is the postcard encoding of a field that
 * module declares, in the order it declares it, and the pinned wire vector in
 * app/tests/serial_protocol is what proves the two agree rather than each
 * agreeing with itself.
 *
 * **Names are walked and discarded**, all of them but `StateDef.name`. That
 * is not an optimization applied afterwards -- it is why `struct
 * eap_protocol_def` has no name fields at all (see serial_protocol.h). The
 * walk is unavoidable regardless: postcard carries no per-field length, so a
 * name cannot be skipped without being read.
 */

/* postcard encodes a signed integer as a zigzag varint. `Operand::Literal` is
 * an `i64` and `SessionVarDef.initial` is an `i64`; nothing else in a
 * `ProtocolDef` is signed. */
static int pc_read_zigzag(const uint8_t *in, size_t in_len, size_t *pos, int64_t *out)
{
	uint64_t raw;

	if (pc_read_varint(in, in_len, pos, &raw) != 0) {
		return -1;
	}
	*out = (int64_t)(raw >> 1) ^ -(int64_t)(raw & 1);
	return 0;
}

/* `Operand` (eap.rs) -- a varint discriminant, then one payload. `Field`,
 * `Session` and `SpanLen` each carry a `u8`, which postcard writes as a raw
 * byte rather than a varint. */
static int pc_read_eap_operand(const uint8_t *raw, size_t raw_len, size_t *pos,
			       struct eap_operand *out)
{
	uint64_t kind;

	if (pc_read_varint(raw, raw_len, pos, &kind) != 0) {
		return -1;
	}
	out->kind = (uint8_t)kind;
	out->index = 0;
	out->literal = 0;
	switch (kind) {
	case EAP_OP_LITERAL: {
		/* Through a local rather than straight into `out->literal`:
		 * `struct eap_operand` is packed (see serial_protocol.h for
		 * why that is worth ~12 KB), so the member's address is
		 * potentially unaligned and taking it is a diagnostic on every
		 * compiler this builds under. Assigning through the struct is
		 * what makes the compiler emit the byte-wise store. */
		int64_t literal;

		if (pc_read_zigzag(raw, raw_len, pos, &literal) != 0) {
			return -1;
		}
		out->literal = literal;
		return 0;
	}
	case EAP_OP_FIELD:
	case EAP_OP_SESSION:
	case EAP_OP_SPAN_LEN:
		if (*pos >= raw_len) {
			return -1;
		}
		out->index = raw[(*pos)++];
		return 0;
	default:
		/* A fifth operand form is a decision with a firmware reflash
		 * attached (design.md §3 decision 60), so a tag this build has
		 * no name for cannot be walked past and is a hard error. */
		return -1;
	}
}

/* `Expr` -- `Term(Operand)` or `Add(Operand, Operand)`. `Add` saturates at
 * evaluation time, not here. */
static int pc_read_eap_expr(const uint8_t *raw, size_t raw_len, size_t *pos,
			    struct eap_expr *out)
{
	uint64_t kind;

	if (pc_read_varint(raw, raw_len, pos, &kind) != 0) {
		return -1;
	}
	out->kind = (uint8_t)kind;
	memset(&out->b, 0, sizeof(out->b));
	if (pc_read_eap_operand(raw, raw_len, pos, &out->a) != 0) {
		return -1;
	}
	switch (kind) {
	case EAP_EXPR_TERM:
		return 0;
	case EAP_EXPR_ADD:
		return pc_read_eap_operand(raw, raw_len, pos, &out->b);
	default:
		return -1;
	}
}

/* `Condition { lhs, op, rhs }` -- a struct, so the fields are in declaration
 * order with no tag of their own. */
static int pc_read_eap_condition(const uint8_t *raw, size_t raw_len, size_t *pos,
				 struct eap_condition *out)
{
	uint64_t op;

	if (pc_read_eap_operand(raw, raw_len, pos, &out->lhs) != 0) {
		return -1;
	}
	if (pc_read_varint(raw, raw_len, pos, &op) != 0) {
		return -1;
	}
	if (op > EAP_CMP_GE) {
		return -1;
	}
	out->op = (uint8_t)op;
	return pc_read_eap_operand(raw, raw_len, pos, &out->rhs);
}

/* `ActiveState.on_enter: Option<WriteAction>` (design.md §3 decision 61). */
static int pc_read_eap_write(const uint8_t *raw, size_t raw_len, size_t *pos,
			     struct eap_write *out)
{
	uint64_t len;

	if (*pos >= raw_len) {
		return -1;
	}
	out->source = raw[(*pos)++];
	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_WRITE_FIELDS) {
		return -1;
	}
	out->fields_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		uint64_t ty;

		if (pc_read_varint(raw, raw_len, pos, &ty) != 0) {
			return -1;
		}
		out->fields[i].ty = (uint8_t)ty;
		if (pc_read_eap_operand(raw, raw_len, pos, &out->fields[i].value) != 0) {
			return -1;
		}
	}
	if (*pos >= raw_len) {
		return -1;
	}
	out->with_response = raw[(*pos)++] != 0;
	return 0;
}

/* `EventArm { frame, remember, when, otherwise }`.
 *
 * `otherwise: None` and `otherwise: Some(<this state>)` are genuinely
 * different behaviors -- see `struct eap_event_arm`'s own comment and
 * embarch-decision-reversals.md row 66 -- so the Option byte is kept rather
 * than folded into a self-transition default. */
static int pc_read_eap_event_arm(const uint8_t *raw, size_t raw_len, size_t *pos,
				 struct eap_event_arm *out)
{
	uint64_t len;

	memset(out, 0, sizeof(*out));
	if (*pos >= raw_len) {
		return -1;
	}
	out->frame = raw[(*pos)++];

	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_REMEMBER_PER_ARM) {
		return -1;
	}
	out->remember_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (*pos >= raw_len) {
			return -1;
		}
		out->remember[i].var = raw[(*pos)++];
		if (pc_read_eap_expr(raw, raw_len, pos, &out->remember[i].value) != 0) {
			return -1;
		}
	}

	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_GUARDS_PER_ARM) {
		return -1;
	}
	out->when_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (pc_read_eap_condition(raw, raw_len, pos, &out->when[i].cond) != 0) {
			return -1;
		}
		if (*pos >= raw_len) {
			return -1;
		}
		out->when[i].goto_state = raw[(*pos)++];
	}

	if (*pos >= raw_len) {
		return -1;
	}
	out->has_otherwise = raw[(*pos)++] != 0;
	if (out->has_otherwise) {
		if (*pos >= raw_len) {
			return -1;
		}
		out->otherwise = raw[(*pos)++];
	}
	return 0;
}

/* `StateDef { name, kind }`, where `kind` is `Active(ActiveState)` or
 * `Terminal(TerminalOutcome)`. The name is the one string kept. */
static int pc_read_eap_state(const uint8_t *raw, size_t raw_len, size_t *pos,
			     struct eap_state *out)
{
	uint64_t kind;
	uint64_t len;

	memset(out, 0, sizeof(*out));
	if (pc_read_str(raw, raw_len, pos, out->name, sizeof(out->name)) != 0) {
		return -1;
	}
	if (pc_read_varint(raw, raw_len, pos, &kind) != 0) {
		return -1;
	}
	out->kind = (uint8_t)kind;
	switch (kind) {
	case EAP_STATE_TERMINAL: {
		uint64_t outcome;

		if (pc_read_varint(raw, raw_len, pos, &outcome) != 0) {
			return -1;
		}
		if (outcome > EAP_TERMINAL_FAIL) {
			/* `Outcome::TimedOut` is not declarable by a manifest
			 * (design.md §3 decision 62); only a run produces it.
			 * A third discriminant here is drift, not a new
			 * outcome. */
			return -1;
		}
		out->terminal = (uint8_t)outcome;
		return 0;
	}
	case EAP_STATE_ACTIVE:
		break;
	default:
		return -1;
	}

	if (*pos >= raw_len) {
		return -1;
	}
	out->has_on_enter = raw[(*pos)++] != 0;
	if (out->has_on_enter && pc_read_eap_write(raw, raw_len, pos, &out->on_enter) != 0) {
		return -1;
	}

	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_EVENT_ARMS_PER_STATE) {
		/* A dev-bench-internal cap below the crate's own -- refused by
		 * name rather than truncated, because a state machine missing a
		 * transition runs and branches wrongly, which is strictly worse
		 * than one that does not run. See EAP_MAX_EVENT_ARMS_PER_STATE. */
		return -1;
	}
	out->on_event_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (pc_read_eap_event_arm(raw, raw_len, pos, &out->on_event[i]) != 0) {
			return -1;
		}
	}

	if (*pos >= raw_len) {
		return -1;
	}
	out->has_on_timeout = raw[(*pos)++] != 0;
	if (out->has_on_timeout) {
		uint64_t after_ms;

		if (pc_read_varint(raw, raw_len, pos, &after_ms) != 0) {
			return -1;
		}
		out->on_timeout.after_ms = (uint32_t)after_ms;
		if (*pos + 2 > raw_len) {
			return -1;
		}
		out->on_timeout.retry = raw[(*pos)++];
		out->on_timeout.goto_state = raw[(*pos)++];
	}
	return 0;
}

/* `FrameDef { name, source, select_if, fields, spans }`. */
static int pc_read_eap_frame(const uint8_t *raw, size_t raw_len, size_t *pos,
			     struct eap_frame *out)
{
	uint64_t len;

	memset(out, 0, sizeof(*out));
	if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
		return -1; /* name -- walked, not kept */
	}
	if (*pos >= raw_len) {
		return -1;
	}
	out->source = raw[(*pos)++];

	if (*pos >= raw_len) {
		return -1;
	}
	out->has_select = raw[(*pos)++] != 0;
	if (out->has_select) {
		uint64_t offset;

		if (pc_read_varint(raw, raw_len, pos, &offset) != 0) {
			return -1;
		}
		out->select_offset = (uint16_t)offset;
		if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
			return -1;
		}
		if (len > EAP_MAX_SELECT_MATCH_LEN || *pos + len > raw_len) {
			return -1;
		}
		memcpy(out->select_eq, raw + *pos, (size_t)len);
		*pos += (size_t)len;
		out->select_len = (uint8_t)len;
	}

	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_FRAME_FIELDS) {
		return -1;
	}
	out->fields_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		uint64_t offset;
		uint64_t ty;

		if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
			return -1; /* field name -- walked, not kept */
		}
		if (pc_read_varint(raw, raw_len, pos, &offset) != 0) {
			return -1;
		}
		if (pc_read_varint(raw, raw_len, pos, &ty) != 0) {
			return -1;
		}
		out->fields[i].offset = (uint16_t)offset;
		out->fields[i].ty = (uint8_t)ty;
	}

	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_FRAME_SPANS) {
		return -1;
	}
	out->spans_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		uint64_t offset;

		if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
			return -1; /* span name -- walked, not kept */
		}
		if (pc_read_varint(raw, raw_len, pos, &offset) != 0) {
			return -1;
		}
		out->spans[i].offset = (uint16_t)offset;
		if (*pos >= raw_len) {
			return -1;
		}
		out->spans[i].has_len = raw[(*pos)++] != 0;
		if (out->spans[i].has_len) {
			uint64_t span_len;

			if (pc_read_varint(raw, raw_len, pos, &span_len) != 0) {
				return -1;
			}
			out->spans[i].len = (uint16_t)span_len;
		}
	}
	return 0;
}

/* One `ProtocolDef` (eap.rs), advancing *pos past it and filling `out` with
 * the part this node executes. Returns 0 on success, -1 on a malformed shape
 * or one past a dev-bench-internal cap.
 *
 * Every count checked here is checked because the value behind it reaches a
 * C array subscript. `validate_protocol` and Core's pre-flight already
 * range-check the *indices* inside a protocol host-side (design.md §3
 * decision 18's rule), and this is the other half: the *capacities* are this
 * firmware's own, so this is the only place that can refuse them.
 */
static int pc_read_protocol_def(const uint8_t *raw, size_t raw_len, size_t *pos,
				struct eap_protocol_def *out)
{
	uint64_t len;

	memset(out, 0, sizeof(*out));
	if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
		return -1; /* protocol name -- walked, not kept */
	}

	/* sources */
	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_SOURCES_PER_PROTOCOL) {
		return -1;
	}
	out->sources_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
			return -1; /* alias -- walked, not kept */
		}
		if (*pos + 32 > raw_len) {
			return -1;
		}
		memcpy(out->sources[i].service_uuid, raw + *pos, 16);
		*pos += 16;
		memcpy(out->sources[i].characteristic_uuid, raw + *pos, 16);
		*pos += 16;
	}

	/* frames */
	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_FRAMES_PER_PROTOCOL) {
		return -1;
	}
	out->frames_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (pc_read_eap_frame(raw, raw_len, pos, &out->frames[i]) != 0) {
			return -1;
		}
	}

	/* session -- integers only (design.md §3 decision 60) */
	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_SESSION_VARS) {
		return -1;
	}
	out->session_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (pc_skip_len_prefixed(raw, raw_len, pos, 1) != 0) {
			return -1; /* variable name -- walked, not kept */
		}
		if (pc_read_zigzag(raw, raw_len, pos, &out->session_initial[i]) != 0) {
			return -1;
		}
	}

	/* states */
	if (pc_read_varint(raw, raw_len, pos, &len) != 0) {
		return -1;
	}
	if (len > EAP_MAX_STATES_PER_PROTOCOL) {
		return -1;
	}
	out->states_len = (uint8_t)len;
	for (uint32_t i = 0; i < (uint32_t)len; i++) {
		if (pc_read_eap_state(raw, raw_len, pos, &out->states[i]) != 0) {
			return -1;
		}
	}
	return 0;
}

/* CRC-32 (ISO-HDLC / the common "CRC-32" used by zip/gzip/PNG/Ethernet;
 * poly 0xEDB88320 reflected, init/xorout 0xFFFFFFFF) -- matches the `crc`
 * crate's `CRC_32_ISO_HDLC` embarch-study-designer's `steps_crc()` (src/crc.rs)
 * uses. Bitwise, not table-driven: this only ever runs once per received
 * `StudyStart`, not a hot path worth the table's static footprint. */
static uint32_t dbm_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			uint32_t mask = -(crc & 1u);

			crc = (crc >> 1) ^ (0xEDB88320u & mask);
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

/* ---- COBS -----------------------------------------------------------------
 *
 * Standard Consistent Overhead Byte Stuffing (embarch-study-designer/design.md
 * §3 decision 10). `cobs_encode` does not append the trailing 0x00 frame
 * delimiter itself — `dbm_encode_frame` does, once, after calling this.
 */

static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < length) {
		if (input[read_index] == 0) {
			output[code_index] = code;
			code = 1;
			code_index = write_index++;
			read_index++;
		} else {
			output[write_index++] = input[read_index++];
			code++;
			if (code == 0xFF) {
				output[code_index] = code;
				code = 1;
				code_index = write_index++;
			}
		}
	}
	output[code_index] = code;
	return write_index;
}

/* Returns the decoded length, or 0 on a malformed frame or an output that
 * wouldn't fit in `output_cap` -- bounds-checked on every write, since `input`
 * is untrusted (bytes off a serial link, possibly corrupted or crafted) and
 * must never be able to overflow the fixed-size decode buffer regardless of
 * what `length` claims. */
static size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t output_cap)
{
	size_t read_index = 0;
	size_t write_index = 0;

	while (read_index < length) {
		uint8_t code = input[read_index];

		if (code == 0 || (read_index + code > length && code != 1)) {
			return 0;
		}
		read_index++;
		for (uint8_t i = 1; i < code; i++) {
			if (write_index >= output_cap) {
				return 0;
			}
			output[write_index++] = input[read_index++];
		}
		if (code != 0xFF && read_index != length) {
			if (write_index >= output_cap) {
				return 0;
			}
			output[write_index++] = 0;
		}
	}
	return write_index;
}

/* ---- DevBenchMessage encode/decode ---------------------------------------- */

/* One `GattTranscriptEntry` (embarch-study-designer src/gatt.rs, §4.3b) as
 * bare postcard bytes, appended at `*pos`. Mirrors that type's field order
 * exactly: the two Option<Uuid>s are postcard's 0/1 discriminant followed,
 * when present, by the UUID's 16 raw bytes -- the same unprefixed
 * fixed-array encoding DataExchange's own UUIDs already use.
 *
 * At schema v8 these bytes are a stream record's payload rather than a
 * message body of their own (design.md §3 decision 39), which is why this is
 * a function instead of an inlined `encode_body` case. */
static int encode_transcript_entry_at(const struct dbm_gatt_transcript_entry *e, uint8_t *out,
				       size_t out_cap, size_t *pos)
{
	uint8_t varint_buf[10];

#define WRITE_VARINT(value)                                                                       \
	do {                                                                                       \
		size_t n = pc_write_varint((uint64_t)(value), varint_buf);                        \
		if (*pos + n > out_cap) {                                                         \
			return -1;                                                                \
		}                                                                                  \
		memcpy(out + *pos, varint_buf, n);                                                \
		*pos += n;                                                                        \
	} while (0)

	WRITE_VARINT(e->rx_utc_ms);
	WRITE_VARINT(e->direction);
	WRITE_VARINT(e->kind);

	if (*pos + 1 > out_cap) {
		return -1;
	}
	out[(*pos)++] = e->has_service_uuid ? 1 : 0;
	if (e->has_service_uuid) {
		if (*pos + 16 > out_cap) {
			return -1;
		}
		memcpy(out + *pos, e->service_uuid, 16);
		*pos += 16;
	}

	if (*pos + 1 > out_cap) {
		return -1;
	}
	out[(*pos)++] = e->has_characteristic_uuid ? 1 : 0;
	if (e->has_characteristic_uuid) {
		if (*pos + 16 > out_cap) {
			return -1;
		}
		memcpy(out + *pos, e->characteristic_uuid, 16);
		*pos += 16;
	}

	if (*pos + 1 > out_cap) {
		return -1;
	}
	out[(*pos)++] = e->att_status; /* u8: raw byte, not varint */

	return pc_write_bytes(e->payload, e->payload_len, out, out_cap, pos);

#undef WRITE_VARINT
}

int dbm_encode_transcript_entry(const struct dbm_gatt_transcript_entry *entry, uint8_t *out,
				 size_t out_cap)
{
	size_t pos = 0;

	if (encode_transcript_entry_at(entry, out, out_cap, &pos) != 0) {
		return -1;
	}
	return (int)pos;
}

static int encode_body(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap,
			size_t *pos)
{
	uint8_t varint_buf[10];

#define WRITE_VARINT(value)                                                                       \
	do {                                                                                       \
		size_t n = pc_write_varint((uint64_t)(value), varint_buf);                        \
		if (*pos + n > out_cap) {                                                         \
			return -1;                                                                \
		}                                                                                  \
		memcpy(out + *pos, varint_buf, n);                                                \
		*pos += n;                                                                        \
	} while (0)

	WRITE_VARINT(msg->tag);

	switch (msg->tag) {
	case DBM_TAG_HELLO:
		WRITE_VARINT(msg->hello.schema_version);
		WRITE_VARINT(msg->hello.host_utc_ms);
		return 0;
	case DBM_TAG_HELLO_ACK:
		WRITE_VARINT(msg->hello_ack.schema_version);
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->hello_ack.compatible ? 1 : 0;
		if (pc_write_bytes((const uint8_t *)msg->hello_ack.firmware_version,
				   strlen(msg->hello_ack.firmware_version), out, out_cap,
				   pos) != 0) {
			return -1;
		}
		/* schema v10 (embarch-study-designer/design.md §3 decision 47).
		 * An empty hardware_id still writes its length prefix -- an
		 * absent field and a zero-length one are different bytes, and
		 * only the latter leaves the frame walkable. */
		return pc_write_bytes((const uint8_t *)msg->hello_ack.hardware_id,
				       strlen(msg->hello_ack.hardware_id), out, out_cap, pos);
	case DBM_TAG_STREAM_OPEN:
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->stream_open.id; /* u8: raw byte, not varint */
		return 0;
	case DBM_TAG_STREAM_CHUNK_BATCH: {
		/* embarch-study-designer schema v8, that doc's §3 decision 39.
		 * Arrival-stamped bytes, never decoded values -- this firmware
		 * assigns no meaning to a record's payload at all; the tap's
		 * declared `StreamEncoding` does that, host-side. */
		const struct dbm_stream_chunk_batch *batch = &msg->stream_chunk_batch;

		if (batch->records_len > DBM_MAX_STREAM_RECORDS_PER_BATCH) {
			return -1; /* would read past records[]'s own bound */
		}
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = batch->id; /* u8: raw byte, not varint */
		WRITE_VARINT(batch->records_len);
		for (uint32_t i = 0; i < batch->records_len; i++) {
			const struct dbm_stream_record *rec = &batch->records[i];

			if (rec->bytes_len > DBM_MAX_STREAM_CHUNK_BYTES) {
				return -1;
			}
			WRITE_VARINT(rec->rx_utc_ms);
			if (pc_write_bytes(rec->bytes, rec->bytes_len, out, out_cap, pos) != 0) {
				return -1;
			}
		}
		return 0;
	}
	case DBM_TAG_STREAM_CLOSE:
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->stream_close.id; /* u8: raw byte, not varint */
		WRITE_VARINT(msg->stream_close.dropped);
		return 0;
	case DBM_TAG_LOG_LINE:
		return pc_write_bytes((const uint8_t *)msg->log_line.text,
				       strlen(msg->log_line.text), out, out_cap, pos);
	case DBM_TAG_STUDY_START:
		/* dev-bench never sends StudyStart in practice (Core is the only
		 * sender) -- this encode path exists for this file's own
		 * round-trip tests. `service_uuids`/`power_sample` aren't stored
		 * on `struct dbm_step` (decision 21's scope), so they're always
		 * encoded empty/None here, for every action kind. */
		if (msg->study_start.steps_len > DBM_MAX_STEPS_PER_STUDY) {
			return -1; /* would read past struct dbm_study_start.steps[]'s bound */
		}
		WRITE_VARINT(msg->study_start.steps_len);
		for (uint32_t i = 0; i < msg->study_start.steps_len; i++) {
			const struct dbm_step *step = &msg->study_start.steps[i];

			if (pc_write_bytes((const uint8_t *)step->name, strlen(step->name), out,
					    out_cap, pos) != 0) {
				return -1;
			}
			WRITE_VARINT(step->action_tag);

			switch (step->action_tag) {
			case DBM_ACTION_BLE_ADVERTISE:
				if (*pos + 1 > out_cap) {
					return -1;
				}
				out[(*pos)++] = step->action.advertise.has_local_name ? 1 : 0;
				if (step->action.advertise.has_local_name &&
				    pc_write_bytes((const uint8_t *)step->action.advertise.local_name,
						    strlen(step->action.advertise.local_name), out,
						    out_cap, pos) != 0) {
					return -1;
				}
				WRITE_VARINT(0); /* service_uuids: Vec<Uuid,4>, always empty */
				WRITE_VARINT(step->action.advertise.adv_interval_ms);
				break;

			case DBM_ACTION_BLE_CONNECT:
				WRITE_VARINT(step->action.connect.role);
				if (*pos + 1 > out_cap) {
					return -1;
				}
				out[(*pos)++] = step->action.connect.has_target_address ? 1 : 0;
				if (step->action.connect.has_target_address) {
					if (*pos + 6 > out_cap) {
						return -1;
					}
					memcpy(out + *pos, step->action.connect.target_address, 6);
					*pos += 6;
					WRITE_VARINT(step->action.connect.target_address_kind);
				}
				/* target_name: Option<String> -- schema v7's trailing
				 * field on this variant (design.md §3 decision 43). */
				if (*pos + 1 > out_cap) {
					return -1;
				}
				out[(*pos)++] = step->action.connect.has_target_name ? 1 : 0;
				if (step->action.connect.has_target_name &&
				    pc_write_bytes((const uint8_t *)step->action.connect.target_name,
						    strlen(step->action.connect.target_name), out,
						    out_cap, pos) != 0) {
					return -1;
				}
				break;

			case DBM_ACTION_DATA_EXCHANGE: {
				const struct dbm_data_exchange_action *de = &step->action.data_exchange;

				if (*pos + 32 > out_cap) {
					return -1;
				}
				memcpy(out + *pos, de->service_uuid, 16);
				*pos += 16;
				memcpy(out + *pos, de->characteristic_uuid, 16);
				*pos += 16;
				WRITE_VARINT(de->operation.kind);
				switch (de->operation.kind) {
				case DBM_GATT_OP_WRITE:
					if (pc_write_bytes(de->operation.payload,
							    de->operation.payload_len, out,
							    out_cap, pos) != 0) {
						return -1;
					}
					break;
				case DBM_GATT_OP_NOTIFY:
				case DBM_GATT_OP_INDICATE:
					WRITE_VARINT(de->operation.timeout_ms);
					break;
				default:
					break; /* Read/Subscribe/StreamCapture: no fields */
				}
				break;
			}

			case DBM_ACTION_BLE_SECURITY:
				/* One varint, the SecurityLevel discriminant
				 * (embarch-study-designer/design.md §3
				 * decision 44, schema v12). */
				WRITE_VARINT(step->action.set_security.level);
				break;

			case DBM_ACTION_GATT_MONITOR_SELECTED:
			case DBM_ACTION_GATT_MONITOR_SELECTED_START: {
				/* A sequence: length varint, then that many
				 * fixed 32-byte targets, no per-element length
				 * prefix (embarch-study-designer/design.md §3
				 * decision 53, schema v14). */
				const struct dbm_gatt_monitor_selected_action *ms =
					&step->action.monitor_selected;

				if (ms->targets_len > DBM_MAX_MONITOR_TARGETS) {
					return -1;
				}
				WRITE_VARINT(ms->targets_len);
				for (uint32_t t = 0; t < ms->targets_len; t++) {
					if (*pos + 32 > out_cap) {
						return -1;
					}
					memcpy(out + *pos, ms->targets[t].service_uuid, 16);
					*pos += 16;
					memcpy(out + *pos, ms->targets[t].characteristic_uuid, 16);
					*pos += 16;
				}
				break;
			}

			case DBM_ACTION_RUN_PROTOCOL:
				/* schema v15 -- two raw `u8` indices. */
				if (*pos + 2 > out_cap) {
					return -1;
				}
				out[(*pos)++] = step->action.run_protocol.protocol;
				out[(*pos)++] = step->action.run_protocol.entry_state;
				break;

			case DBM_ACTION_GATT_DISCOVER:
			case DBM_ACTION_GATT_MONITOR_ALL:
			case DBM_ACTION_GATT_MONITOR_START:
			case DBM_ACTION_GATT_MONITOR_STOP:
			case DBM_ACTION_BLE_UNBOND:
				break; /* field-less */

			default:
				return -1;
			}

			WRITE_VARINT(step->timeout_ms);
			/* `Step::power_sample` was encoded here as a permanent
			 * `None` byte and is **retired** at schema v9
			 * (embarch-study-designer/design.md §3 decision 39's
			 * 2026-08-25 amendment): a `StreamSource::PowerFrontEnd`
			 * tap is the only way to author a power capture now.
			 * Nothing ever read the field -- this encoder wrote None
			 * unconditionally, the decoder below read and discarded
			 * it, and the one host-side authoring path always emitted
			 * None. */
			if (*pos + 1 > out_cap) {
				return -1;
			}
			out[(*pos)++] = step->continue_on_fail ? 1 : 0;
			/* Step::delay_before_ms -- schema v6's trailing field
			 * (embarch-study-designer/design.md §3 decision 42). */
			WRITE_VARINT(step->delay_before_ms);
		}
		WRITE_VARINT(msg->study_start.steps_crc);
		/* `streams` + `streams_crc` (schema v9, design.md §3 decision 39
		 * and its 2026-08-25 amendment). This firmware never *sends* a
		 * StudyStart -- Core does -- so this encoder exists only for the
		 * round-trip tests, and it has no taps of its own to write: an
		 * empty `Vec<StreamTap>` and, correspondingly, the CRC of nothing.
		 *
		 * That 0 is not a placeholder. CRC-32/ISO-HDLC over zero bytes is
		 * genuinely 0 (init and xorout both 0xFFFFFFFF, which cancel), so
		 * the decoder's own check below passes on these bytes for the
		 * right reason rather than by exemption. Unlike the two
		 * `power_samples_ref`/`waveform_ref` bytes this file used to
		 * write, these fields really do exist on the Rust type. */
		WRITE_VARINT(0); /* streams: Vec<StreamTap>, empty */
		WRITE_VARINT(0); /* streams_crc: CRC-32 of nothing */
		/* dev_bench_log_level -- schema v13 (design.md §3 decision 39).
		 * Round-tripped from the struct rather than written as a fixed
		 * value, which is what lets this file's own round-trip test prove
		 * the field survives both directions. */
		WRITE_VARINT(msg->study_start.dev_bench_log_level);
		/* protocols + protocols_crc -- schema v15 (design.md §3
		 * decision 58). Written as an **empty list**, exactly as
		 * `streams` above and for the identical reason: this encoder
		 * exists only for this file's own round-trip tests, and a
		 * `ProtocolDef` cannot be re-encoded from `struct
		 * eap_protocol_def` at all -- the decoder discards every name in
		 * a manifest but the state names (serial_protocol.h), so a
		 * re-encode would produce different bytes and a different CRC
		 * from the ones that arrived.
		 *
		 * That is not a gap in coverage, it is where the coverage moved
		 * to: decision 36's both-languages rule puts the real proof in a
		 * literal frame this crate produced, decoded here and asserted
		 * field by field (app/tests/serial_protocol), which is the only
		 * kind of test that can catch the two languages disagreeing.
		 *
		 * The 0 CRC is genuine, not a placeholder: CRC-32/ISO-HDLC over
		 * zero bytes is 0, so the decoder's own check passes on these
		 * bytes for the right reason. */
		WRITE_VARINT(0); /* protocols: Vec<ProtocolDef>, empty */
		WRITE_VARINT(0); /* protocols_crc: CRC-32 of nothing */
		return 0;
	case DBM_TAG_STEP_RESULT: {
		const struct dbm_step_result_payload *r = &msg->step_result.result;

		WRITE_VARINT(msg->step_result.step_index);
		if (pc_write_bytes((const uint8_t *)r->step_name, strlen(r->step_name), out, out_cap,
				    pos) != 0) {
			return -1;
		}
		WRITE_VARINT(r->outcome.tag);
		if (r->outcome.tag == 1 &&
		    pc_write_bytes((const uint8_t *)r->outcome.fail_reason,
				    strlen(r->outcome.fail_reason), out, out_cap, pos) != 0) {
			return -1;
		}
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = r->has_captured_data ? 1 : 0;
		if (r->has_captured_data &&
		    pc_write_bytes(r->captured_data, r->captured_data_len, out, out_cap, pos) != 0) {
			return -1;
		}
		/* `power_samples_ref`/`waveform_ref` were encoded here as two
		 * permanent `None` bytes. They are **retired** from `StepResult`
		 * by design.md §3 decision 39 (schema v8) -- a capture belongs to
		 * the study's declared taps, reported once as
		 * `StudyResult.streams`, not to one step -- and this file kept
		 * writing them anyway. Removed at v9, alongside `power_sample`:
		 * two bytes Rust does not expect, on the one message this
		 * firmware sends most.
		 *
		 * Found by walking this encoder against the Rust type while
		 * implementing v9, not by a test: nothing pinned `StepResult`'s
		 * bytes across the two languages, because it was not a *new*
		 * record when decision 36's both-languages rule came in. The v9
		 * pass adds that pin (app/tests/serial_protocol) so the gap
		 * cannot reopen.
		 *
		 * gatt_services (design.md §3 decisions 31/32) -- unlike those
		 * two, this firmware populates it for real. */
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = r->has_gatt_services ? 1 : 0;
		if (r->has_gatt_services) {
			if (r->gatt_services_len > DBM_MAX_DISCOVERED_SERVICES) {
				return -1;
			}
			WRITE_VARINT(r->gatt_services_len);
			for (uint32_t s = 0; s < r->gatt_services_len; s++) {
				const struct dbm_gatt_service_info *svc = &r->gatt_services[s];

				if (*pos + 16 > out_cap) {
					return -1;
				}
				memcpy(out + *pos, svc->uuid, 16);
				*pos += 16;
				if (svc->characteristics_len > DBM_MAX_CHARS_PER_SERVICE) {
					return -1;
				}
				WRITE_VARINT(svc->characteristics_len);
				for (uint32_t c = 0; c < svc->characteristics_len; c++) {
					const struct dbm_gatt_characteristic_info *chr =
						&svc->characteristics[c];

					if (*pos + 17 > out_cap) {
						return -1;
					}
					memcpy(out + *pos, chr->uuid, 16);
					*pos += 16;
					out[(*pos)++] = chr->properties; /* u8: raw byte */
				}
			}
		}

		/* `gatt_activity`'s `Option` byte and its records were encoded
		 * here. **Retired at schema v14**
		 * (embarch-study-designer/design.md §3 decision 54): the field
		 * is gone from the Rust `StepResult`, so writing even the `None`
		 * byte for it would shift `security_level` by one and decode as
		 * a security level that was never reported -- the precise
		 * failure the `power_samples_ref`/`waveform_ref` bytes caused
		 * for a whole schema version before v9 caught them. The v14 wire
		 * vector pins the 21-byte frame that proves it doesn't. */

		/* `security_level: Option<SecurityLevel>` -- schema v12's
		 * trailing field (embarch-study-designer/design.md §3 decision
		 * 50), and the last field of `StepResult` since v14. */
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = r->has_security_level ? 1 : 0;
		if (r->has_security_level) {
			WRITE_VARINT(r->security_level);
		}

		/* `protocol: Option<ProtocolOutcome>` -- schema v15's trailing
		 * field (embarch-study-designer/design.md §3 decision 62), and
		 * the last field of `StepResult` since v15.
		 *
		 * `None` for every action but `RunProtocol`, which is every
		 * action that existed before it -- so on the message this
		 * firmware sends most, this is one `0x00` byte. That is why the
		 * field was appended rather than inserted: the wire diff a human
		 * has to check is a suffix, and this decoder adopts it by
		 * reading one more Option at the end rather than by re-walking
		 * the message. */
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = r->has_protocol ? 1 : 0;
		if (r->has_protocol) {
			if (pc_write_bytes((const uint8_t *)r->protocol_final_state,
					    strlen(r->protocol_final_state), out, out_cap,
					    pos) != 0) {
				return -1;
			}
			WRITE_VARINT(r->protocol_outcome.tag);
			if (r->protocol_outcome.tag == 1 &&
			    pc_write_bytes((const uint8_t *)r->protocol_outcome.fail_reason,
					    strlen(r->protocol_outcome.fail_reason), out, out_cap,
					    pos) != 0) {
				return -1;
			}
		}
		return 0;
	}
	case DBM_TAG_STUDY_DONE:
		if (*pos + 1 > out_cap) {
			return -1;
		}
		out[(*pos)++] = msg->study_done.completed ? 1 : 0;
		return 0;
	default:
		return -1;
	}

#undef WRITE_VARINT
}

int dbm_encode_frame(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap)
{
	/* `static`, not a stack-local array: DBM_MAX_RAW_LEN now scales with
	 * DBM_MAX_STEPS_PER_STUDY (StudyStart's own worst case), too large for
	 * a small embedded call stack, especially with dbm_decode_frame's own
	 * same-size scratch buffer potentially live in a caller's frame at the
	 * same time (e.g. main.c's send_message_locked). Safe because every
	 * caller holds main.c's link_tx_mutex -- which, as of design.md §3
	 * decision 36, is what serializes this static, not the old
	 * single-threaded-link assumption: the transcript TX thread is a
	 * second sender, and it has to keep draining while the dispatch loop
	 * is blocked inside a long ble_bridge_execute(). Same posture as
	 * receive_message's static rx_buf in main.c, which stays
	 * single-reader. */
	static uint8_t raw[DBM_MAX_RAW_LEN];
	size_t raw_len = 0;

	if (encode_body(msg, raw, sizeof(raw), &raw_len) != 0) {
		return -1;
	}

	/* cobs_encode's worst case output is raw_len + ceil(raw_len / 254) + 1. */
	size_t cobs_cap = raw_len + (raw_len / 254) + 2;

	if (cobs_cap + 1 > out_cap) {
		return -1;
	}
	size_t cobs_len = cobs_encode(raw, raw_len, out);

	out[cobs_len] = 0x00;
	return (int)(cobs_len + 1);
}

static int decode_body(const uint8_t *raw, size_t raw_len, struct dev_bench_message *msg)
{
	size_t pos = 0;
	uint64_t tag;

	if (pc_read_varint(raw, raw_len, &pos, &tag) != 0) {
		return -1;
	}
	msg->tag = (enum dbm_tag)tag;

	uint64_t tmp;

	switch (msg->tag) {
	case DBM_TAG_HELLO:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello.schema_version = (uint32_t)tmp;
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello.host_utc_ms = tmp;
		return 0;
	case DBM_TAG_HELLO_ACK:
		if (pc_read_varint(raw, raw_len, &pos, &tmp) != 0) {
			return -1;
		}
		msg->hello_ack.schema_version = (uint32_t)tmp;
		if (pos >= raw_len) {
			return -1;
		}
		msg->hello_ack.compatible = raw[pos++] != 0;
		if (pc_read_str(raw, raw_len, &pos, msg->hello_ack.firmware_version,
				 sizeof(msg->hello_ack.firmware_version)) != 0) {
			return -1;
		}
		/* schema v10 (embarch-study-designer/design.md §3 decision 47).
		 * dev-bench never receives a HelloAck in service -- this arm
		 * exists so the round-trip tests can walk what the encoder above
		 * wrote, which is precisely what caught the stale-Option drift
		 * in StepResult. */
		return pc_read_str(raw, raw_len, &pos, msg->hello_ack.hardware_id,
				    sizeof(msg->hello_ack.hardware_id));
	/* No decode arm for DBM_TAG_STREAM_OPEN/STREAM_CHUNK_BATCH/
	 * STREAM_CLOSE: dev-bench only ever *sends* those three, and Core is
	 * the only reader. A C-side round trip would prove this encoder
	 * self-consistent while saying nothing about whether Rust agrees --
	 * which is the only thing that matters, and is what the literal-frame
	 * pinning in app/tests/serial_protocol covers instead. Their frames
	 * therefore decode here as an unknown tag, deliberately. */
	case DBM_TAG_LOG_LINE:
		return pc_read_str(raw, raw_len, &pos, msg->log_line.text,
				    sizeof(msg->log_line.text));
	case DBM_TAG_STUDY_START: {
		struct dbm_study_start *ss = &msg->study_start;

		memset(ss, 0, sizeof(*ss));

		uint64_t steps_len;

		if (pc_read_varint(raw, raw_len, &pos, &steps_len) != 0) {
			return -1;
		}
		if (steps_len > DBM_MAX_STEPS_PER_STUDY) {
			return -1;
		}

		size_t steps_start_pos = pos;
		uint32_t decoded = 0;
		bool unsupported = false;

		for (uint32_t i = 0; i < steps_len; i++) {
			struct dbm_step *step = &ss->steps[i];

			if (pc_read_str(raw, raw_len, &pos, step->name, sizeof(step->name)) != 0) {
				return -1;
			}

			uint64_t action_tag;

			if (pc_read_varint(raw, raw_len, &pos, &action_tag) != 0) {
				return -1;
			}

			bool recognized = true;

			switch (action_tag) {
			case DBM_ACTION_BLE_ADVERTISE: {
				if (pos >= raw_len) {
					return -1;
				}
				bool has_local_name = raw[pos++] != 0;

				if (has_local_name) {
					if (pc_read_str(raw, raw_len, &pos,
							 step->action.advertise.local_name,
							 sizeof(step->action.advertise.local_name)) !=
					    0) {
						return -1;
					}
				} else {
					step->action.advertise.local_name[0] = '\0';
				}
				step->action.advertise.has_local_name = has_local_name;

				/* service_uuids: Vec<Uuid,4> -- not stored, decision 21's
				 * original scope note, unchanged by decisions 31/32. */
				if (pc_skip_len_prefixed(raw, raw_len, &pos, 16) != 0) {
					return -1;
				}

				uint64_t adv_interval_ms;

				if (pc_read_varint(raw, raw_len, &pos, &adv_interval_ms) != 0) {
					return -1;
				}
				step->action.advertise.adv_interval_ms = (uint16_t)adv_interval_ms;
				break;
			}

			case DBM_ACTION_BLE_CONNECT: {
				uint64_t role;

				if (pc_read_varint(raw, raw_len, &pos, &role) != 0) {
					return -1;
				}
				step->action.connect.role = (uint8_t)role;

				if (pos >= raw_len) {
					return -1;
				}
				bool has_target = raw[pos++] != 0;

				step->action.connect.has_target_address = has_target;
				if (has_target) {
					if (pos + 6 > raw_len) {
						return -1;
					}
					memcpy(step->action.connect.target_address, raw + pos, 6);
					pos += 6;

					uint64_t kind;

					if (pc_read_varint(raw, raw_len, &pos, &kind) != 0) {
						return -1;
					}
					step->action.connect.target_address_kind = (uint8_t)kind;
				} else {
					memset(step->action.connect.target_address, 0, 6);
					step->action.connect.target_address_kind = 0;
				}

				/* target_name: Option<String> -- schema v7 (design.md §3
				 * decision 43). Read unconditionally: the Hello/HelloAck
				 * schema-version handshake has already refused any peer
				 * that wouldn't have sent it. */
				if (pos >= raw_len) {
					return -1;
				}
				bool has_target_name = raw[pos++] != 0;

				if (has_target_name) {
					if (pc_read_str(raw, raw_len, &pos,
							 step->action.connect.target_name,
							 sizeof(step->action.connect.target_name)) !=
					    0) {
						return -1;
					}
				} else {
					step->action.connect.target_name[0] = '\0';
				}
				step->action.connect.has_target_name = has_target_name;
				break;
			}

			case DBM_ACTION_DATA_EXCHANGE: {
				struct dbm_data_exchange_action *de = &step->action.data_exchange;

				if (pos + 32 > raw_len) {
					return -1;
				}
				memcpy(de->service_uuid, raw + pos, 16);
				pos += 16;
				memcpy(de->characteristic_uuid, raw + pos, 16);
				pos += 16;

				uint64_t op_kind;

				if (pc_read_varint(raw, raw_len, &pos, &op_kind) != 0) {
					return -1;
				}
				de->operation.kind = (uint8_t)op_kind;
				de->operation.payload_len = 0;
				de->operation.timeout_ms = 0;

				switch (op_kind) {
				case DBM_GATT_OP_WRITE: {
					uint64_t len;

					if (pc_read_varint(raw, raw_len, &pos, &len) != 0) {
						return -1;
					}
					if (len > sizeof(de->operation.payload) ||
					    pos + len > raw_len) {
						return -1;
					}
					memcpy(de->operation.payload, raw + pos, (size_t)len);
					pos += (size_t)len;
					de->operation.payload_len = (uint32_t)len;
					break;
				}
				case DBM_GATT_OP_NOTIFY:
				case DBM_GATT_OP_INDICATE: {
					uint64_t t;

					if (pc_read_varint(raw, raw_len, &pos, &t) != 0) {
						return -1;
					}
					de->operation.timeout_ms = (uint32_t)t;
					break;
				}
				case DBM_GATT_OP_READ:
				case DBM_GATT_OP_SUBSCRIBE:
				case DBM_GATT_OP_STREAM_CAPTURE:
					break;
				default:
					/* A malformed/future GattOperation kind this decoder
					 * doesn't recognize -- unlike an unrecognized Action
					 * kind (which can safely be reported as
					 * has_unsupported_action, since GattDiscover/
					 * GattMonitorAll's own field-less shape means nothing
					 * after the tag needs walking), an operation kind this
					 * decoder can't parse leaves the remaining bytes of
					 * this DataExchange action unwalkable -- so this is a
					 * hard decode error, not a per-step "unsupported"
					 * fallback. */
					return -1;
				}
				break;
			}

			case DBM_ACTION_BLE_SECURITY: {
				uint64_t level;

				if (pc_read_varint(raw, raw_len, &pos, &level) != 0) {
					return -1;
				}
				if (level > DBM_SECURITY_L4) {
					/* A level this firmware has no mapping
					 * for. Unlike an unrecognized *action*
					 * tag, the remaining bytes are still
					 * walkable (the varint was consumed),
					 * so this could have been a per-step
					 * "unsupported" -- but a security step
					 * that silently became a weaker one is
					 * the exact silent-degradation failure
					 * decision 44 exists to refuse, so it
					 * is a hard decode error instead. */
					return -1;
				}
				step->action.set_security.level = (uint8_t)level;
				break;
			}

			case DBM_ACTION_GATT_MONITOR_SELECTED:
			case DBM_ACTION_GATT_MONITOR_SELECTED_START: {
				/* schema v14 -- the first Action variants to
				 * carry a sequence. A decoder that treated
				 * these as field-less would read this length
				 * varint as the next step's name length and
				 * decode the rest of the study into nonsense
				 * that still parses; the crate's v14 wire
				 * vector is pinned against exactly that. */
				uint64_t targets_len;

				if (pc_read_varint(raw, raw_len, &pos, &targets_len) != 0) {
					return -1;
				}
				if (targets_len > DBM_MAX_MONITOR_TARGETS) {
					/* Core considers this legal; this
					 * firmware cannot hold it. A hard
					 * decode error rather than a truncated
					 * subscription list: a study that
					 * silently monitored a subset of what
					 * it named is the silently-empty
					 * capture this whole family of
					 * decisions keeps being opened by. */
					return -1;
				}
				if (pos + (size_t)targets_len * 32 > raw_len) {
					return -1;
				}
				for (uint32_t t = 0; t < (uint32_t)targets_len; t++) {
					struct dbm_gatt_target *tgt =
						&step->action.monitor_selected.targets[t];

					memcpy(tgt->service_uuid, raw + pos, 16);
					pos += 16;
					memcpy(tgt->characteristic_uuid, raw + pos, 16);
					pos += 16;
				}
				step->action.monitor_selected.targets_len = (uint32_t)targets_len;
				break;
			}

			case DBM_ACTION_RUN_PROTOCOL: {
				/* schema v15 -- two raw `u8` indices, not
				 * varints (embarch-study-designer/src/study.rs's
				 * `Action::RunProtocol { protocol, entry_state }`).
				 * Range-checking them against the study's own
				 * `protocols` happens at dispatch, not here:
				 * `protocols` arrives *after* every step on the
				 * wire, so at this point there is nothing yet to
				 * check against. */
				if (pos + 2 > raw_len) {
					return -1;
				}
				step->action.run_protocol.protocol = raw[pos++];
				step->action.run_protocol.entry_state = raw[pos++];
				break;
			}

			case DBM_ACTION_GATT_DISCOVER:
			case DBM_ACTION_GATT_MONITOR_ALL:
			case DBM_ACTION_GATT_MONITOR_START:
			case DBM_ACTION_GATT_MONITOR_STOP:
			case DBM_ACTION_BLE_UNBOND:
				break; /* field-less */

			default:
				/* A future Action variant this decoder predates. Everything
				 * after this tag (this step's remaining fields, any further
				 * steps, steps_crc) has an unknown shape we can't safely
				 * walk past, so the whole StudyStart is rejected (see this
				 * function's own doc comment in the header). */
				recognized = false;
				break;
			}

			if (!recognized) {
				unsupported = true;
				break;
			}
			step->action_tag = (uint8_t)action_tag;

			uint64_t timeout_ms;

			if (pc_read_varint(raw, raw_len, &pos, &timeout_ms) != 0) {
				return -1;
			}
			step->timeout_ms = (uint32_t)timeout_ms;

			/* `power_sample` was read-and-discarded here and is
			 * **retired** at schema v9 (design.md §3 decision 39's
			 * 2026-08-25 amendment). `continue_on_fail` now follows
			 * `timeout_ms` directly. */
			if (pos >= raw_len) {
				return -1;
			}
			step->continue_on_fail = raw[pos++] != 0;

			/* Step::delay_before_ms -- schema v6's trailing field
			 * (embarch-study-designer/design.md §3 decision 42). Reading it
			 * is unconditional: the Hello/HelloAck schema-version handshake
			 * has already refused any peer that wouldn't have sent it, so a
			 * missing varint here is a genuine truncated frame, not an old
			 * sender to be tolerated. */
			uint64_t delay_before_ms;

			if (pc_read_varint(raw, raw_len, &pos, &delay_before_ms) != 0) {
				return -1;
			}
			step->delay_before_ms = (uint32_t)delay_before_ms;

			decoded++;
		}

		ss->steps_len = decoded;
		ss->has_unsupported_action = unsupported;

		if (unsupported) {
			ss->steps_crc_valid = false;
			ss->streams_crc_valid = false;
			ss->protocols_crc_valid = false;
			return 0;
		}

		size_t steps_end_pos = pos;
		uint64_t steps_crc;

		if (pc_read_varint(raw, raw_len, &pos, &steps_crc) != 0) {
			return -1;
		}
		ss->steps_crc = (uint32_t)steps_crc;
		ss->steps_crc_valid =
			dbm_crc32(raw + steps_start_pos, steps_end_pos - steps_start_pos) == ss->steps_crc;

		/* streams + streams_crc -- schema v9 (design.md §3 decision 39's
		 * 2026-08-25 amendment). A **sibling** seal, checked independently
		 * of steps_crc above, which is the whole point of there being two:
		 * a mismatch says which half is corrupt.
		 *
		 * Both spans are contiguous and each CRC immediately follows the
		 * one it covers, which is exactly why this is a second CRC rather
		 * than a widened one -- steps_crc sits *between* steps and
		 * streams, so one CRC over both would mean digesting two
		 * non-contiguous spans here, or reshuffling StudyStart's fields.
		 *
		 * Like steps_crc, the digest covers the concatenated element
		 * encodings and **not** the vector's own length prefix, matching
		 * `streams_crc()`'s one-tap-at-a-time digest in src/crc.rs. */
		uint64_t streams_len;

		if (pc_read_varint(raw, raw_len, &pos, &streams_len) != 0) {
			return -1;
		}
		if (streams_len > DBM_MAX_STREAMS_PER_STUDY) {
			return -1;
		}

		size_t streams_start_pos = pos;

		for (uint32_t i = 0; i < streams_len; i++) {
			if (pc_read_stream_tap(raw, raw_len, &pos, &ss->streams[i]) != 0) {
				return -1;
			}
		}
		ss->streams_len = (uint32_t)streams_len;

		size_t streams_end_pos = pos;
		uint64_t streams_crc;

		if (pc_read_varint(raw, raw_len, &pos, &streams_crc) != 0) {
			return -1;
		}
		ss->streams_crc = (uint32_t)streams_crc;
		ss->streams_crc_valid =
			dbm_crc32(raw + streams_start_pos, streams_end_pos - streams_start_pos) ==
			ss->streams_crc;

		/* dev_bench_log_level -- schema v13 (design.md §3 decision 39).
		 * Read *after* the streams_crc span above, so it is outside both
		 * seals by construction rather than by remembering to exclude it.
		 *
		 * An unknown discriminant is clamped rather than rejected: a
		 * newer Core asking for a level this build has no name for is not
		 * a reason to refuse a study, and the handshake's own
		 * schema_version check is what actually guards wire drift. */
		uint64_t log_level;

		if (pc_read_varint(raw, raw_len, &pos, &log_level) != 0) {
			return -1;
		}
		if (log_level > DBM_LOG_LEVEL_DBG) {
			log_level = DBM_LOG_LEVEL_DBG;
		}
		ss->dev_bench_log_level = (uint8_t)log_level;

		/* protocols + protocols_crc -- schema v15
		 * (embarch-study-designer/design.md §3 decision 58, §4.9). The
		 * study's **third** seal, and a sibling of the two above rather
		 * than a widening of either: each covers one contiguous span and
		 * is carried immediately after it, so this hand-written C
		 * digests one run of bytes per seal and a mismatch names which
		 * of the three is corrupt.
		 *
		 * Appended after `dev_bench_log_level` rather than inserted
		 * beside `streams_crc` -- postcard is positional, and an append
		 * makes the wire diff a suffix. The structural rule is satisfied
		 * by `protocols_crc` following `protocols`, which is a property
		 * of the pair, not of where the pair sits.
		 *
		 * Like its two siblings, the digest covers the concatenated
		 * element encodings and **not** the vector's own length prefix,
		 * matching `protocols_crc()`'s one-protocol-at-a-time digest in
		 * src/crc.rs. */
		uint64_t protocols_len;

		if (pc_read_varint(raw, raw_len, &pos, &protocols_len) != 0) {
			return -1;
		}
		if (protocols_len > DBM_MAX_PROTOCOLS_PER_STUDY) {
			return -1;
		}

		size_t protocols_start_pos = pos;

		for (uint32_t i = 0; i < protocols_len; i++) {
			if (pc_read_protocol_def(raw, raw_len, &pos, &ss->protocols[i]) != 0) {
				return -1;
			}
		}
		ss->protocols_len = (uint32_t)protocols_len;

		size_t protocols_end_pos = pos;

		/* The disclosed byte cap (DBM_MAX_PROTOCOLS_WIRE_LEN). Checked
		 * on the *walked* span rather than guessed at from the counts,
		 * and after the walk rather than before it, because postcard
		 * carries no length for a sequence's bytes -- the only way to
		 * know how big the span is, is to have crossed it. A span this
		 * firmware could not have received at all is caught earlier
		 * still, by the frame-length check in dbm_decode_frame. */
		if (protocols_end_pos - protocols_start_pos > DBM_MAX_PROTOCOLS_WIRE_LEN) {
			return -1;
		}

		uint64_t protocols_crc;

		if (pc_read_varint(raw, raw_len, &pos, &protocols_crc) != 0) {
			return -1;
		}
		ss->protocols_crc = (uint32_t)protocols_crc;
		ss->protocols_crc_valid =
			dbm_crc32(raw + protocols_start_pos,
				  protocols_end_pos - protocols_start_pos) == ss->protocols_crc;
		return 0;
	}
	case DBM_TAG_STEP_RESULT: {
		struct dbm_step_result *sr = &msg->step_result;

		memset(sr, 0, sizeof(*sr));

		uint64_t step_index;

		if (pc_read_varint(raw, raw_len, &pos, &step_index) != 0) {
			return -1;
		}
		sr->step_index = (uint32_t)step_index;

		if (pc_read_str(raw, raw_len, &pos, sr->result.step_name,
				 sizeof(sr->result.step_name)) != 0) {
			return -1;
		}

		uint64_t outcome_tag;

		if (pc_read_varint(raw, raw_len, &pos, &outcome_tag) != 0) {
			return -1;
		}
		if (outcome_tag > 2) {
			return -1;
		}
		sr->result.outcome.tag = (uint8_t)outcome_tag;
		if (outcome_tag == 1 &&
		    pc_read_str(raw, raw_len, &pos, sr->result.outcome.fail_reason,
				sizeof(sr->result.outcome.fail_reason)) != 0) {
			return -1;
		}

		if (pos >= raw_len) {
			return -1;
		}
		bool has_captured_data = raw[pos++] != 0;

		sr->result.has_captured_data = has_captured_data;
		if (has_captured_data) {
			uint64_t captured_len;

			if (pc_read_varint(raw, raw_len, &pos, &captured_len) != 0) {
				return -1;
			}
			if (captured_len > sizeof(sr->result.captured_data) ||
			    pos + captured_len > raw_len) {
				return -1;
			}
			memcpy(sr->result.captured_data, raw + pos, (size_t)captured_len);
			pos += (size_t)captured_len;
			sr->result.captured_data_len = (uint32_t)captured_len;
		}

		/* `power_samples_ref`/`waveform_ref` were skipped here; both are
		 * retired from `StepResult` (design.md §3 decision 39) and the
		 * skip is removed at v9 -- see this file's encoder for the full
		 * account of why it outlived the fields. */

		/* gatt_services (design.md §3 decisions 31/32) --
		 * this firmware only ever encodes these (see encode_body), but
		 * decode is exercised by this file's own round-trip tests too. */
		if (pos >= raw_len) {
			return -1;
		}
		bool has_gatt_services = raw[pos++] != 0;

		sr->result.has_gatt_services = has_gatt_services;
		if (has_gatt_services) {
			uint64_t services_len;

			if (pc_read_varint(raw, raw_len, &pos, &services_len) != 0) {
				return -1;
			}
			if (services_len > DBM_MAX_DISCOVERED_SERVICES) {
				return -1;
			}
			for (uint32_t s = 0; s < services_len; s++) {
				struct dbm_gatt_service_info *svc = &sr->result.gatt_services[s];

				if (pos + 16 > raw_len) {
					return -1;
				}
				memcpy(svc->uuid, raw + pos, 16);
				pos += 16;

				uint64_t chars_len;

				if (pc_read_varint(raw, raw_len, &pos, &chars_len) != 0) {
					return -1;
				}
				if (chars_len > DBM_MAX_CHARS_PER_SERVICE) {
					return -1;
				}
				for (uint32_t c = 0; c < chars_len; c++) {
					struct dbm_gatt_characteristic_info *chr =
						&svc->characteristics[c];

					if (pos + 17 > raw_len) {
						return -1;
					}
					memcpy(chr->uuid, raw + pos, 16);
					pos += 16;
					chr->properties = raw[pos++]; /* u8: raw byte */
				}
				svc->characteristics_len = (uint32_t)chars_len;
			}
			sr->result.gatt_services_len = (uint32_t)services_len;
		}

		/* `gatt_activity` was read here. Retired at schema v14
		 * (embarch-study-designer/design.md §3 decision 54) -- see the
		 * matching note in the encoder above. */

		/* `security_level: Option<SecurityLevel>` -- schema v12's
		 * trailing field (embarch-study-designer/design.md §3 decision
		 * 50). Read unconditionally: the Hello/HelloAck schema-version
		 * handshake has already refused any peer that wouldn't have
		 * sent it. */
		if (pos >= raw_len) {
			return -1;
		}
		bool has_security_level = raw[pos++] != 0;

		sr->result.has_security_level = has_security_level;
		if (has_security_level) {
			uint64_t level;

			if (pc_read_varint(raw, raw_len, &pos, &level) != 0) {
				return -1;
			}
			if (level > DBM_SECURITY_L4) {
				return -1;
			}
			sr->result.security_level = (uint8_t)level;
		}

		/* `protocol: Option<ProtocolOutcome>` -- schema v15's trailing
		 * field (embarch-study-designer/design.md §3 decision 62). */
		if (pos >= raw_len) {
			return -1;
		}
		bool has_protocol = raw[pos++] != 0;

		sr->result.has_protocol = has_protocol;
		if (has_protocol) {
			uint64_t protocol_outcome_tag;

			if (pc_read_str(raw, raw_len, &pos, sr->result.protocol_final_state,
					 sizeof(sr->result.protocol_final_state)) != 0) {
				return -1;
			}
			if (pc_read_varint(raw, raw_len, &pos, &protocol_outcome_tag) != 0) {
				return -1;
			}
			if (protocol_outcome_tag > 2) {
				return -1;
			}
			sr->result.protocol_outcome.tag = (uint8_t)protocol_outcome_tag;
			if (protocol_outcome_tag == 1 &&
			    pc_read_str(raw, raw_len, &pos,
					sr->result.protocol_outcome.fail_reason,
					sizeof(sr->result.protocol_outcome.fail_reason)) != 0) {
				return -1;
			}
		}
		return 0;
	}
	case DBM_TAG_STUDY_DONE:
		if (pos >= raw_len) {
			return -1;
		}
		msg->study_done.completed = raw[pos++] != 0;
		return 0;
	default:
		return -1;
	}
}

int dbm_decode_frame(const uint8_t *in, size_t in_len, struct dev_bench_message *out)
{
	/* `static`, not stack-local -- see dbm_encode_frame's own comment above. */
	static uint8_t raw[DBM_MAX_RAW_LEN];

	if (in_len == 0 || in_len > DBM_MAX_FRAME_LEN) {
		return -1;
	}
	size_t raw_len = cobs_decode(in, in_len, raw, sizeof(raw));

	if (raw_len == 0 || raw_len > sizeof(raw)) {
		return -1;
	}
	return decode_body(raw, raw_len, out);
}
