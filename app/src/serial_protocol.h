/* DevBenchMessage wire protocol: COBS framing + postcard-compatible encoding.
 *
 * Decisions 7/10/12/20. Mirrors
 * embarch-study-designer's `protocol::DevBenchMessage` field-for-field and
 * byte-for-byte (postcard varint/fixint rules) so this hand-written C
 * implementation and the Rust crate's own round-trip tests both describe the
 * identical wire format. `embarch-study-designer`'s FFI surface doesn't cover
 * DevBenchMessage encode/decode (only `Study` decode+CRC, see study_ffi.h) —
 * that's this file's job.
 *
 * Pure encode/decode logic only, no I/O — keeps this unit-testable on any
 * host (see app/tests/serial_protocol) independent of a real UART.
 */
#ifndef EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_
#define EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Mirrors embarch-study-designer's limits::MAX_FIRMWARE_VERSION_LEN / MAX_LOG_LINE_LEN
 * / MAX_LOCAL_NAME_LEN / MAX_NAME_LEN / MAX_STEPS_PER_STUDY / MAX_FAIL_REASON_LEN /
 * MAX_PAYLOAD_LEN (embarch-study-designer/src/limits.rs) — kept in sync by hand until
 * decision 8's west-module wiring lets this firmware pull the constants directly from
 * that crate. */
#define DBM_MAX_FIRMWARE_VERSION_LEN 32
/* limits::MAX_HARDWARE_ID_LEN — this board's own factory-unique chip ID,
 * hex-encoded (`embarch-study-designer` decision 47,
 * `embarch-core` decision 35). */
#define DBM_MAX_HARDWARE_ID_LEN 32
#define DBM_MAX_LOG_LINE_LEN 128
#define DBM_MAX_LOCAL_NAME_LEN 26
#define DBM_MAX_NAME_LEN 32
/* A dev-bench-internal capacity cap, NOT a mirror of the crate's own
 * limits::MAX_STEPS_PER_STUDY (still 64 there, unaffected -- Core/embarch-api
 * still validate/CRC a `Study` against that ceiling). Real gap found and
 * fixed, Milestone 3 (Study Designer: Feature-Branch Iteration): once
 * `struct dbm_step`'s action union had to grow to also hold
 * `Action::DataExchange`'s up-to-512-byte `Write` payload (decisions
 * 31/32's own GattDiscover/GattMonitorAll additions triggered writing this
 * union out in full for the first time), a full 64-slot `steps[]` array
 * pushed `struct dev_bench_message`'s union well past what this board's
 * available RAM can hold several static copies of (confirmed empirically --
 * a real build at 64 slots would not fit). 16 is sized well above every `Study` this suite
 * has actually authored so far (the largest is this milestone's own 3-step
 * self-test) with real headroom, not against a proven fuzzing need -- a
 * `StudyStart` claiming more than this many steps is rejected outright by
 * `dbm_decode_frame` (a real, disclosed dev-bench capacity limit, not a
 * silent truncation), the same way an oversized `steps_len` already was
 * before this cap existed for any other reason. Revisit this number, not
 * the crate's own MAX_STEPS_PER_STUDY, if a real fuzzing workload ever
 * needs more steps than this against real dev-bench hardware.
 */
#define DBM_MAX_STEPS_PER_STUDY 16
#define DBM_MAX_FAIL_REASON_LEN 64
#define DBM_MAX_PAYLOAD_LEN 512
/* Mirrors embarch-study-designer's limits::MAX_DISCOVERED_SERVICES/
 * MAX_CHARS_PER_SERVICE (`embarch-study-designer` decisions 31/32/33's update) --
 * these two, unlike DBM_MAX_STEPS_PER_STUDY above, are small enough that
 * mirroring the crate's own real ceiling costs no meaningful RAM. */
#define DBM_MAX_DISCOVERED_SERVICES 8
#define DBM_MAX_CHARS_PER_SERVICE 16
/* DBM_MAX_GATT_ACTIVITY_RECORDS was here. Retired at schema v14 with the
 * field it bounded (`embarch-study-designer` decision 54): it was
 * the single largest contributor to `struct dbm_step_result_payload`'s size
 * (32 records at up to DBM_MAX_PAYLOAD_LEN bytes each) and had been flagged
 * as a real static-RAM risk from the outset by that doc's decision 32. What
 * it bounded was a capped in-memory copy of a capture the tap pipeline
 * already streams to Core uncapped, so the cap bought nothing and cost 16 KB
 * on a board whose sram0_0_seg has overflowed twice. */
/* Largest transcript-entry payload this firmware ever *produces* (`embarch-study-designer`
 * decision 36) -- one ATT MTU's worth of notification, not the crate's
 * full MAX_PAYLOAD_LEN. See `struct dbm_gatt_transcript_entry`'s own comment
 * for why the two deliberately differ. Sized to hold a full 247-byte ATT_MTU
 * notification (247 - 3 bytes of ATT header), the value app/prj.conf
 * configures CONFIG_BT_L2CAP_TX_MTU/CONFIG_BT_BUF_ACL_RX_SIZE for. */
#define DBM_MAX_TRANSCRIPT_PAYLOAD_LEN 244
/* Mirrors embarch-study-designer's limits::MAX_STREAM_CHUNK_BYTES /
 * MAX_STREAM_RECORDS_PER_BATCH (schema v8, that doc's §3 decision 39, §4.8).
 * Deliberately *smaller* than the crate's own 512/4 for the same reason
 * DBM_MAX_TRANSCRIPT_PAYLOAD_LEN is smaller than DBM_MAX_PAYLOAD_LEN: this
 * firmware never produces a record larger than one ATT MTU's worth of
 * notification, and sending fewer bytes than the receiving type can hold is
 * always wire-legal -- the reverse is not. */
#define DBM_MAX_STREAM_CHUNK_BYTES 256
#define DBM_MAX_STREAM_RECORDS_PER_BATCH 4
/* Mirrors embarch-study-designer's limits::MAX_STREAMS_PER_STUDY (schema
 * v9). At its full value rather than shrunk, unlike DBM_MAX_STEPS_PER_STUDY:
 * refusing a tap count Core considers legal would be this firmware inventing
 * a limit.
 *
 * Taps *are* stored now (Milestone 7 Phase B item 3), but `struct
 * dbm_stream_tap` is 12 bytes rather than the ~80 a full mirror of
 * `StreamTap` would cost -- see that struct for why storing `name` and
 * `encoding` here would be storing host-side knowledge this firmware is
 * specifically not supposed to hold. Eight of them is under 100 bytes, on a
 * board whose `sram0_0_seg` has already overflowed twice during this
 * decision's implementation (decisions 27, 28). */
#define DBM_MAX_STREAMS_PER_STUDY 8

/* The `.eap` protocol manifest types a `StudyStart` carries and this file
 * decodes (`embarch-study-designer` decisions 58-62, §4.9).
 * Their own header, because eap_interp.c and ble_bridge also need them and
 * neither needs the link protocol -- see eap.h for what those types drop and
 * why, and for the per-manifest count caps.
 *
 * The two constants below stay here rather than moving across with them:
 * both bound the *message*, not the grammar.
 */
#include "eap.h"

#define DBM_MAX_PROTOCOLS_PER_STUDY 2
/* Largest postcard-encoded `protocols` span this firmware will accept, in
 * bytes -- a **disclosed byte-count cap**, and the only limit here that is
 * not a mirror of a count.
 *
 * It exists because eap.h's count caps multiply into a wire bound nothing would
 * ever send: at the crate's ceilings a single `ProtocolDef` can encode to
 * ~7.4 KB, almost all of it names this firmware discards, and sizing three
 * staging buffers for two of those would cost ~30 KB for a span whose real
 * worked example (the BDS batch download, §4.9) is **398 bytes** including
 * the whole rest of the StudyStart. A byte cap says the true thing -- "this
 * bench accepts a manifest up to this big" -- where a product of eleven
 * ceilings says a false one.
 *
 * 3 KB is roughly eight times the real worked protocol. A span past it is
 * refused at decode with the limit named, the same disclosed-capacity posture
 * DBM_MAX_STEPS_PER_STUDY takes. */
#define DBM_MAX_PROTOCOLS_WIRE_LEN 3072

/* Largest single postcard-encoded (pre-COBS) DevBenchMessage this firmware sends/receives.
 *
 * StudyStart's per-step worst case grew once decode had to cover every
 * `Action` kind, not just BleAdvertise (`embarch-study-designer` decisions 31/32):
 * `DataExchange { GattOperation::Write { payload } }` dominates a single
 * step's own encoding now (service_uuid+characteristic_uuid, 16 bytes each,
 * plus a Write payload up to DBM_MAX_PAYLOAD_LEN bytes with its own length
 * varint), well past BleAdvertise's own ~30 bytes -- generously rounded up
 * to DBM_MAX_PAYLOAD_LEN + 64 bytes/step to cover it plus every other
 * per-step field with margin. GattDiscover/GattMonitorAll are both
 * field-less and add only their own action tag varint, cheaper than
 * BleAdvertise, so they don't move this bound.
 *
 * StepResult's own worst case is `gatt_services` (up to
 * DBM_MAX_DISCOVERED_SERVICES services, each up to DBM_MAX_CHARS_PER_SERVICE
 * characteristics at 17 bytes each) plus `captured_data`.
 *
 * **StudyStart is the larger message again as of schema v14.** It had not
 * been since gatt_activity arrived: that field added ~16.6 KB to StepResult's
 * bound and made it this file's largest by a factor of two. Retiring it
 * (`embarch-study-designer` decision 54) takes all of that back,
 * which is what shrinks DBM_MAX_RAW_LEN and every buffer sized from it.
 *
 * `+ DBM_MAX_PROTOCOLS_WIRE_LEN` at schema v15
 * (`embarch-study-designer` decision 58): the `protocols` span and
 * its seal ride at the end of StudyStart, and unlike `streams` they are far
 * too large to hide inside the per-step rounding slack. Counted as one
 * disclosed byte cap rather than as a product of the eleven `.eap` count
 * ceilings above -- see DBM_MAX_PROTOCOLS_WIRE_LEN for why.
 */
#define DBM_MAX_STUDY_START_LEN \
	(8 + (DBM_MAX_STEPS_PER_STUDY * (DBM_MAX_PAYLOAD_LEN + 64)) + 8 + \
	 DBM_MAX_PROTOCOLS_WIRE_LEN + 8)
#define DBM_MAX_GATT_SERVICES_LEN \
	(4 + (DBM_MAX_DISCOVERED_SERVICES * (16 + 4 + (DBM_MAX_CHARS_PER_SERVICE * 17))))
/* `+ EAP_MAX_STATE_NAME_LEN + DBM_MAX_FAIL_REASON_LEN + 8` at schema v15:
 * `StepResult.protocol: Option<ProtocolOutcome>` (`embarch-study-designer`
 * decision 62) is one trailing Option byte plus, when a
 * `RunProtocol` step really ran, a state name and an `Outcome` that can carry
 * its own `fail_reason`. */
#define DBM_MAX_STEP_RESULT_LEN                                                                  \
	(24 + DBM_MAX_NAME_LEN + DBM_MAX_FAIL_REASON_LEN + DBM_MAX_PAYLOAD_LEN +                  \
	 DBM_MAX_GATT_SERVICES_LEN + EAP_MAX_STATE_NAME_LEN + DBM_MAX_FAIL_REASON_LEN + 8)
/* A transcript record is tiny next to either of the two above -- a fixed
 * header plus one bounded payload -- so it never moves DBM_MAX_RAW_LEN.
 * Stated as its own constant anyway so the encoder has something real to
 * bounds-check against rather than borrowing an unrelated message's bound. */
#define DBM_MAX_TRANSCRIPT_RECORD_LEN (64 + DBM_MAX_TRANSCRIPT_PAYLOAD_LEN)
#define DBM_MAX_RAW_LEN (DBM_MAX_STUDY_START_LEN > DBM_MAX_STEP_RESULT_LEN ? DBM_MAX_STUDY_START_LEN \
										  : DBM_MAX_STEP_RESULT_LEN)
/* COBS worst case adds one overhead byte per 254 payload bytes, plus a leading code byte
 * and a trailing 0x00 delimiter. */
#define DBM_MAX_FRAME_LEN (DBM_MAX_RAW_LEN + (DBM_MAX_RAW_LEN / 254) + 2)

/* Largest frame this firmware can ever *send*, and the exact mirror of
 * DBM_MAX_INBOUND_RAW_LEN below -- the same finding, applied to the other
 * direction, and found the same way: by asking which messages actually cross
 * this link rather than which the type can hold.
 *
 * dev-bench sends HelloAck, LogLine, StreamOpen/StreamChunkBatch/StreamClose,
 * StepResult and StudyDone. It never sends a StudyStart -- Core is the only
 * sender -- so StepResult is the largest thing it can put on the wire, and
 * main.c's TX staging buffer was holding StudyStart's bound for it. That cost
 * ~6.6 KB before this schema version and would have cost ~9.7 KB after it,
 * since DBM_MAX_STUDY_START_LEN now carries DBM_MAX_PROTOCOLS_WIRE_LEN as
 * well -- growth in a buffer for a message this node cannot produce.
 *
 * Scoped to the *application's* TX buffer for the same reason its inbound
 * twin is scoped to the application's RX buffer: `dbm_encode_frame`'s own
 * staging buffer stays at DBM_MAX_RAW_LEN, because this suite's round-trip
 * tests encode a StudyStart through it precisely to prove encode and decode
 * agree, and narrowing it would trade a real test for RAM. */
#define DBM_MAX_OUTBOUND_RAW_LEN DBM_MAX_STEP_RESULT_LEN
#define DBM_MAX_OUTBOUND_FRAME_LEN \
	(DBM_MAX_OUTBOUND_RAW_LEN + (DBM_MAX_OUTBOUND_RAW_LEN / 254) + 2)

/* Largest frame this firmware can ever *receive*, as opposed to the largest it
 * can handle at all (DBM_MAX_FRAME_LEN above).
 *
 * The two are wildly different, and that difference is worth ~10 KB of SRAM on
 * a board that has none to spare. DBM_MAX_RAW_LEN is now StudyStart's bound
 * (it was StepResult's, 19808 bytes dominated by the retired `gatt_activity`,
 * until schema v14) -- and both are messages sized for a worst case Core
 * never sends. Core sends dev-bench exactly two messages:
 * `Hello` (a dozen bytes) and `StudyStart`. So an RX staging buffer sized to
 * DBM_MAX_FRAME_LEN is sized for a frame that cannot arrive.
 *
 * Found while making CONFIG_LOG fit on the ESP32-C5 (decision
 * 38): that board's `sram0_0_seg` was at 98.5% before the logging subsystem
 * asked for ~11 KB of it, ~5 KB of which is Espressif's linker script forcing
 * log_core/log_output/log_msg/cbprintf into IRAM and therefore not negotiable.
 * main.c's `receive_message` was holding 19887 bytes for a 9415-byte worst
 * case, which is where the room came from.
 *
 * Deliberately scoped to the *application's* RX buffer, not to
 * dbm_decode_frame's own staging buffer, which stays at DBM_MAX_RAW_LEN: the
 * decoder is a general one (this suite's round-trip tests decode StepResults
 * through it precisely to prove encode and decode agree), and narrowing it
 * would trade a real test for RAM. `+ streams + margin` because
 * DBM_MAX_STUDY_START_LEN's own formula predates StudyStart carrying
 * `streams`/`streams_crc` (decision 29(a)) and covers them only
 * out of its per-step rounding slack -- counted explicitly here rather than
 * left to that slack, since this bound now has RAM riding on it.
 */
#define DBM_MAX_INBOUND_RAW_LEN \
	(DBM_MAX_STUDY_START_LEN + (DBM_MAX_STREAMS_PER_STUDY * 16) + 16)
#define DBM_MAX_INBOUND_FRAME_LEN \
	(DBM_MAX_INBOUND_RAW_LEN + (DBM_MAX_INBOUND_RAW_LEN / 254) + 2)

/* Mirrors embarch-study-designer's `DevBenchLogLevel` (src/study.rs) -- these
 * are that enum's postcard discriminants, which are also deliberately its
 * Zephyr severity numbers (`DevBenchLogLevel::zephyr_level`), so no
 * translation table is needed on this side. Schema v13, `embarch-study-designer`
 * decision 39.
 *
 * Appended, never reordered, for the same positional-encoding reason every
 * other enum on this wire is. */
#define DBM_LOG_LEVEL_OFF 0
#define DBM_LOG_LEVEL_ERR 1
#define DBM_LOG_LEVEL_WRN 2
#define DBM_LOG_LEVEL_INF 3
#define DBM_LOG_LEVEL_DBG 4

/* Matches `DevBenchMessage`'s variant order exactly; postcard encodes this
 * as the enum's varint discriminant, so the order here must never drift from
 * the crate's.
 *
 * **Tags 2/3/4 changed meaning at schema v8** (`embarch-study-designer`
 * decision 39): the old StreamStart/StreamChunk/StreamEnd trio
 * was retired outright and the generic StreamOpen/StreamChunkBatch/
 * StreamClose trio took their slots. Decision 10's append-only rule is about
 * additions to a shipped protocol; the `Hello`/`HelloAck` version handshake
 * refusing a v7 peer is what makes reusing the slots safe, and no firmware
 * carrying the old shapes was ever flashed. Nothing else moved: Hello,
 * HelloAck, LogLine, StudyStart, StepResult and StudyDone all keep the
 * discriminants they have always had. */
enum dbm_tag {
	DBM_TAG_HELLO = 0,
	DBM_TAG_HELLO_ACK = 1,
	DBM_TAG_STREAM_OPEN = 2,
	DBM_TAG_STREAM_CHUNK_BATCH = 3,
	DBM_TAG_STREAM_CLOSE = 4,
	DBM_TAG_LOG_LINE = 5,
	DBM_TAG_STUDY_START = 6,
	DBM_TAG_STEP_RESULT = 7,
	DBM_TAG_STUDY_DONE = 8,
	/* `GattTranscriptRecord`, tag 10 -- **retired by schema v8**
	 * (`embarch-study-designer` decision 39) and, as of
	 * Milestone 7 Phase B item 3, **no longer sent or encoded**. The
	 * transcript itself is untouched: its entry type, its both-directions
	 * coverage, its uncapped streaming and its `gatt.csv` columns are all
	 * unchanged. An entry now rides as the byte payload of a
	 * DBM_TAG_STREAM_CHUNK_BATCH record, on whichever tap the Study
	 * declared `StreamSource::GattTranscript` for.
	 *
	 * The tag value is left burned rather than reused. Tags 2/3/4 were
	 * reused when the old StreamStart/StreamChunk/StreamEnd trio retired,
	 * and that was safe because no firmware carrying the old shapes had
	 * ever been flashed. That is no longer the argument available here --
	 * this firmware is being flashed -- so 10 stays spent.
	 *
	 * `dbm_encode_transcript_entry` below is the piece that carried
	 * forward: it emits exactly the entry bytes a stream record's payload
	 * holds, and is what the cross-language pinning covers. */
};

/* Mirrors `GattDirection` (embarch-study-designer src/gatt.rs). Append-only,
 * same wire-compatibility rule as `dbm_tag`. */
enum dbm_gatt_direction {
	DBM_GATT_DIR_OUT = 0,
	DBM_GATT_DIR_IN = 1,
	DBM_GATT_DIR_LOCAL = 2,
};

/* Mirrors `GattEventKind` (embarch-study-designer src/gatt.rs). Append-only. */
enum dbm_gatt_event_kind {
	DBM_GATT_EVT_CONNECTED = 0,
	DBM_GATT_EVT_DISCONNECTED = 1,
	DBM_GATT_EVT_DISCOVERY_STARTED = 2,
	DBM_GATT_EVT_SERVICE_DISCOVERED = 3,
	DBM_GATT_EVT_CHARACTERISTIC_DISCOVERED = 4,
	DBM_GATT_EVT_SUBSCRIBED = 5,
	DBM_GATT_EVT_UNSUBSCRIBED = 6,
	DBM_GATT_EVT_WRITE_REQUEST = 7,
	DBM_GATT_EVT_WRITE_RESPONSE = 8,
	DBM_GATT_EVT_READ_REQUEST = 9,
	DBM_GATT_EVT_READ_RESPONSE = 10,
	DBM_GATT_EVT_NOTIFICATION = 11,
	DBM_GATT_EVT_INDICATION = 12,
	DBM_GATT_EVT_ERROR = 13,
};

/* Mirrors `GattTranscriptEntry` (embarch-study-designer src/gatt.rs, §4.3b).
 *
 * `payload` is sized by DBM_MAX_TRANSCRIPT_PAYLOAD_LEN rather than
 * DBM_MAX_PAYLOAD_LEN: the Rust type accepts up to MAX_PAYLOAD_LEN, but this
 * firmware never *produces* an entry larger than one ATT MTU's worth of
 * notification, and a transcript entry lives in a queue with several slots
 * (main.c), where DBM_MAX_PAYLOAD_LEN per slot would cost real RAM this
 * board has already overflowed once (decision 27's own SRAM
 * finding). Sending fewer bytes than the receiving type can hold is always
 * wire-legal; the reverse is not. */
struct dbm_gatt_transcript_entry {
	uint64_t rx_utc_ms;
	uint8_t direction; /* enum dbm_gatt_direction */
	uint8_t kind;      /* enum dbm_gatt_event_kind */
	bool has_service_uuid;
	uint8_t service_uuid[16];
	bool has_characteristic_uuid;
	uint8_t characteristic_uuid[16];
	uint8_t att_status;
	uint16_t payload_len;
	uint8_t payload[DBM_MAX_TRANSCRIPT_PAYLOAD_LEN];
};

/* Mirrors embarch-study-designer's `Unit` (src/sample.rs). Append-only, same
 * wire-compatibility rule as `dbm_tag` above. */
enum dbm_unit {
	DBM_UNIT_MILLIAMPS = 0,
	DBM_UNIT_VOLTS = 1,
	DBM_UNIT_MILLIWATTS = 2,
	DBM_UNIT_RAW = 3,
};

/* `Hello` lost `steps_crc` (moved to `StudyStart` — embarch-study-designer
 * schema v3, `embarch-study-designer` decisions 24/27). */
struct dbm_hello {
	uint32_t schema_version;
	uint64_t host_utc_ms;
};

struct dbm_hello_ack {
	uint32_t schema_version;
	bool compatible;
	/* NUL-terminated; wire form has no NUL, `+1` is this struct's own headroom. */
	char firmware_version[DBM_MAX_FIRMWARE_VERSION_LEN + 1];
	/* This board's own chip ID, hex-encoded lowercase (schema v10,
	 * `embarch-study-designer` decision 47). Core compares it
	 * against the identity its JTAG probe just read, which is the only
	 * thing that ties the runtime serial link and the JTAG connection to
	 * the same silicon -- since the port migration they are physically
	 * separate USB devices.
	 *
	 * Empty string when this build has no `hwinfo` driver: a bench that
	 * cannot answer says nothing rather than sending something. */
	char hardware_id[DBM_MAX_HARDWARE_ID_LEN + 1];
};

/* Mirrors `DevBenchMessage::StreamOpen`/`StreamClose` (embarch-study-designer
 * src/protocol.rs, schema v8). `id` is the tap's own index in
 * `Study.streams` -- the wire handle that replaced the old
 * `step_index` + `channel` pair. When a tap opens is a property of its
 * declared `StreamScope`, not of the wire, which is why neither carries a
 * step index any more. */
struct dbm_stream_open {
	uint8_t id;
};

struct dbm_stream_close {
	uint8_t id;
	/* How many records the producer lost. Carried on close so a stream
	 * that dropped data says so, rather than presenting a shorter,
	 * plausible capture as complete. */
	uint32_t dropped;
};

/* Mirrors `StreamRecord` (embarch-study-designer src/streams.rs, §4.8): one
 * arrival-stamped run of bytes, **never a decoded value**. What the bytes
 * mean is declared once by the tap's `StreamEncoding` and resolved
 * host-side; nothing in this firmware interprets them. */
struct dbm_stream_record {
	uint64_t rx_utc_ms;
	uint8_t bytes[DBM_MAX_STREAM_CHUNK_BYTES];
	uint32_t bytes_len;
};

struct dbm_stream_chunk_batch {
	uint8_t id;
	struct dbm_stream_record records[DBM_MAX_STREAM_RECORDS_PER_BATCH];
	uint32_t records_len;
};

/* Mirrors `StreamSource`'s variant order (embarch-study-designer
 * src/streams.rs). Positional, like every other tag mirror here. */
enum dbm_stream_source_tag {
	DBM_STREAM_SRC_GATT_NOTIFY = 0,
	DBM_STREAM_SRC_POWER_FRONT_END = 1,
	DBM_STREAM_SRC_GATT_TRANSCRIPT = 2,
	DBM_STREAM_SRC_DEV_BENCH_LOG = 3,
	DBM_STREAM_SRC_SIGNAL = 4,
};

/* Mirrors `StreamScope` (embarch-study-designer src/streams.rs). */
enum dbm_stream_scope_tag {
	DBM_STREAM_SCOPE_WHOLE_STUDY = 0,
	DBM_STREAM_SCOPE_STEPS = 1,
};

/* One declared tap, as much of it as **this node acts on** -- schema v9's
 * `StreamTap` (embarch-study-designer src/streams.rs, §4.8), deliberately
 * not mirrored in full.
 *
 * `StreamTap` declares four things: where the bytes come from, how long the
 * tap lives, how to render what arrives, and what to call the output. Only
 * the first two are dev-bench's business. `encoding` is what a payload
 * *means*, and decision 39's entire premise is that dev-bench stamps arrival
 * and interprets nothing -- storing an encoding here would be storing the
 * one thing this firmware exists not to know. `name` names a file in Core's
 * results directory, which this node never sees.
 *
 * So this is 12 bytes rather than the ~80 a full mirror would cost, and the
 * decoder still *walks* the fields it doesn't keep (it has to: postcard
 * carries no per-field length, so skipping to `scope` means parsing `name`
 * and `encoding` whether or not you keep them).
 *
 * `source_tag` is kept, though dev-bench interprets nothing, because it is
 * not about meaning: it says which node produces the bytes. A
 * DBM_STREAM_SRC_SIGNAL tap is Core's to open (`StreamSource::
 * is_dev_bench_mediated`), and dev-bench must not open it or Core would see
 * two producers on one id. */
struct dbm_stream_tap {
	uint8_t id;
	uint8_t source_tag; /* enum dbm_stream_source_tag */
	uint8_t scope_tag;  /* enum dbm_stream_scope_tag */
	/* Both inclusive, and meaningful only for DBM_STREAM_SCOPE_STEPS. */
	uint32_t scope_from;
	uint32_t scope_to;
	/* The characteristic a DBM_STREAM_SRC_GATT_NOTIFY tap routes, raw
	 * big-endian; meaningless for every other source tag
	 * (`embarch-study-designer` decision 55, schema v14).
	 * Previously the decoder skipped these 32
	 * bytes outright.
	 *
	 * **This is addressing, not meaning, and the distinction is the whole
	 * reason it is allowed here.** `encoding` stays deliberately unkept --
	 * what a payload *means* is the knowledge decision 39 took away from
	 * this firmware, and it still holds none of it. Which characteristic's
	 * notifications go to which tap id is routing, exactly like
	 * `source_tag`: this node has to know it because this node is the one
	 * that sends the record.
	 *
	 * The service UUID is skipped rather than kept: routing matches on the
	 * characteristic, which is what a notification callback identifies
	 * itself by, and a second 16-byte array per tap would be 128 bytes of
	 * static RAM for a comparison nothing makes. */
	uint8_t characteristic_uuid[16];
};

/* Mirrors `StreamScope::covers` (embarch-study-designer src/streams.rs) --
 * whether this tap is open while `step_index` runs. `to` is inclusive; a
 * single-step window is `from == to`. */
static inline bool dbm_stream_tap_covers(const struct dbm_stream_tap *tap, uint32_t step_index)
{
	if (tap->scope_tag == DBM_STREAM_SCOPE_WHOLE_STUDY) {
		return true;
	}
	return step_index >= tap->scope_from && step_index <= tap->scope_to;
}

/* Whether dev-bench is the node that produces this tap's bytes. Mirrors
 * `StreamSource::is_dev_bench_mediated`: everything except a Signal tap,
 * which Core opens on its own carrier. */
static inline bool dbm_stream_tap_is_ours(const struct dbm_stream_tap *tap)
{
	return tap->source_tag != DBM_STREAM_SRC_SIGNAL;
}

struct dbm_log_line {
	char text[DBM_MAX_LOG_LINE_LEN + 1];
};

/* Mirrors embarch-study-designer's `Action` (src/study.rs), one variant per
 * enum tag -- `embarch-study-designer` decisions 44/50 added the
 * last two of these.
 * Append-only, same discipline as `enum dbm_tag`: the tag values below match
 * `Action`'s own declared variant order exactly, since postcard encodes an
 * enum discriminant positionally. */
enum dbm_action_tag {
	DBM_ACTION_BLE_ADVERTISE = 0,
	DBM_ACTION_BLE_CONNECT = 1,
	DBM_ACTION_DATA_EXCHANGE = 2,
	DBM_ACTION_GATT_DISCOVER = 3,
	DBM_ACTION_GATT_MONITOR_ALL = 4,
	/* `embarch-study-designer` decision 36 -- a capture window that outlives its own
	 * step, so a stimulus write and a capture can finally overlap. Both
	 * field-less, same as GattDiscover/GattMonitorAll. */
	DBM_ACTION_GATT_MONITOR_START = 5,
	DBM_ACTION_GATT_MONITOR_STOP = 6,
	/* `embarch-study-designer` decisions 44/50, schema v12 --
	 * the first Action variant since BleConnect to carry a field, and one
	 * that carries none. Appended, never inserted: postcard encodes the
	 * discriminant positionally, so inserting would shift both of these
	 * and every future one. */
	DBM_ACTION_BLE_SECURITY = 7,
	DBM_ACTION_BLE_UNBOND = 8,
	/* `embarch-study-designer` decision 53, schema v14 -- the
	 * same discovery-and-subscribe walk as GATT_MONITOR_ALL/START, narrowed
	 * to the characteristics the study names.
	 *
	 * These are the first Action variants ever to carry a *sequence*, which
	 * matters more to this decoder than it looks: every monitor action
	 * before them was field-less, so a decoder that walked one of these as
	 * field-less would read the target-count varint as the next step's name
	 * length and produce a step list that still decodes, into nonsense. The
	 * v14 wire vector in the crate's tests/firmware_test_vectors.rs exists
	 * for exactly that. */
	DBM_ACTION_GATT_MONITOR_SELECTED = 9,
	DBM_ACTION_GATT_MONITOR_SELECTED_START = 10,
	/* `embarch-study-designer` decision 60, schema v15 -- hand
	 * the link to a declared `.eap` state machine for the length of this
	 * step. Appended, never inserted, same positional-encoding rule as
	 * every tag above it.
	 *
	 * **This is the first Action this firmware does not merely dispatch but
	 * interprets.** Every other tag here names a fixed thing ble_bridge
	 * knows how to do; this one names an index into `StudyStart.protocols`
	 * and a state to enter, and what happens next is whatever the manifest
	 * says. That is the cost decision 60 accepted by name: what a payload
	 * *means* now reaches this node, which is the knowledge decision 39
	 * took away from it. Decision 59's split is what keeps that cost to the
	 * primitives a running machine can actually reach -- `repeat`,
	 * `bitpack`, `crc32` and `fixed` stay host-side and never arrive
	 * here. */
	DBM_ACTION_RUN_PROTOCOL = 11,
};

/* Mirrors `Action::RunProtocol { protocol, entry_state }`. Both are indices,
 * never names. Both reach a C array subscript here, which is why
 * `embarch_study_designer::eap::validate_protocol` and Core's own pre-flight
 * range-check them before a `Study` is ever submitted -- and why this
 * firmware re-checks them anyway at dispatch (§3 decision 18's rule: name the
 * specific failure rather than letting a raw index fail). */
struct dbm_run_protocol_action {
	uint8_t protocol;
	uint8_t entry_state;
};

/* How many characteristics one selective monitor step may name -- mirrors
 * embarch-study-designer's limits::MAX_MONITOR_TARGETS. At the crate's full
 * value: refusing a count Core considers legal would be this firmware
 * inventing a limit, and this array costs nothing extra in practice because
 * `struct dbm_step`'s union is already sized by `dbm_data_exchange_action`
 * (~553 bytes), which is larger than 16 targets (512). */
#define DBM_MAX_MONITOR_TARGETS 16

/* Mirrors `SecurityLevel` (embarch-study-designer/src/study.rs,
 * decision 44). Values are postcard discriminants -- the enum's *declaration*
 * order, which is NOT the spec's level number: L1 encodes as 0. That offset
 * is the whole reason this mirror is spelled out rather than assumed, and
 * `dbm_security_level_number()` below is the one place it is undone.
 *
 * L1 is carried because `StepResult.security_level` reports it (a link that
 * never got encrypted is exactly the answer a study debugging a
 * security-requiring DUT needs); no study may *request* it, and
 * ble_bridge_real.c fails such a step rather than treating it as a no-op. */
enum dbm_security_level {
	DBM_SECURITY_L1 = 0,
	DBM_SECURITY_L2 = 1,
	DBM_SECURITY_L3 = 2,
	DBM_SECURITY_L4 = 3,
};

/* The Bluetooth spec's own level number for a `enum dbm_security_level` --
 * 1 for DBM_SECURITY_L1, and so on. The one place the declaration-order/
 * level-number offset is undone, so no log line or fail_reason computes it
 * itself. */
static inline uint8_t dbm_security_level_number(uint8_t level)
{
	return (uint8_t)(level + 1);
}

/* Mirrors the FFI-side EssdBleAdvertiseAction shape 1:1 -- see study_ffi.h. */
struct dbm_ble_advertise_action {
	char local_name[DBM_MAX_LOCAL_NAME_LEN + 1]; /* NUL-terminated */
	bool has_local_name;
	uint16_t adv_interval_ms;
};

/* Mirrors `Action::BleConnect` (src/study.rs) -- `target_address` bytes are
 * in embarch-study-designer's own display order (§4.3: most
 * significant byte first), same convention `ble_bridge.h`'s
 * `struct ble_connect_params` already documents. */
struct dbm_ble_connect_action {
	uint8_t role; /* 0 = Central, 1 = Peripheral (mirrors BleRole) */
	bool has_target_address;
	uint8_t target_address_kind; /* 0 = Public, 1 = Random (mirrors BleAddressKind) */
	uint8_t target_address[6];
	/* Advertised local name to connect to (`embarch-study-designer`
	 * decision 43, schema v7). Encoded last in the BleConnect variant, so
	 * decoded last here. `has_target_name == false` means no name filter --
	 * connect to whichever connectable peripheral advertises first, the
	 * pre-v7 behavior. */
	bool has_target_name;
	char target_name[DBM_MAX_LOCAL_NAME_LEN + 1]; /* NUL-terminated */
};

/* Mirrors `GattOperation` (src/study.rs). `payload`/`payload_len` are valid
 * only when `kind == DBM_GATT_OP_WRITE`; `timeout_ms` only for
 * `DBM_GATT_OP_NOTIFY`/`DBM_GATT_OP_INDICATE`. */
enum dbm_gatt_op_kind {
	DBM_GATT_OP_READ = 0,
	DBM_GATT_OP_WRITE = 1,
	DBM_GATT_OP_NOTIFY = 2,
	DBM_GATT_OP_INDICATE = 3,
	DBM_GATT_OP_SUBSCRIBE = 4,
	DBM_GATT_OP_STREAM_CAPTURE = 5,
};

struct dbm_gatt_operation {
	uint8_t kind;
	uint8_t payload[DBM_MAX_PAYLOAD_LEN];
	uint32_t payload_len;
	uint32_t timeout_ms;
};

/* Mirrors `Action::DataExchange` (src/study.rs) -- UUIDs are raw big-endian
 * bytes, same convention as everywhere else in this header. */
struct dbm_data_exchange_action {
	uint8_t service_uuid[16];
	uint8_t characteristic_uuid[16];
	struct dbm_gatt_operation operation;
};

/* Mirrors `Action::BleSecurity` (embarch-study-designer/src/study.rs,
 * decision 44). One field, encoded as a varint after the action
 * tag. `Action::BleUnbond` (`embarch-study-designer` decision 50) is field-less and needs no
 * struct,
 * same as the GattDiscover/GattMonitor* family below. */
struct dbm_ble_set_security_action {
	uint8_t level; /* enum dbm_security_level */
};

/* Mirrors `GattTarget` (embarch-study-designer/src/gatt.rs,
 * decision 53) -- two raw big-endian UUIDs back to back, no length prefixes,
 * same convention as every other UUID in this header. Both, not the
 * characteristic alone: subscribing needs the service to discover within,
 * exactly as DataExchange has always needed both. */
struct dbm_gatt_target {
	uint8_t service_uuid[16];
	uint8_t characteristic_uuid[16];
};

/* Mirrors `Action::GattMonitorSelected`/`GattMonitorSelectedStart`
 * (`embarch-study-designer` decision 53). One field, a sequence -- so the decoder reads
 * a length varint and then that many fixed 32-byte targets. */
struct dbm_gatt_monitor_selected_action {
	struct dbm_gatt_target targets[DBM_MAX_MONITOR_TARGETS];
	uint32_t targets_len;
};

/* Action::GattDiscover/GattMonitorAll (`embarch-study-designer` decisions 31/32) are both
 * field-less -- no struct needed; `dbm_step.action_tag` alone identifies
 * them, matching this crate's "simplest possible FFI/wire surface" framing
 * for both. */

struct dbm_step {
	char name[DBM_MAX_NAME_LEN + 1];
	uint32_t timeout_ms;
	bool continue_on_fail;
	/* How long to wait before starting this step's action
	 * (`embarch-study-designer` decision 42, schema v6). Encoded
	 * last in `Step`, so it is read last here too -- see the crate's own
	 * `Step::delay_before_ms` doc comment for why it was appended rather
	 * than inserted. Distinct from `timeout_ms`, which still bounds only
	 * the action itself. */
	uint32_t delay_before_ms;
	uint8_t action_tag; /* enum dbm_action_tag */
	union {
		struct dbm_ble_advertise_action advertise;
		struct dbm_ble_connect_action connect;
		struct dbm_data_exchange_action data_exchange;
		struct dbm_ble_set_security_action set_security;
		struct dbm_gatt_monitor_selected_action monitor_selected;
		struct dbm_run_protocol_action run_protocol;
		/* DBM_ACTION_GATT_DISCOVER/DBM_ACTION_GATT_MONITOR_ALL/
		 * DBM_ACTION_GATT_MONITOR_START/DBM_ACTION_GATT_MONITOR_STOP/
		 * DBM_ACTION_BLE_UNBOND carry no fields of their own. */
	} action;
};

struct dbm_study_start {
	uint32_t steps_len;
	struct dbm_step steps[DBM_MAX_STEPS_PER_STUDY];
	uint32_t steps_crc;
	/* Not part of the wire format -- set by dbm_decode_frame itself
	 * (`embarch-study-designer` decision 17/19): whether the just-decoded `steps`
	 * recompute to `steps_crc`. `false` on a genuine mismatch; also `false`
	 * (with `steps_len` left at 0) when decoding stopped early because a
	 * step's action wasn't BleAdvertise -- see dbm_decode_frame's own doc
	 * comment for why CRC can't be computed in that case. */
	bool steps_crc_valid;
	/* The declared `StreamTap`s (schema v9, `embarch-study-designer` decision 39 and
	 * its 2026-08-25 amendment), as much of each as this node acts on --
	 * see `struct dbm_stream_tap` for what is deliberately not kept.
	 *
	 * Stored as of Milestone 7 Phase B item 3. Before that the decoder
	 * walked and discarded them, because walking is what locates the span
	 * `streams_crc` covers and nothing here opened a tap; the walker that
	 * pass added is exactly what this one needed, which is why item 0's
	 * +654 bytes was that cost arriving early rather than waste. */
	struct dbm_stream_tap streams[DBM_MAX_STREAMS_PER_STUDY];
	uint32_t streams_len;
	uint32_t streams_crc;
	/* Not part of the wire format, same as `steps_crc_valid` above:
	 * whether the walked `streams` span recomputes to `streams_crc`.
	 * Checked **independently** of `steps_crc_valid`, which is why decision
	 * 39's amendment chose a sibling seal over a widened one -- a mismatch
	 * says which half of a `Study` is corrupt. Also `false` when decoding
	 * stopped early on an unsupported action, for the same reason
	 * `steps_crc_valid` is. */
	bool streams_crc_valid;
	/* Set when a step's Action isn't BleAdvertise (decision 21's initial
	 * scope) -- decode still returns 0 (a well-formed StudyStart arrived),
	 * but the caller must not dispatch it. */
	bool has_unsupported_action;
	/* How loud this firmware should be while this study runs -- schema v13
	 * (`embarch-study-designer` decision 39), one of DBM_LOG_LEVEL_*. Appended after
	 * `streams_crc` on the wire, and covered by **neither** seal: the two
	 * CRCs cover what dev-bench executes and what it captures, and how
	 * verbose it is about doing so changes neither. */
	uint8_t dev_bench_log_level;
	/* The `.eap` protocol manifests this study resolved at build time --
	 * schema v15 (`embarch-study-designer` decision 58, §4.9),
	 * appended after `dev_bench_log_level` on the wire.
	 *
	 * Unlike `streams`, which this node stores 12 bytes of and walks past
	 * the rest, a protocol is stored **whole minus its names**: dev-bench
	 * executes this one (decision 60), so every index, offset, guard and
	 * write template has to survive the decode. */
	struct eap_protocol_def protocols[DBM_MAX_PROTOCOLS_PER_STUDY];
	uint32_t protocols_len;
	uint32_t protocols_crc;
	/* Not part of the wire format, same as `steps_crc_valid`/
	 * `streams_crc_valid`: whether the decoded `protocols` span recomputes
	 * to `protocols_crc`. The **third** seal, checked independently of the
	 * other two so a mismatch names which of the three is corrupt -- which
	 * is the whole reason there are three sibling seals rather than one
	 * widened one. Also `false` when decoding stopped early on an
	 * unsupported action, for the same reason its two siblings are. */
	bool protocols_crc_valid;
};

struct dbm_outcome {
	uint8_t tag; /* 0=Pass, 1=Fail, 2=TimedOut */
	char fail_reason[DBM_MAX_FAIL_REASON_LEN + 1]; /* valid only if tag == 1 */
};

/* Mirrors `GattCharacteristicInfo`/`GattServiceInfo` (embarch-study-designer/
 * src/gatt.rs, §4.3a) -- `properties` is the raw ATT
 * characteristic-properties byte, passed through unchanged. */
struct dbm_gatt_characteristic_info {
	uint8_t uuid[16];
	uint8_t properties;
};

struct dbm_gatt_service_info {
	uint8_t uuid[16];
	struct dbm_gatt_characteristic_info characteristics[DBM_MAX_CHARS_PER_SERVICE];
	uint32_t characteristics_len;
};

struct dbm_step_result_payload {
	char step_name[DBM_MAX_NAME_LEN + 1];
	struct dbm_outcome outcome;
	bool has_captured_data;
	uint8_t captured_data[DBM_MAX_PAYLOAD_LEN];
	uint32_t captured_data_len;
	/* `power_samples_ref`/`waveform_ref` were described here as two
	 * permanently-None wire fields. They are **retired** from `StepResult`
	 * by `embarch-study-designer` decision 39 (schema v8) and the two bytes this file
	 * kept writing for them are gone at v9 -- see serial_protocol.c's
	 * StepResult encoder for how they outlived the fields. */
	/* gatt_services (`embarch-study-designer` decisions 31/32): populated by every
	 * discovering action; encode_body/decode_body both handle a real Some.
	 *
	 * `gatt_activity` was here and is **retired** at schema v14
	 * (`embarch-study-designer` decision 54), taking
	 * `struct dbm_gatt_activity_record` and DBM_MAX_GATT_ACTIVITY_RECORDS
	 * with it. It was 32 × 526 bytes of static RAM -- by a wide margin the
	 * largest single contributor to this struct -- holding a capped copy of
	 * something the tap pipeline already streams to a file uncapped. */
	bool has_gatt_services;
	struct dbm_gatt_service_info gatt_services[DBM_MAX_DISCOVERED_SERVICES];
	uint32_t gatt_services_len;
	/* `StepResult.security_level: Option<SecurityLevel>` -- schema v12's
	 * trailing field (`embarch-study-designer` decision 44).
	 * Encoded last, so this is one appended Option byte rather than a
	 * re-shuffle of a message this firmware sends more than any other.
	 *
	 * Populated for *every* step, not only a security one: whichever level
	 * the link happened to be at is what makes a later step's failure
	 * legible, and `false` here means "there was no connection to ask
	 * about", never "nobody looked". */
	bool has_security_level;
	uint8_t security_level; /* enum dbm_security_level */
	/* `StepResult.protocol: Option<ProtocolOutcome>` -- schema v15's
	 * trailing field (`embarch-study-designer` decision 62),
	 * and now the last field of StepResult. Appended, so this is one more
	 * Option byte on the message this firmware sends most rather than a
	 * re-shuffle of it.
	 *
	 * **Two fields, and the state name is the one the tap cannot say.**
	 * There is deliberately no list of decoded values here: that is the
	 * shape decision 54 retired `gatt_activity` for, and decoded bytes
	 * reach a reader through the tap the study declared, rendered
	 * host-side. Whether `final_state` was a terminal state is not stored
	 * either -- it is a lookup in the `ProtocolDef` the `Study` already
	 * carries, and a stored copy could disagree with it.
	 *
	 * `protocol_outcome` is a full `struct dbm_outcome` rather than a
	 * duplicate of the step's own: the step can fail for a reason the
	 * machine never saw (a dropped link, an exhausted step timeout), and
	 * telling "the protocol reached its `failed` state" apart from "the
	 * protocol never finished" is the whole diagnostic value of recording
	 * a final state. */
	bool has_protocol;
	char protocol_final_state[EAP_MAX_STATE_NAME_LEN + 1];
	struct dbm_outcome protocol_outcome;
};

struct dbm_step_result {
	uint32_t step_index;
	struct dbm_step_result_payload result;
};

struct dbm_study_done {
	bool completed;
};

struct dev_bench_message {
	enum dbm_tag tag;
	union {
		struct dbm_hello hello;
		struct dbm_hello_ack hello_ack;
		struct dbm_stream_open stream_open;
		struct dbm_stream_chunk_batch stream_chunk_batch;
		struct dbm_stream_close stream_close;
		struct dbm_log_line log_line;
		struct dbm_study_start study_start;
		struct dbm_step_result step_result;
		struct dbm_study_done study_done;
	};
};

/* Encodes one `GattTranscriptEntry` (embarch-study-designer src/gatt.rs,
 * §4.3b) as bare postcard bytes -- no message tag, no COBS framing, no
 * step index. Writes at most `out_cap` bytes to `out` and returns the length
 * written, or a negative value if it wouldn't fit.
 *
 * Split out at schema v8 (`embarch-study-designer` decision 39): these are exactly the
 * bytes a `StreamChunkBatch` record's payload carries on a tap declared
 * `StreamEncoding::GattTranscript`, which is what the transcript became once
 * its own message class was retired. Exposed (rather than left static)
 * because it is the half of the retired encoder that survives, and because
 * the cross-language wire pinning asserts against it directly. */
int dbm_encode_transcript_entry(const struct dbm_gatt_transcript_entry *entry, uint8_t *out,
				 size_t out_cap);

/* Encodes `msg` as postcard bytes wrapped in a COBS frame, including the
 * trailing 0x00 delimiter. Returns the frame length (> 0) on success, or a
 * negative value if `out_cap` is too small for the encoded frame. */
int dbm_encode_frame(const struct dev_bench_message *msg, uint8_t *out, size_t out_cap);

/* Decodes one COBS-stuffed frame into `out`. `in`/`in_len` must be exactly one
 * frame's stuffed bytes, WITHOUT the trailing 0x00 delimiter — the caller (the
 * UART RX loop, which scans for the 0x00 frame boundary) strips it first, so
 * this function stays testable without any I/O involved. Returns 0 on
 * success; a negative value on a malformed frame, unknown tag, truncated
 * message, or a string field too long for its buffer.
 *
 * For DBM_TAG_STUDY_START specifically (`embarch-study-designer`
 * decisions 17/19/31/32, decision 21): every
 * step's Action kind this crate defines is decodable now
 * (BleAdvertise/BleConnect/DataExchange/GattDiscover/GattMonitorAll) —
 * decision 21's original BleAdvertise-only scope is closed. `steps_len`
 * beyond DBM_MAX_STEPS_PER_STUDY (a dev-bench-internal capacity cap, see
 * that constant's own doc comment — smaller than the crate's own
 * limits::MAX_STEPS_PER_STUDY) still stops decoding immediately and returns
 * -1 rather than reading out of bounds. Encountering an action tag this
 * decoder doesn't recognize at all (a future crate-side append this
 * firmware predates) still sets `out->study_start.has_unsupported_action`
 * and stops decoding that step, same fallback behavior decision 21
 * originally established, just no longer reachable for any of today's five
 * kinds. Otherwise `steps_crc_valid` reports whether the CRC-32 (ISO-HDLC)
 * computed over the raw wire bytes of the decoded `steps` matches the
 * `steps_crc` field also received on the wire — independently reproducing
 * embarch-study-designer's own `steps_crc()` (src/crc.rs), which hashes each
 * step's postcard encoding concatenated in order; since postcard's encoding
 * is canonical, hashing each step's original as-received bytes is bit-for-bit
 * identical to hashing a fresh re-encoding of the same decoded value, so this
 * needs no FFI round-trip into embarch-study-designer to get the identical
 * answer (see study_ffi.h's own note on why study_ffi_decode_study --
 * decoding a `Study`, not a `StudyStart` -- doesn't fit this call site). */
int dbm_decode_frame(const uint8_t *in, size_t in_len, struct dev_bench_message *out);

#endif /* EMBARCH_DEV_BENCH_SERIAL_PROTOCOL_H_ */
