/* DevBenchMessage wire protocol: COBS framing + postcard-compatible encoding.
 *
 * embarch-dev-bench/design.md §2, §3 decisions 7/10/12/20. Mirrors
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
 * hex-encoded (embarch-study-designer/design.md §3 decision 47,
 * embarch-core/design.md §3 decision 35). */
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
 * a real build at 64 slots would not fit; see design.md's own changelog for
 * the measured numbers). 16 is sized well above every `Study` this suite
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
 * MAX_CHARS_PER_SERVICE (design.md §3 decisions 31/32/33's update) --
 * these two, unlike DBM_MAX_STEPS_PER_STUDY above, are small enough that
 * mirroring the crate's own real ceiling costs no meaningful RAM. */
#define DBM_MAX_DISCOVERED_SERVICES 8
#define DBM_MAX_CHARS_PER_SERVICE 16
/* Mirrors limits::MAX_GATT_ACTIVITY_RECORDS -- the single largest
 * contributor to `struct dbm_step_result_payload`'s size (32 records at up
 * to DBM_MAX_PAYLOAD_LEN bytes each), flagged as a real stack/static-RAM
 * risk from the outset by embarch-study-designer/design.md §3 decision 32's
 * own text -- kept at the crate's full ceiling rather than shrunk further
 * (unlike DBM_MAX_STEPS_PER_STUDY above) so this milestone's own "a normal
 * acquisition window, nothing dropped" validation isn't manufactured into
 * an overflow case by an artificially small dev-bench-side cap. */
#define DBM_MAX_GATT_ACTIVITY_RECORDS 32
/* Largest transcript-entry payload this firmware ever *produces* (design.md
 * §3 decision 36) -- one ATT MTU's worth of notification, not the crate's
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
 * nothing is stored per tap here yet -- this bounds how many the decoder is
 * willing to *walk* while locating the span `streams_crc` covers -- so
 * mirroring the crate's real ceiling costs no RAM at all, and refusing a tap
 * count Core considers legal would be this firmware inventing a limit. */
#define DBM_MAX_STREAMS_PER_STUDY 8

/* Largest single postcard-encoded (pre-COBS) DevBenchMessage this firmware sends/receives.
 *
 * StudyStart's per-step worst case grew once decode had to cover every
 * `Action` kind, not just BleAdvertise (design.md §3 decisions 31/32):
 * `DataExchange { GattOperation::Write { payload } }` dominates a single
 * step's own encoding now (service_uuid+characteristic_uuid, 16 bytes each,
 * plus a Write payload up to DBM_MAX_PAYLOAD_LEN bytes with its own length
 * varint), well past BleAdvertise's own ~30 bytes -- generously rounded up
 * to DBM_MAX_PAYLOAD_LEN + 64 bytes/step to cover it plus every other
 * per-step field with margin. GattDiscover/GattMonitorAll are both
 * field-less and add only their own action tag varint, cheaper than
 * BleAdvertise, so they don't move this bound.
 *
 * StepResult's own worst case grew independently: `gatt_services` (up to
 * DBM_MAX_DISCOVERED_SERVICES services, each up to DBM_MAX_CHARS_PER_SERVICE
 * characteristics at 17 bytes each) plus `gatt_activity` (up to
 * DBM_MAX_GATT_ACTIVITY_RECORDS records, each up to DBM_MAX_PAYLOAD_LEN
 * bytes) now dominates over `captured_data` alone -- this is what makes
 * StepResult, not StudyStart, this file's actual largest message once both
 * are computed for real (see the constants immediately below).
 */
#define DBM_MAX_STUDY_START_LEN \
	(8 + (DBM_MAX_STEPS_PER_STUDY * (DBM_MAX_PAYLOAD_LEN + 64)) + 8)
#define DBM_MAX_GATT_SERVICES_LEN \
	(4 + (DBM_MAX_DISCOVERED_SERVICES * (16 + 4 + (DBM_MAX_CHARS_PER_SERVICE * 17))))
#define DBM_MAX_GATT_ACTIVITY_LEN \
	(4 + (DBM_MAX_GATT_ACTIVITY_RECORDS * (8 + 2 + 4 + DBM_MAX_PAYLOAD_LEN)))
#define DBM_MAX_STEP_RESULT_LEN                                                                  \
	(24 + DBM_MAX_NAME_LEN + DBM_MAX_FAIL_REASON_LEN + DBM_MAX_PAYLOAD_LEN +                  \
	 DBM_MAX_GATT_SERVICES_LEN + DBM_MAX_GATT_ACTIVITY_LEN)
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

/* Matches `DevBenchMessage`'s variant order exactly; postcard encodes this
 * as the enum's varint discriminant, so the order here must never drift from
 * the crate's.
 *
 * **Tags 2/3/4 changed meaning at schema v8** (embarch-study-designer/
 * design.md §3 decision 39): the old StreamStart/StreamChunk/StreamEnd trio
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
	 * (embarch-study-designer/design.md §3 decision 39). The transcript
	 * itself survives untouched: its entry type, its both-directions
	 * coverage, its uncapped streaming and its `gatt.csv` columns are all
	 * unchanged, and an entry now rides as the byte payload of a
	 * DBM_TAG_STREAM_CHUNK_BATCH record on a tap declared
	 * `StreamEncoding::GattTranscript`.
	 *
	 * The tag and its encoder are still here, and main.c still sends it,
	 * because rewiring that send needs the tap `id` from
	 * `StudyStart.streams` -- which this firmware does not decode yet.
	 * That is Milestone 7 Phase B's work (embarch-doc's
	 * embarch-outpost/milestone-1.md §3), deliberately not started here.
	 * Until it lands, this firmware emits a message the v8 Rust decoder
	 * has no variant for. It has never been flashed, which is what makes
	 * that survivable rather than an outage.
	 *
	 * `dbm_encode_transcript_entry` below is the piece that carries
	 * forward: it emits exactly the entry bytes a stream record's payload
	 * holds, and is what the cross-language pinning now covers. */
	DBM_TAG_GATT_TRANSCRIPT_RECORD = 10,
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
 * board has already overflowed once (design.md §3 decision 27's own SRAM
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

struct dbm_gatt_transcript_record {
	uint32_t step_index;
	struct dbm_gatt_transcript_entry entry;
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
 * schema v3, embarch-study-designer/design.md §3 decisions 24/27). */
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
	 * embarch-study-designer/design.md §3 decision 47). Core compares it
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

struct dbm_log_line {
	char text[DBM_MAX_LOG_LINE_LEN + 1];
};

/* Mirrors embarch-study-designer's `Action` (src/study.rs), one variant per
 * enum tag -- design.md §3 decisions 31/32 added the last two of these.
 * Append-only, same discipline as `enum dbm_tag`: the tag values below match
 * `Action`'s own declared variant order exactly, since postcard encodes an
 * enum discriminant positionally. */
enum dbm_action_tag {
	DBM_ACTION_BLE_ADVERTISE = 0,
	DBM_ACTION_BLE_CONNECT = 1,
	DBM_ACTION_DATA_EXCHANGE = 2,
	DBM_ACTION_GATT_DISCOVER = 3,
	DBM_ACTION_GATT_MONITOR_ALL = 4,
	/* design.md §3 decision 36 -- a capture window that outlives its own
	 * step, so a stimulus write and a capture can finally overlap. Both
	 * field-less, same as GattDiscover/GattMonitorAll. */
	DBM_ACTION_GATT_MONITOR_START = 5,
	DBM_ACTION_GATT_MONITOR_STOP = 6,
};

/* Mirrors the FFI-side EssdBleAdvertiseAction shape 1:1 -- see study_ffi.h. */
struct dbm_ble_advertise_action {
	char local_name[DBM_MAX_LOCAL_NAME_LEN + 1]; /* NUL-terminated */
	bool has_local_name;
	uint16_t adv_interval_ms;
};

/* Mirrors `Action::BleConnect` (src/study.rs) -- `target_address` bytes are
 * in embarch-study-designer's own display order (design.md §4.3: most
 * significant byte first), same convention `ble_bridge.h`'s
 * `struct ble_connect_params` already documents. */
struct dbm_ble_connect_action {
	uint8_t role; /* 0 = Central, 1 = Peripheral (mirrors BleRole) */
	bool has_target_address;
	uint8_t target_address_kind; /* 0 = Public, 1 = Random (mirrors BleAddressKind) */
	uint8_t target_address[6];
	/* Advertised local name to connect to (embarch-study-designer/design.md
	 * §3 decision 43, schema v7). Encoded last in the BleConnect variant, so
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

/* Action::GattDiscover/GattMonitorAll (design.md §3 decisions 31/32) are both
 * field-less -- no struct needed; `dbm_step.action_tag` alone identifies
 * them, matching this crate's "simplest possible FFI/wire surface" framing
 * for both. */

struct dbm_step {
	char name[DBM_MAX_NAME_LEN + 1];
	uint32_t timeout_ms;
	bool continue_on_fail;
	/* How long to wait before starting this step's action
	 * (embarch-study-designer/design.md §3 decision 42, schema v6). Encoded
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
		/* DBM_ACTION_GATT_DISCOVER/DBM_ACTION_GATT_MONITOR_ALL carry no
		 * fields of their own. */
	} action;
};

struct dbm_study_start {
	uint32_t steps_len;
	struct dbm_step steps[DBM_MAX_STEPS_PER_STUDY];
	uint32_t steps_crc;
	/* Not part of the wire format -- set by dbm_decode_frame itself
	 * (design.md §3 decision 17/19): whether the just-decoded `steps`
	 * recompute to `steps_crc`. `false` on a genuine mismatch; also `false`
	 * (with `steps_len` left at 0) when decoding stopped early because a
	 * step's action wasn't BleAdvertise -- see dbm_decode_frame's own doc
	 * comment for why CRC can't be computed in that case. */
	bool steps_crc_valid;
	/* How many `StreamTap`s the decoder walked past (schema v9, design.md
	 * §3 decision 39 and its 2026-08-25 amendment). The taps themselves are
	 * **not stored**: this firmware doesn't open them yet, and walking them
	 * is what locates the span `streams_crc` covers -- the same walk-and-
	 * discard this decoder already does for `BleAdvertise::service_uuids`.
	 * Opening taps for real is Milestone 7 Phase B item 3's, and that is
	 * when this grows a `streams[]` array. */
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
};

struct dbm_outcome {
	uint8_t tag; /* 0=Pass, 1=Fail, 2=TimedOut */
	char fail_reason[DBM_MAX_FAIL_REASON_LEN + 1]; /* valid only if tag == 1 */
};

/* Mirrors `GattCharacteristicInfo`/`GattServiceInfo` (embarch-study-designer/
 * src/gatt.rs, design.md §4.3a) -- `properties` is the raw ATT
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

/* Mirrors `GattActivityRecord` -- `characteristic_index` indexes into this
 * same StepResult's `gatt_services`, flattened service-then-characteristic
 * in discovery order (that type's own documented convention, design.md
 * §4.3a). */
struct dbm_gatt_activity_record {
	uint64_t rx_utc_ms;
	uint16_t characteristic_index;
	uint8_t payload[DBM_MAX_PAYLOAD_LEN];
	uint32_t payload_len;
};

struct dbm_step_result_payload {
	char step_name[DBM_MAX_NAME_LEN + 1];
	struct dbm_outcome outcome;
	bool has_captured_data;
	uint8_t captured_data[DBM_MAX_PAYLOAD_LEN];
	uint32_t captured_data_len;
	/* `power_samples_ref`/`waveform_ref` were described here as two
	 * permanently-None wire fields. They are **retired** from `StepResult`
	 * by design.md §3 decision 39 (schema v8) and the two bytes this file
	 * kept writing for them are gone at v9 -- see serial_protocol.c's
	 * StepResult encoder for how they outlived the fields. */
	/* gatt_services/gatt_activity (design.md §3 decisions 31/32): populated
	 * by GattDiscover (services only) and GattMonitorAll (both);
	 * encode_body/decode_body both handle a real Some. */
	bool has_gatt_services;
	struct dbm_gatt_service_info gatt_services[DBM_MAX_DISCOVERED_SERVICES];
	uint32_t gatt_services_len;
	bool has_gatt_activity;
	struct dbm_gatt_activity_record gatt_activity[DBM_MAX_GATT_ACTIVITY_RECORDS];
	uint32_t gatt_activity_len;
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
		struct dbm_gatt_transcript_record gatt_transcript;
	};
};

/* Encodes one `GattTranscriptEntry` (embarch-study-designer src/gatt.rs,
 * §4.3b) as bare postcard bytes -- no message tag, no COBS framing, no
 * step index. Writes at most `out_cap` bytes to `out` and returns the length
 * written, or a negative value if it wouldn't fit.
 *
 * Split out at schema v8 (design.md §3 decision 39): these are exactly the
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
 * For DBM_TAG_STUDY_START specifically (embarch-study-designer/design.md §3
 * decisions 17/19/31/32, embarch-dev-bench/design.md §3 decision 21): every
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
