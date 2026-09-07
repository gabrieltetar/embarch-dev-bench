/* Manufacturer Specific Data (BT_DATA_MANUFACTURER_DATA, 0xFF) parsing and
 * rendering for the advertiser census (embarch-dev-bench/decisions/ble.md
 * decision 44). Split out of ble_bridge_real.c on purpose: `bt_data_parse`
 * itself lives in Zephyr's BT host (subsys/bluetooth/host/data.c), which is
 * not linked for native_sim (ble_bridge_stub.c stands in there instead --
 * design.md §3 decision 16), but the offset arithmetic below has nothing to
 * do with the BT host -- it only needs the bytes `bt_data_parse` has already
 * split out of the AD structure. Keeping it in its own pure-C translation
 * unit, built for every board like eap_interp.c and dev_bench_log.c, is what
 * lets a ztest pin it against a synthetic payload under native_sim.
 */
#ifndef SCAN_SEEN_MFG_H_
#define SCAN_SEEN_MFG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Bytes of an element's payload (after its 2-byte company ID) actually kept
 * per advertiser. [assumed, not measured against a real ESP32-C5 RAM report
 * -- see embarch-dev-bench/open.md.] Chosen small on purpose: SRAM is this
 * board's binding constraint (spec.md, 87.04% at last measurement, three
 * prior overflows), and the one payload this task was written to find is two
 * bytes (a client FICR suffix, ble.md decision 44) -- so 4 gives that a
 * margin without adding a general-purpose byte buffer to a 256-entry static
 * table. Anything longer is not dropped silently: `total_len` still records
 * the real length and the render below says how many bytes were not kept. */
#define SCAN_SEEN_MFG_PAYLOAD_MAX 4

struct scan_seen_mfg_data {
	uint16_t company_id; /* valid only when id_valid */
	uint8_t total_len;   /* payload length after the company ID, as advertised --
			       * uncapped, so a reader can tell a cap was hit */
	uint8_t stored_len;  /* MIN(total_len, SCAN_SEEN_MFG_PAYLOAD_MAX); how many of
			       * payload[] below actually hold advertised bytes */
	uint8_t payload[SCAN_SEEN_MFG_PAYLOAD_MAX];
	bool present;  /* an AD element of this type was seen at all -- absence is a
			* fact (§ task 013), distinct from an element with 0 payload bytes */
	bool id_valid; /* the element carried at least 2 bytes, enough for a company ID */
};

/* Parses one Manufacturer Specific Data AD element's raw bytes -- `data`
 * points at the element's own payload with `data_len` its length, exactly
 * what `bt_data_parse` hands a callback as `data->data`/`data->data_len`,
 * already stripped of the AD structure's own length/type header.
 *
 * Defensive at the one offset that matters: `data_len` is checked against
 * `sizeof(uint16_t)` *before* the company ID bytes are read, never assumed
 * present, which is the bounds-past-length shape `dev-bench/003` and `/004`
 * are open about elsewhere in this codebase. A `data_len` of 0 or 1 is a
 * legal (if unusual) AD element, not a malformed buffer, and is recorded as
 * `present && !id_valid` rather than read out of bounds.
 *
 * Idempotent within one scan_cb call graph: does not overwrite an already-
 * `present` `out`, so the first element seen for an address wins -- the
 * same "don't clobber a good answer with a later duplicate" rule the local
 * name field already follows.
 */
void scan_seen_mfg_data_parse(struct scan_seen_mfg_data *out, const uint8_t *data,
			       uint8_t data_len);

/* Renders `mfg` into `buf` (capacity `buf_len`), NUL-terminated and never
 * written past `buf_len`. Three distinguishable outcomes, on purpose:
 *   - no element was seen at all               -> "(none advertised)"
 *   - an element was seen but too short for a company ID -> says so, with the
 *     actual byte count
 *   - a valid company ID and payload           -> "company=0x%04X data=<hex>",
 *     plus a trailing note naming exactly how many bytes past
 *     SCAN_SEEN_MFG_PAYLOAD_MAX were not kept, when that happened
 * Absence is never rendered as an empty or zeroed payload, and the payload
 * cap is never silent, the same property task 013 asks of the census as a
 * whole. Returns the number of bytes written, excluding the NUL (the
 * `snprintk` convention), so a caller can detect its own buffer running out.
 */
size_t scan_seen_mfg_data_render(char *buf, size_t buf_len, const struct scan_seen_mfg_data *mfg);

#endif /* SCAN_SEEN_MFG_H_ */
