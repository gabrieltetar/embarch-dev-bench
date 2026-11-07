/* Bounded, no-partial-fragment append of one "<sep>'<name>'" entry onto a
 * comma-separated advertiser name list (embarch-dev-bench/decisions/scanning.md
 * decisions 32 and 45's failed-name census). Split out of ble_bridge_real.c for the
 * same reason scan_seen_mfg.c was (see its own header comment): the bound
 * arithmetic here has nothing to do with the BT host, so a ztest can pin its
 * truncation behaviour under native_sim even though ble_bridge_real.c itself
 * never builds there -- app/CMakeLists.txt picks ble_bridge_stub.c instead
 * (design.md §3 decision 16).
 *
 * dev-bench/007 is the reason this exists: the version this replaced grew
 * `buf` with `snprintk` and only afterward checked whether the write had
 * been truncated -- but `snprintk` had already deposited whatever fit before
 * returning the would-be length, so the check came too late to stop a
 * partial name (or a bare trailing separator) from staying in the buffer.
 * This formats each entry into scratch space first and copies it into `buf`
 * only once it is known to fit whole.
 */
#ifndef SCAN_SEEN_NAMES_H_
#define SCAN_SEEN_NAMES_H_

#include <stdbool.h>
#include <stddef.h>

/* Generous fixed size for one formatted entry's scratch buffer -- a
 * separator plus quoted name is well under this for any name this suite's
 * BLE_MAX_LOCAL_NAME_LEN (26 bytes) allows; sized as a round number rather
 * than coupled to that BT-specific constant so this header stays free of any
 * BT-host dependency. */
#define SCAN_SEEN_NAMES_ENTRY_MAX 128

/* Appends "<sep>'<name>'" to `buf` (a NUL-terminated string of `*used` bytes
 * so far, in a buffer of `buf_len` bytes total) -- but only if the whole
 * entry fits within the first `max_len` bytes of `buf` (`max_len <=
 * buf_len`, and `<= buf_len` is enforced here rather than trusted). `max_len`
 * lets a caller reserve trailing bytes in `buf` for something appended
 * afterward -- a truncation marker, say -- without this function's own
 * bound needing to know about it.
 *
 * On success: copies the entry into `buf` starting at `*used`, advances
 * `*used` past it, and returns true.
 *
 * On failure (the entry does not fit whole, `sep` or `name` is malformed, or
 * `entry` would overflow its own scratch buffer): `buf` and `*used` are left
 * exactly as they were -- no partial entry, no dangling separator -- and
 * this returns false. The caller is expected to stop appending and record
 * that the list was cut. */
bool scan_seen_names_append(char *buf, size_t buf_len, size_t max_len, size_t *used,
			     const char *sep, const char *name);

/* ---- The name-filter census's `fail_reason` (decisions 45, 46, 52) -------
 *
 * Composed here rather than in ble_bridge_real.c so the ztest suite reaches
 * the byte budget, not only the appender (decision 52). The census itself --
 * BT addresses, AD parsing -- stays in that file; it hands over only how many
 * advertisers it saw, each one's name ("" for none), and whether the census
 * filled up.
 *
 * The two markers, and why they stay two: `SCAN_SEEN_TRUNCATED_MARKER` means
 * the *name list* ran out of `fail_reason` before it ran out of named
 * advertisers, the common case at three or four names.
 * `SCAN_SEEN_OVERFLOW_MARKER` means the *census* dropped advertisers because
 * it was full -- rare, and not exclusive with the first. When both hold,
 * both are written, concatenated (decision 45). */

/* A format: how many advertisers advertised a name, then how many were seen,
 * so a reader can tell a short list from a partial one. */
#define SCAN_SEEN_PREFIX           "no name match; %u/%u named"
/* Both counts are at most SCAN_SEEN_NAMES_MAX_SEEN, three digits each. */
#define SCAN_SEEN_NAMES_MAX_SEEN   999
#define SCAN_SEEN_PREFIX_MAX_LEN   (sizeof("no name match; 999/999 named") - 1)
#define SCAN_SEEN_NAMES_SEP        ": "
#define SCAN_SEEN_TRUNCATED_MARKER " (truncated)"
#define SCAN_SEEN_OVERFLOW_MARKER  " (census full)"

/* The bytes of `fail_reason` everything but the names can need at once: the
 * prefix at its widest, the separator, and both markers. A caller's
 * `fail_reason` must be longer than this, or the combined case gets cut. */
#define SCAN_SEEN_FAIL_REASON_FIXED_LEN                                                    \
	(SCAN_SEEN_PREFIX_MAX_LEN + (sizeof(SCAN_SEEN_NAMES_SEP) - 1) +                    \
	 (sizeof(SCAN_SEEN_TRUNCATED_MARKER) - 1) + (sizeof(SCAN_SEEN_OVERFLOW_MARKER) - 1))

/* The name of advertiser `index` (0 .. seen - 1), "" if it advertised none. */
typedef const char *(*scan_seen_name_fn)(size_t index, const void *ctx);

/* Writes `SCAN_SEEN_PREFIX[: '<name>', ...][ (truncated)][ (census full)]`
 * into `out` (`out_cap` bytes, NUL included). Nameless advertisers count in
 * `seen` and are left out of the list; with none named, the separator is
 * left out too, since `0/10 named` already says it. The list is budgeted so
 * that whichever marker applies always fits: `(census full)`'s bytes are
 * reserved only when `census_overflowed`, `(truncated)`'s always, since
 * whether it fires is only known once the list is built. Returns false, with
 * `out` empty, if `out_cap` cannot hold SCAN_SEEN_FAIL_REASON_FIXED_LEN plus
 * a NUL or `seen` is over SCAN_SEEN_NAMES_MAX_SEEN. */
bool scan_seen_names_fail_reason(char *out, size_t out_cap, size_t seen,
				 scan_seen_name_fn name_at, const void *ctx, bool census_overflowed);

#endif
