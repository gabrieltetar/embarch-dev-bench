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

#endif
