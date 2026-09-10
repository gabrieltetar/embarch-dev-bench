#include "scan_seen_names.h"

#include <string.h>

#include <zephyr/sys/printk.h>

bool scan_seen_names_append(char *buf, size_t buf_len, size_t max_len, size_t *used,
			     const char *sep, const char *name)
{
	if (max_len > buf_len) {
		max_len = buf_len;
	}
	if (*used > max_len) {
		return false; /* already out of the budget the caller reserved */
	}

	char entry[SCAN_SEEN_NAMES_ENTRY_MAX];
	int entry_len = snprintk(entry, sizeof(entry), "%s'%s'", sep, name);

	/* entry_len < 0: a formatting error. entry_len as a would-be length
	 * >= sizeof(entry): the entry itself is too big for its own scratch
	 * space (never expected -- SCAN_SEEN_NAMES_ENTRY_MAX comfortably
	 * exceeds any name this suite advertises). Either way, nothing has
	 * been written to `buf` yet, so returning here leaves it untouched. */
	if (entry_len < 0 || (size_t)entry_len >= sizeof(entry)) {
		return false;
	}

	/* The actual bound this function exists for: the entry fits its own
	 * scratch space but not the room the caller has left in `buf`. An exact
	 * fit (entry_len == the remaining budget) is accepted -- `max_len` counts
	 * content bytes, not the terminating NUL, which the buf_len check right
	 * below covers separately. Still nothing written to `buf` on failure --
	 * `entry` is scratch, copied out only once both checks have passed. */
	if ((size_t)entry_len > max_len - *used) {
		return false;
	}
	/* Room for the NUL this memcpy also copies, in `buf` itself -- distinct
	 * from `max_len`, which a caller may set well short of `buf_len` (to
	 * reserve trailing bytes for something appended afterward) or, in the
	 * ordinary case, right up against it. */
	if (*used + (size_t)entry_len + 1 > buf_len) {
		return false;
	}

	memcpy(buf + *used, entry, (size_t)entry_len + 1);
	*used += (size_t)entry_len;
	return true;
}
