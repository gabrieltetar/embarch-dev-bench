#include "scan_seen_names.h"

#include <string.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

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

/* The name list under construction. Module scope, as the function-local
 * static it replaces was: the one caller runs on the bridge's dispatch
 * thread, and a 128-byte stack buffer on that thread is not worth saving. */
static char census_summary[SCAN_SEEN_NAMES_ENTRY_MAX];

static const char *name_or_empty(scan_seen_name_fn name_at, const void *ctx, size_t index)
{
	const char *name = name_at(index, ctx);

	return (name != NULL) ? name : "";
}

bool scan_seen_names_fail_reason(char *out, size_t out_cap, size_t seen,
				 scan_seen_name_fn name_at, const void *ctx, bool census_overflowed)
{
	if (out_cap == 0) {
		return false;
	}
	out[0] = '\0';
	if (out_cap - 1 <= SCAN_SEEN_FAIL_REASON_FIXED_LEN || seen > SCAN_SEEN_NAMES_MAX_SEEN) {
		return false;
	}

	/* Budgeted against the widest prefix rather than this one, and with the
	 * markers that can fire already taken out, so the final `snprintk`
	 * never has to cut the combined string -- the double truncation that
	 * once ate a marker silently (decision 45). `(census full)` is known
	 * now, so its bytes are reserved only when it will be written;
	 * `(truncated)` is only known once the list is built, so its bytes
	 * always are. */
	size_t marker_budget = (sizeof(SCAN_SEEN_TRUNCATED_MARKER) - 1) +
			       (census_overflowed ? (sizeof(SCAN_SEEN_OVERFLOW_MARKER) - 1) : 0);
	size_t name_budget = (out_cap - 1) - SCAN_SEEN_PREFIX_MAX_LEN -
			     (sizeof(SCAN_SEEN_NAMES_SEP) - 1) - marker_budget;

	if (name_budget > sizeof(census_summary) - 1) {
		name_budget = sizeof(census_summary) - 1;
	}

	size_t used = 0;
	unsigned int named = 0;
	bool truncated = false;

	census_summary[0] = '\0';
	for (size_t i = 0; i < seen; i++) {
		const char *name = name_or_empty(name_at, ctx, i);

		if (name[0] == '\0') {
			continue;
		}
		/* Counted whether or not it fits, so the prefix says how many
		 * the list could have shown. */
		if (!truncated &&
		    !scan_seen_names_append(census_summary, sizeof(census_summary), name_budget,
					    &used, (named == 0) ? "" : ", ", name)) {
			truncated = true; /* complete entries only -- no partial fragment */
		}
		named++;
	}

	/* With nothing named the list is empty and so is its separator:
	 * `0/10 named` already says "seen, none named". */
	int n = snprintk(out, out_cap, SCAN_SEEN_PREFIX "%s%s%s%s", named, (unsigned int)seen,
			 (census_summary[0] != '\0') ? SCAN_SEEN_NAMES_SEP : "", census_summary,
			 truncated ? SCAN_SEEN_TRUNCATED_MARKER : "",
			 census_overflowed ? SCAN_SEEN_OVERFLOW_MARKER : "");

	/* The budget above is what makes a cut impossible; a cut here is a bug
	 * in it, not a long name. */
	__ASSERT(n >= 0 && (size_t)n < out_cap, "%s: census fail_reason was cut", __func__);
	ARG_UNUSED(n); /* read only by the assert, which a build may compile out */
	return true;
}
