/* scan_seen_names.c's own tests (dev-bench task 007).
 *
 * These pin the specific bug the task was written for: `scan_seen_names_append`
 * must never leave a partial entry (a dangling separator, or a name cut mid-
 * string) in the caller's buffer when an entry does not fit -- the buffer and
 * the `used` cursor must come back exactly as they were, so a caller can tell
 * "nothing more was appended" from "something was appended, cut short" just
 * by comparing before and after.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "scan_seen_names.h"

ZTEST(scan_seen_names, test_names_that_fit_are_joined_with_separators)
{
	char buf[64] = "";
	size_t used = 0;

	zassert_true(scan_seen_names_append(buf, sizeof(buf), sizeof(buf), &used, "", "one"));
	zassert_true(
		scan_seen_names_append(buf, sizeof(buf), sizeof(buf), &used, ", ", "two"));
	zassert_str_equal(buf, "'one', 'two'");
	zassert_equal(used, strlen("'one', 'two'"));
}

ZTEST(scan_seen_names, test_an_entry_that_does_not_fit_leaves_no_partial_fragment)
{
	char buf[64] = "";
	size_t used = 0;

	/* "'one'" is 5 bytes. Budget the call to exactly that -- appending a
	 * second, ", 'two'" entry has no room at all. */
	zassert_true(scan_seen_names_append(buf, sizeof(buf), 5, &used, "", "one"));
	zassert_str_equal(buf, "'one'");
	zassert_equal(used, 5);

	bool fit = scan_seen_names_append(buf, sizeof(buf), 5, &used, ", ", "two");

	zassert_false(fit);
	/* The bug this replaced: snprintk had already written ", 't" (or however
	 * much fit) into buf before the old code noticed the write was too long.
	 * Here, a rejected append must change neither buf nor used at all. */
	zassert_str_equal(buf, "'one'");
	zassert_equal(used, 5);
}

ZTEST(scan_seen_names, test_max_len_reserves_trailing_room_the_caller_will_fill_in_later)
{
	char buf[64] = "";
	size_t used = 0;

	/* buf itself has plenty of room, but the caller has reserved the last 20
	 * bytes of it (a marker to append afterward) via max_len -- 'one' should
	 * still fit against a small max_len even though sizeof(buf) is generous. */
	zassert_true(scan_seen_names_append(buf, sizeof(buf), 5, &used, "", "one"));
	zassert_equal(used, 5);

	/* A second name that fits sizeof(buf) but not the reserved max_len must
	 * still be rejected, and cleanly -- proves max_len is honoured
	 * independently of buf_len, not just clamped once and forgotten. */
	bool fit = scan_seen_names_append(buf, sizeof(buf), 5, &used, ", ", "two");

	zassert_false(fit);
	zassert_str_equal(buf, "'one'");
	zassert_equal(used, 5);
}

ZTEST(scan_seen_names, test_max_len_larger_than_buf_len_is_clamped_not_trusted)
{
	char buf[6] = "";
	size_t used = 0;

	/* max_len (100) claims more room than buf actually has (6 bytes, enough
	 * for "'one'" plus the NUL and nothing else) -- this must not be taken at
	 * face value and write past buf. */
	zassert_true(scan_seen_names_append(buf, sizeof(buf), 100, &used, "", "one"));
	zassert_str_equal(buf, "'one'");
	zassert_equal(used, 5);

	bool fit = scan_seen_names_append(buf, sizeof(buf), 100, &used, ", ", "two");

	zassert_false(fit);
	zassert_str_equal(buf, "'one'");
	zassert_equal(used, 5);
}

ZTEST(scan_seen_names, test_a_name_exactly_at_the_boundary_is_rejected_not_squeezed_in)
{
	char buf[64] = "";
	size_t used = 0;

	/* "'one'" is exactly 5 bytes; a max_len of 5 leaves zero spare bytes, so
	 * even a would-be-exact fit that consumes every last byte is treated as
	 * not fitting -- there must always be room left for the NUL this
	 * function itself does not count in max_len's budget the same way
	 * strlen() wouldn't, but which snprintk needs. */
	bool fit = scan_seen_names_append(buf, sizeof(buf), 4, &used, "", "one");

	zassert_false(fit);
	zassert_str_equal(buf, "");
	zassert_equal(used, 0);
}

ZTEST_SUITE(scan_seen_names, NULL, NULL, NULL, NULL, NULL);
