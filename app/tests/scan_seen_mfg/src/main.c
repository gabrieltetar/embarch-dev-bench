/* scan_seen_mfg.c's own tests (embarch-dev-bench/decisions/ble.md decision
 * 44, dev-bench task 013).
 *
 * All payloads here are synthetic -- constructed byte arrays, never a
 * capture off a real DUT. Per task 013: the correspondence between these
 * bytes and any specific board's hardware ID is a claim read off client
 * firmware source, not something this suite treats as measured, and this
 * file proves only the parsing arithmetic, never an identity.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "scan_seen_mfg.h"

static struct scan_seen_mfg_data fresh(void)
{
	return (struct scan_seen_mfg_data){0};
}

ZTEST(scan_seen_mfg, test_no_element_seen_renders_as_absence_not_a_zero)
{
	struct scan_seen_mfg_data mfg = fresh();
	char buf[64];

	zassert_false(mfg.present);
	(void)scan_seen_mfg_data_render(buf, sizeof(buf), &mfg);
	zassert_str_equal(buf, "(none advertised)");
}

ZTEST(scan_seen_mfg, test_zero_length_element_is_present_but_has_no_valid_company_id)
{
	struct scan_seen_mfg_data mfg = fresh();
	char buf[64];

	/* An element with a 0-byte payload is legal on the air (the AD structure
	 * itself still carries the 0xFF type), just too short for a company ID --
	 * this must not read data[0]/data[1] out of bounds. */
	scan_seen_mfg_data_parse(&mfg, NULL, 0);

	zassert_true(mfg.present);
	zassert_false(mfg.id_valid);
	zassert_equal(mfg.total_len, 0);
	(void)scan_seen_mfg_data_render(buf, sizeof(buf), &mfg);
	zassert_str_equal(buf, "(malformed: 0 byte(s), shorter than a company ID)");
}

ZTEST(scan_seen_mfg, test_one_byte_element_is_the_off_by_one_this_suite_has_paid_for_before)
{
	struct scan_seen_mfg_data mfg = fresh();
	const uint8_t one_byte[] = {0xAB};
	char buf[64];

	/* Exactly one byte short of a company ID -- the shape dev-bench/003 and
	 * /004 are open about elsewhere in this codebase: an index or length one
	 * past what was actually bounds-checked. */
	scan_seen_mfg_data_parse(&mfg, one_byte, sizeof(one_byte));

	zassert_true(mfg.present);
	zassert_false(mfg.id_valid);
	zassert_equal(mfg.total_len, 1);
	zassert_equal(mfg.stored_len, 0);
	(void)scan_seen_mfg_data_render(buf, sizeof(buf), &mfg);
	zassert_str_equal(buf, "(malformed: 1 byte(s), shorter than a company ID)");
}

ZTEST(scan_seen_mfg, test_company_id_only_no_payload_bytes)
{
	struct scan_seen_mfg_data mfg = fresh();
	const uint8_t company_only[] = {0x34, 0x12}; /* little-endian 0x1234 */
	char buf[64];

	scan_seen_mfg_data_parse(&mfg, company_only, sizeof(company_only));

	zassert_true(mfg.present);
	zassert_true(mfg.id_valid);
	zassert_equal(mfg.company_id, 0x1234);
	zassert_equal(mfg.total_len, 0);
	zassert_equal(mfg.stored_len, 0);
	(void)scan_seen_mfg_data_render(buf, sizeof(buf), &mfg);
	zassert_str_equal(buf, "company=0x1234 data=");
}

ZTEST(scan_seen_mfg, test_payload_within_the_cap_is_rendered_in_full)
{
	struct scan_seen_mfg_data mfg = fresh();
	/* Company ID 0x5902 (little-endian), then a 2-byte payload -- the exact
	 * shape task 013 was written for: two bytes after the company ID. */
	const uint8_t elem[] = {0x02, 0x59, 0x6C, 0xDF};
	char buf[64];

	scan_seen_mfg_data_parse(&mfg, elem, sizeof(elem));

	zassert_true(mfg.id_valid);
	zassert_equal(mfg.company_id, 0x5902);
	zassert_equal(mfg.total_len, 2);
	zassert_equal(mfg.stored_len, 2);
	(void)scan_seen_mfg_data_render(buf, sizeof(buf), &mfg);
	zassert_str_equal(buf, "company=0x5902 data=6CDF");
}

ZTEST(scan_seen_mfg, test_payload_past_the_cap_is_visible_never_a_silent_truncation)
{
	struct scan_seen_mfg_data mfg = fresh();
	/* Company ID + 6 payload bytes; SCAN_SEEN_MFG_PAYLOAD_MAX is 4, so two of
	 * these must be reported as not stored, never dropped without a trace. */
	const uint8_t elem[] = {0xEF, 0xBE, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
	char buf[64];

	zassert_equal(SCAN_SEEN_MFG_PAYLOAD_MAX, 4);

	scan_seen_mfg_data_parse(&mfg, elem, sizeof(elem));

	zassert_true(mfg.id_valid);
	zassert_equal(mfg.company_id, 0xBEEF);
	zassert_equal(mfg.total_len, 6);
	zassert_equal(mfg.stored_len, 4);
	zassert_mem_equal(mfg.payload, ((uint8_t[]){0x01, 0x02, 0x03, 0x04}), 4);
	(void)scan_seen_mfg_data_render(buf, sizeof(buf), &mfg);
	zassert_str_equal(buf, "company=0xBEEF data=01020304 (+2 more byte(s), not stored)");
}

ZTEST(scan_seen_mfg, test_first_element_seen_wins_a_later_one_does_not_overwrite_it)
{
	struct scan_seen_mfg_data mfg = fresh();
	const uint8_t first[] = {0x01, 0x00, 0xAA};
	const uint8_t second[] = {0x02, 0x00, 0xBB, 0xCC};

	scan_seen_mfg_data_parse(&mfg, first, sizeof(first));
	scan_seen_mfg_data_parse(&mfg, second, sizeof(second));

	zassert_equal(mfg.company_id, 0x0001);
	zassert_equal(mfg.total_len, 1);
	zassert_equal(mfg.payload[0], 0xAA);
}

ZTEST_SUITE(scan_seen_mfg, NULL, NULL, NULL, NULL, NULL);
