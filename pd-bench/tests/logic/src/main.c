/* Unit tests for pd_logic.c, the PD bench's PDO and RDO arithmetic and its shell rules.
 *
 * The expected RDOs are written from the USB PD R3.2 field layout, not from
 * the unions the code uses, so a union that disagrees with the specification
 * fails here. Runs on native_sim: the USB-C stack itself is not exercised.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/dt-bindings/usb-c/pd.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "pd_logic.h"

/* RDO fields, USB PD R3.2: Object Position, No USB Suspend, and the current and voltage units */
#define RDO_OBJECT_POS(pos)      ((uint32_t)(pos) << 28)
#define RDO_NO_USB_SUSPEND       BIT(24)
#define RDO_FIXED_OP_10MA(ma)    ((uint32_t)((ma) / 10) << 10)
#define RDO_FIXED_MAX_10MA(ma)   ((uint32_t)((ma) / 10))
#define RDO_PPS_VOLTAGE_20MV(mv) ((uint32_t)((mv) / 20) << 9)
#define RDO_PPS_CURRENT_50MA(ma) ((uint32_t)((ma) / 50))

/* The DRP1M1's devicetree offer, and a wider one for the rules that need a second voltage */
static const uint32_t dt_offer[] = {PDO_FIXED(5000, 3000, PDO_FIXED_DUAL_ROLE)};
static const uint32_t wide_offer[] = {
	PDO_FIXED(5000, 3000, PDO_FIXED_DUAL_ROLE),
	PDO_FIXED(9000, 3000, 0),
	PDO_FIXED(15000, 3000, 0),
};

ZTEST_SUITE(pd_logic, NULL, NULL, NULL, NULL, NULL);

ZTEST(pd_logic, test_rdo_fixed_encoding)
{
	uint32_t expected = RDO_OBJECT_POS(2) | RDO_NO_USB_SUSPEND | RDO_FIXED_OP_10MA(1500) |
			    RDO_FIXED_MAX_10MA(1500);

	zassert_equal(PdLogicRdoFixed(2, 1500), expected, "0x%08x", PdLogicRdoFixed(2, 1500));
	zassert_equal(PdLogicRdoPos(expected), 2);
}

ZTEST(pd_logic, test_rdo_pps_encoding)
{
	uint32_t expected = RDO_OBJECT_POS(3) | RDO_NO_USB_SUSPEND | RDO_PPS_VOLTAGE_20MV(9000) |
			    RDO_PPS_CURRENT_50MA(2000);

	zassert_equal(PdLogicRdoPps(3, 9000, 2000), expected, "0x%08x",
		      PdLogicRdoPps(3, 9000, 2000));
	zassert_equal(PdLogicRdoPos(expected), 3);
}

ZTEST(pd_logic, test_pdo_classifiers)
{
	uint32_t fixed = PDO_FIXED(9000, 2000, 0);
	uint32_t pps = PDO_PPS_APDO(3300, 21000, 3000);

	zassert_true(PdLogicPdoIsFixed(fixed));
	zassert_false(PdLogicPdoIsPps(fixed));
	zassert_equal(PdLogicFixedMv(fixed), 9000);
	zassert_equal(PdLogicFixedMa(fixed), 2000);
	zassert_true(PdLogicPdoIsPps(pps));
	zassert_false(PdLogicPdoIsFixed(pps));
}

ZTEST(pd_logic, test_max_caps_at_3a)
{
	const uint32_t caps[] = {
		PDO_FIXED(5000, 3000, 0),
		PDO_FIXED(9000, 3000, 0),
		PDO_FIXED(20000, 5000, 0),
		PDO_PPS_APDO(3300, 21000, 5000),
	};
	uint32_t rdo = PdLogicRdoHighestPower(caps, ARRAY_SIZE(caps));

	zassert_equal(rdo, PdLogicRdoFixed(3, PD_LOGIC_CABLE_MA), "0x%08x", rdo);
}

ZTEST(pd_logic, test_max_tie_goes_to_lower_voltage)
{
	const uint32_t caps[] = {
		PDO_FIXED(5000, 3000, 0),
		PDO_FIXED(15000, 1000, 0),
	};

	zassert_equal(PdLogicRdoPos(PdLogicRdoHighestPower(caps, ARRAY_SIZE(caps))), 1);
}

ZTEST(pd_logic, test_choice_checked_against_caps)
{
	const uint32_t caps[] = {
		PDO_FIXED(5000, 3000, 0),
		PDO_PPS_APDO(3300, 11000, 3000),
	};
	struct pd_logic_want fixed_over = {.kind = PD_LOGIC_WANT_FIXED, .pos = 1, .ma = 3500};
	struct pd_logic_want pps_ok = {.kind = PD_LOGIC_WANT_PPS, .pos = 2, .mv = 9000, .ma = 2000};
	struct pd_logic_want pps_high = {
		.kind = PD_LOGIC_WANT_PPS, .pos = 2, .mv = 12000, .ma = 1000};
	const char *why = NULL;

	zassert_equal(PdLogicRdoFor(caps, ARRAY_SIZE(caps), &fixed_over, &why), 0);
	zassert_str_equal(why, "current above the PDO");
	zassert_equal(PdLogicRdoFor(caps, ARRAY_SIZE(caps), &pps_ok, &why),
		      PdLogicRdoPps(2, 9000, 2000));
	zassert_equal(PdLogicRdoFor(caps, ARRAY_SIZE(caps), &pps_high, &why), 0);
	zassert_str_equal(why, "voltage outside the APDO");
	zassert_equal(PdLogicRdoFor(caps, 0, &pps_ok, &why), 0);
	zassert_str_equal(why, "no source caps");
}

ZTEST(pd_logic, test_request_above_pdo_current_refused)
{
	uint32_t pdo = PDO_FIXED(5000, 1500, 0);

	zassert_true(PdLogicRequestFits(pdo, PdLogicRdoFixed(1, 1500)));
	zassert_false(PdLogicRequestFits(pdo, PdLogicRdoFixed(1, 2000)));
	zassert_false(PdLogicRequestFits(0, PdLogicRdoFixed(1, 100)), "no PDO at that position");
	zassert_false(PdLogicRequestFits(PDO_PPS_APDO(3300, 11000, 3000), PdLogicRdoFixed(1, 100)),
		      "not a fixed PDO");
}

ZTEST(pd_logic, test_srccaps_accepts_st_offer)
{
	char arg[] = "5000:1500";
	struct pd_logic_offer_refusal refusal;
	uint32_t pdo = 0;
	int mv = 0;

	zassert_ok(PdLogicOfferPdo(arg, 0, dt_offer, ARRAY_SIZE(dt_offer), &pdo, &mv, &refusal));
	zassert_equal(mv, 5000);
	zassert_equal(pdo, PDO_FIXED(5000, 1500, PDO_FIXED_DUAL_ROLE), "0x%08x", pdo);
}

ZTEST(pd_logic, test_srccaps_first_pdo_is_5v)
{
	char arg[] = "9000:1000";
	struct pd_logic_offer_refusal refusal;
	uint32_t pdo = 0;
	int mv = 0;

	zassert_equal(
		PdLogicOfferPdo(arg, 0, wide_offer, ARRAY_SIZE(wide_offer), &pdo, &mv, &refusal),
		-EINVAL);
	zassert_equal(refusal.err, PD_LOGIC_OFFER_FIRST_NOT_5V);
}

ZTEST(pd_logic, test_srccaps_voltage_must_rise)
{
	char arg[] = "5000:1000";
	struct pd_logic_offer_refusal refusal;
	uint32_t pdo = 0;
	int mv = 0;

	zassert_equal(
		PdLogicOfferPdo(arg, 9000, wide_offer, ARRAY_SIZE(wide_offer), &pdo, &mv, &refusal),
		-EINVAL);
	zassert_equal(refusal.err, PD_LOGIC_OFFER_NOT_RISING);
}

ZTEST(pd_logic, test_srccaps_voltage_not_in_devicetree)
{
	char arg[] = "9000:1000";
	struct pd_logic_offer_refusal refusal;
	uint32_t pdo = 0;
	int mv = 0;

	zassert_equal(
		PdLogicOfferPdo(arg, 5000, dt_offer, ARRAY_SIZE(dt_offer), &pdo, &mv, &refusal),
		-EINVAL);
	zassert_equal(refusal.err, PD_LOGIC_OFFER_NOT_IN_DEVICETREE);
	zassert_equal(refusal.mv, 9000);
}

ZTEST(pd_logic, test_srccaps_current_above_devicetree)
{
	char arg[] = "5000:3500";
	struct pd_logic_offer_refusal refusal;
	uint32_t pdo = 0;
	int mv = 0;

	zassert_equal(PdLogicOfferPdo(arg, 0, dt_offer, ARRAY_SIZE(dt_offer), &pdo, &mv, &refusal),
		      -EINVAL);
	zassert_equal(refusal.err, PD_LOGIC_OFFER_CURRENT_OUT_OF_RANGE);
	zassert_equal(refusal.max_ma, 3000);
}

ZTEST(pd_logic, test_srccaps_bad_text)
{
	char no_colon[] = "5000";
	char bad_ma[] = "5000:lots";
	struct pd_logic_offer_refusal refusal;
	uint32_t pdo = 0;
	int mv = 0;

	zassert_equal(
		PdLogicOfferPdo(no_colon, 0, dt_offer, ARRAY_SIZE(dt_offer), &pdo, &mv, &refusal),
		-EINVAL);
	zassert_equal(refusal.err, PD_LOGIC_OFFER_NO_COLON);
	zassert_equal(
		PdLogicOfferPdo(bad_ma, 0, dt_offer, ARRAY_SIZE(dt_offer), &pdo, &mv, &refusal),
		-EINVAL);
	zassert_equal(refusal.err, PD_LOGIC_OFFER_BAD_NUMBER);
	zassert_str_equal(refusal.bad_number, "lots");
}
