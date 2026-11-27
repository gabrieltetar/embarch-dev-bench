/* PD bench: the pure half of the app (pd_logic.h). */

#include "pd_logic.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/dt-bindings/usb-c/pd.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

/* Power in mW from mV and mA */
#define MW(mv, ma) ((mv) * (ma) / 1000)
/* A base-10 number */
#define DECIMAL    10

void PdLogicIdle(bool drp, struct pd_logic_idle *idle)
{
	*idle = (struct pd_logic_idle){
		.keepalive_on = true,
		.dr_reply = PD_LOGIC_REPLY_ACCEPT,
		.pr_reply = PD_LOGIC_REPLY_ACCEPT,
		.request_reply = PD_LOGIC_REPLY_ACCEPT,
		.want_kind = PD_LOGIC_WANT_HIGHEST_POWER,
		.offer_from_devicetree = drp,
		.drop_contract = true,
		.vbus_off = drp,
	};
}

bool PdLogicPdoIsFixed(uint32_t pdo)
{
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return fixed.type == PDO_FIXED;
}

bool PdLogicPdoIsPps(uint32_t pdo)
{
	union pd_augmented_supply_pdo_source aug = {.raw_value = pdo};

	return aug.type == PDO_AUGMENTED &&
	       ((pdo >> PDO_APDO_TYPE_SHIFT) & PDO_APDO_TYPE_MASK) == APDO_TYPE_PPS;
}

int PdLogicFixedMv(uint32_t pdo)
{
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(fixed.voltage);
}

int PdLogicFixedMa(uint32_t pdo)
{
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(fixed.max_current);
}

uint32_t PdLogicRdoFixed(int pos, int ma)
{
	union pd_rdo rdo = {.raw_value = 0};

	rdo.fixed.object_pos = pos;
	rdo.fixed.operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(ma);
	rdo.fixed.min_or_max_operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(ma);
	rdo.fixed.no_usb_suspend = 1;

	return rdo.raw_value;
}

uint32_t PdLogicRdoPps(int pos, int mv, int ma)
{
	union pd_rdo rdo = {.raw_value = 0};

	rdo.augmented.object_pos = pos;
	rdo.augmented.output_voltage = PD_LOGIC_MV_TO_PPS_RDO_VOLTAGE(mv);
	rdo.augmented.operating_current = PD_CONVERT_MA_TO_AUGMENTED_PDO_CURRENT(ma);
	rdo.augmented.no_usb_suspend = 1;

	return rdo.raw_value;
}

int PdLogicRdoPos(uint32_t rdo)
{
	union pd_rdo r = {.raw_value = rdo};

	return r.fixed.object_pos;
}

uint32_t PdLogicRdoHighestPower(const uint32_t *caps, int cnt)
{
	int best_pos = 1;
	int best_mw = -1;
	int best_ma = PD_LOGIC_SAFE_MA;

	for (int i = 0; i < cnt; i++) {
		uint32_t pdo = caps[i];
		int ma;
		int mw;

		if (!PdLogicPdoIsFixed(pdo)) {
			continue;
		}
		ma = MIN(PdLogicFixedMa(pdo), PD_LOGIC_CABLE_MA);
		mw = MW(PdLogicFixedMv(pdo), ma);
		if (mw > best_mw) {
			best_mw = mw;
			best_pos = i + 1;
			best_ma = ma;
		}
	}

	return PdLogicRdoFixed(best_pos, best_ma);
}

uint32_t PdLogicRdoFor(const uint32_t *caps, int cnt, const struct pd_logic_want *want,
		       const char **why)
{
	uint32_t pdo;

	if (cnt == 0) {
		*why = "no source caps";
		return 0;
	}
	if (want->kind == PD_LOGIC_WANT_HIGHEST_POWER) {
		return PdLogicRdoHighestPower(caps, cnt);
	}
	if (want->pos < 1 || want->pos > cnt) {
		*why = "position not offered";
		return 0;
	}

	pdo = caps[want->pos - 1];
	if (want->kind == PD_LOGIC_WANT_FIXED) {
		if (!PdLogicPdoIsFixed(pdo)) {
			*why = "not a fixed PDO";
			return 0;
		}
		if (want->ma > PdLogicFixedMa(pdo)) {
			*why = "current above the PDO";
			return 0;
		}
		return PdLogicRdoFixed(want->pos,
				       want->ma > 0 ? want->ma
						    : MIN(PdLogicFixedMa(pdo), PD_LOGIC_CABLE_MA));
	}

	if (!PdLogicPdoIsPps(pdo)) {
		*why = "not a PPS APDO";
		return 0;
	}

	union pd_augmented_supply_pdo_source aug = {.raw_value = pdo};

	if (want->mv < PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug.min_voltage) ||
	    want->mv > PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug.max_voltage)) {
		*why = "voltage outside the APDO";
		return 0;
	}
	if (want->ma > PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(aug.max_current)) {
		*why = "current above the APDO";
		return 0;
	}

	return PdLogicRdoPps(want->pos, want->mv, want->ma);
}

bool PdLogicRequestFits(uint32_t pdo, uint32_t request)
{
	union pd_rdo rdo = {.raw_value = request};
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return pdo != 0 && PdLogicPdoIsFixed(pdo) &&
	       rdo.fixed.operating_current <= fixed.max_current;
}

int PdLogicParseInt(const char *s, int *val)
{
	char *end;
	long v = strtol(s, &end, DECIMAL);

	if (*s == '\0' || *end != '\0' || v < 0 || v > PD_LOGIC_NUMBER_MAX) {
		return -EINVAL;
	}
	*val = (int)v;

	return 0;
}

/* Refuses with `err` and the voltage it concerns */
static int offer_refuse(struct pd_logic_offer_refusal *refusal, enum pd_logic_offer_err err, int mv)
{
	refusal->err = err;
	refusal->mv = mv;

	return -EINVAL;
}

int PdLogicOfferPdo(char *arg, int prev_mv, const uint32_t *offer_max, size_t offer_max_cnt,
		    uint32_t *pdo, int *mv, struct pd_logic_offer_refusal *refusal)
{
	char *colon = strchr(arg, ':');
	int ma;

	*refusal = (struct pd_logic_offer_refusal){.err = PD_LOGIC_OFFER_OK};
	if (colon == NULL) {
		return offer_refuse(refusal, PD_LOGIC_OFFER_NO_COLON, 0);
	}
	*colon = '\0';
	if (PdLogicParseInt(arg, mv) < 0) {
		refusal->bad_number = arg;
		return offer_refuse(refusal, PD_LOGIC_OFFER_BAD_NUMBER, 0);
	}
	if (PdLogicParseInt(colon + 1, &ma) < 0) {
		refusal->bad_number = colon + 1;
		return offer_refuse(refusal, PD_LOGIC_OFFER_BAD_NUMBER, *mv);
	}
	if (prev_mv == 0 && *mv != PD_LOGIC_FIRST_PDO_MV) {
		return offer_refuse(refusal, PD_LOGIC_OFFER_FIRST_NOT_5V, *mv);
	}
	if (*mv <= prev_mv) {
		return offer_refuse(refusal, PD_LOGIC_OFFER_NOT_RISING, *mv);
	}
	for (size_t i = 0; i < offer_max_cnt; i++) {
		if (!PdLogicPdoIsFixed(offer_max[i]) || PdLogicFixedMv(offer_max[i]) != *mv) {
			continue;
		}
		if (ma < PD_LOGIC_OFFER_MIN_MA || ma > PdLogicFixedMa(offer_max[i])) {
			refusal->max_ma = PdLogicFixedMa(offer_max[i]);
			return offer_refuse(refusal, PD_LOGIC_OFFER_CURRENT_OUT_OF_RANGE, *mv);
		}
		*pdo = (offer_max[i] & ~(PDO_CURR_MASK << PDO_FIXED_CURR_SHIFT)) |
		       PDO_FIXED_CURR(ma);
		return 0;
	}

	return offer_refuse(refusal, PD_LOGIC_OFFER_NOT_IN_DEVICETREE, *mv);
}

void PdLogicTimesClear(struct pd_logic_times *t)
{
	*t = (struct pd_logic_times){0};
}

void PdLogicTimesAdd(struct pd_logic_times *t, enum pd_logic_event event, uint64_t us)
{
	__ASSERT_NO_MSG(event < PD_LOGIC_EVENT_MAX);

	t->kept[t->total % PD_LOGIC_TIMES_KEPT] = (struct pd_logic_stamp){event, us};
	t->total++;
	t->last_us[event] = us;
	t->seen[event] = true;
}

int PdLogicTimesCount(const struct pd_logic_times *t)
{
	return MIN(t->total, PD_LOGIC_TIMES_KEPT);
}

struct pd_logic_stamp PdLogicTimesGet(const struct pd_logic_times *t, int i)
{
	__ASSERT_NO_MSG(i >= 0 && i < PdLogicTimesCount(t));

	return t->kept[(t->total - PdLogicTimesCount(t) + i) % PD_LOGIC_TIMES_KEPT];
}

const char *PdLogicEventName(enum pd_logic_event event)
{
	switch (event) {
	case PD_LOGIC_EVENT_ATTACH:
		return "attach";
	case PD_LOGIC_EVENT_SRC_CAPS:
		return "src_caps";
	case PD_LOGIC_EVENT_REQUEST_SENT:
		return "request";
	case PD_LOGIC_EVENT_ACCEPT:
		return "accept";
	case PD_LOGIC_EVENT_PS_RDY:
		return "ps_rdy";
	case PD_LOGIC_EVENT_HARD_RESET_SENT:
		return "hard_reset_sent";
	case PD_LOGIC_EVENT_HARD_RESET_RECEIVED:
		return "hard_reset_received";
	case PD_LOGIC_EVENT_KEEPALIVE_STOPPED:
		return "keepalive_stopped";
	default:
		__ASSERT_NO_MSG(false);
		return "";
	}
}

/* The two events an interval runs between */
static void interval_ends(enum pd_logic_interval interval, enum pd_logic_event *from,
			  enum pd_logic_event *to)
{
	switch (interval) {
	case PD_LOGIC_INTERVAL_ATTACH_TO_SRC_CAPS:
		*from = PD_LOGIC_EVENT_ATTACH;
		*to = PD_LOGIC_EVENT_SRC_CAPS;
		break;
	case PD_LOGIC_INTERVAL_REQUEST_TO_PS_RDY:
		*from = PD_LOGIC_EVENT_REQUEST_SENT;
		*to = PD_LOGIC_EVENT_PS_RDY;
		break;
	case PD_LOGIC_INTERVAL_KEEPALIVE_STOPPED_TO_HARD_RESET:
		*from = PD_LOGIC_EVENT_KEEPALIVE_STOPPED;
		*to = PD_LOGIC_EVENT_HARD_RESET_RECEIVED;
		break;
	default:
		__ASSERT_NO_MSG(false);
		*from = *to = PD_LOGIC_EVENT_ATTACH;
		break;
	}
}

const char *PdLogicIntervalName(enum pd_logic_interval interval)
{
	switch (interval) {
	case PD_LOGIC_INTERVAL_ATTACH_TO_SRC_CAPS:
		return "attach_to_src_caps";
	case PD_LOGIC_INTERVAL_REQUEST_TO_PS_RDY:
		return "request_to_ps_rdy";
	case PD_LOGIC_INTERVAL_KEEPALIVE_STOPPED_TO_HARD_RESET:
		return "keepalive_stopped_to_hard_reset";
	default:
		__ASSERT_NO_MSG(false);
		return "";
	}
}

bool PdLogicInterval(const struct pd_logic_times *t, enum pd_logic_interval interval,
		     uint64_t *us)
{
	enum pd_logic_event from;
	enum pd_logic_event to;

	interval_ends(interval, &from, &to);
	if (!t->seen[from] || !t->seen[to] || t->last_us[from] > t->last_us[to]) {
		return false;
	}
	*us = t->last_us[to] - t->last_us[from];

	return true;
}
