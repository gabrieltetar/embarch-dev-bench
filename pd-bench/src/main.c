/* PD bench: a USB Power Delivery partner the host scripts over the shell.
 *
 * embarch-host-study-proposal.md, phase 1. This is not the dev-bench wire:
 * the board's console carries a Zephyr shell, and the host drives it with
 * Core's signal exchange (embarch-core decision 79). Every `pd` command ends
 * its output with one line, `pd: ok` or `pd: error <reason>`, the marker an
 * exchange waits for. What the bench asks for or offers is set by the host and
 * used the next time the stack needs it; the contract that follows is read
 * back with `pd status`, never assumed.
 *
 * On the X-NUCLEO-SNK1M1 the bench is a sink. On the X-NUCLEO-DRP1M1
 * (CONFIG_USBC_CSM_DRP) it is a dual-role port: it switches the shield's CN3
 * supply onto VBUS through the TCPP03 when the stack asks, answers a sink's
 * Request and a PR_Swap as the host sets, and sends PR_Swap on command.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/usbc_pd.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/dt-bindings/usb-c/pd.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb_c/usbc.h>

#ifdef CONFIG_USBC_CSM_DRP
#include "tcpp03.h"
#endif

LOG_MODULE_REGISTER(pd_bench, LOG_LEVEL_INF);

#define PORT_NODE DT_ALIAS(usbc_port0)
#define USER_NODE DT_PATH(zephyr_user)

#ifdef CONFIG_USBC_CSM_DRP
BUILD_ASSERT(DT_ENUM_HAS_VALUE(PORT_NODE, power_role, dual), "a DRP build needs a dual-role port");
BUILD_ASSERT(DT_HAS_COMPAT_STATUS_OKAY(st_tcpp03), "the DRP build sources through a TCPP03");
BUILD_ASSERT(DT_PROP_LEN(PORT_NODE, source_pdos) <= PDO_MAX_DATA_OBJECTS);
#else
BUILD_ASSERT(DT_ENUM_HAS_VALUE(PORT_NODE, power_role, sink), "a sink-only build needs a sink port");
#endif

/* A PPS Sink sends a Request at least every tPPSRequest, 10 s (USB PD R3.2 p237) */
#define PPS_KEEPALIVE_MS 8000
/* What a 5 V Request asks for when nothing better is valid */
#define SAFE_MA 500
/* `pd max` never asks for more than a cable without an e-marker carries */
#define CABLE_MA 3000
/* A PR_Swap's AMS and power transition end well within this */
#define SWAP_WINDOW_MS 2000
/* How long `pd ask` waits for the partner's answer */
#define ASK_TIMEOUT_MS 1000

#ifdef CONFIG_USBC_CSM_DRP
/* PS_RDY follows the Accept by tSrcTransition (25-35 ms): the supply does not move */
#define SRC_TRANSITION_MS 30
/* After a Hard Reset VBUS stays at vSafe0V for tSrcRecover, 0.66-1 s */
#define SRC_RECOVER_MS 700
/* The source discharges VBUS this far under vSafe0V, a margin for the stack's own check */
#define DISCHARGED_MV (PD_V_SAFE_0V_MAX_MV - 200)
#define VBUS_POLL_MS 2
#endif

/* How the bench answers a partner's DR_Swap */
enum dr_answer {
	DR_ACCEPT,
	DR_REJECT,
	DR_NOT_SUPPORTED,
};

static const char *const dr_answer_names[] = {"accept", "reject", "nosupport"};

#ifdef CONFIG_USBC_CSM_DRP
/* How the bench answers a partner's PR_Swap: the stack sends Accept or Reject, never Wait */
enum pr_answer {
	PR_ACCEPT,
	PR_REJECT,
};

static const char *const pr_answer_names[] = {"accept", "reject"};

static const char *const req_answer_names[] = {
	[SNK_REQUEST_VALID] = "accept",
	[SNK_REQUEST_REJECT] = "reject",
	[SNK_REQUEST_WAIT] = "wait",
};
#endif

/* The swap the bench sent last: a Reject or Not_Supported answers it */
enum swap_kind {
	SWAP_NONE,
	SWAP_DR,
	SWAP_PR,
};

enum ask_kind {
	ASK_NONE,
	ASK_STATUS,
	ASK_PPS_STATUS,
	ASK_BATT_STATUS,
	ASK_SNK_CAPS,
};

/* The partner's information the stack can ask for; the rest has no DPM request */
static const struct {
	const char *name;
	enum usbc_policy_request_t req;
} asks[] = {
	[ASK_STATUS] = {"status", REQUEST_PE_GET_STATUS},
	[ASK_PPS_STATUS] = {"ppsstatus", REQUEST_PE_GET_PPS_STATUS},
	[ASK_BATT_STATUS] = {"battstatus", REQUEST_PE_GET_BATTERY_STATUS},
	[ASK_SNK_CAPS] = {"snkcaps", REQUEST_GET_SNK_CAPS},
};

enum want_kind {
	WANT_MAX,
	WANT_FIXED,
	WANT_PPS,
};

struct want {
	enum want_kind kind;
	int pos;
	int mv;
	int ma;
};

/* What `pd status` and `pd role` show of the roles and the contract */
struct role {
	bool attached;
	bool source;
	bool dfp;
	bool contract;
	uint32_t rdo;
	uint32_t pdo;
};

#ifdef CONFIG_USBC_CSM_DRP
/* After a Hard Reset as source: VBUS at vSafe0V since hr_at, back on, or failed */
enum hard_reset {
	HR_NONE,
	HR_RECOVER,
	HR_FAILED,
};
#endif

static const struct device *const port_dev = DEVICE_DT_GET(PORT_NODE);
static const struct device *const vbus_dev = DEVICE_DT_GET(DT_PHANDLE(PORT_NODE, vbus));
static const struct device *const tcpc_dev = DEVICE_DT_GET(DT_PHANDLE(PORT_NODE, tcpc));
#ifdef CONFIG_USBC_CSM_DRP
static const struct device *const vprov_dev = DEVICE_DT_GET(DT_NODELABEL(vprov));
/* The most the bench may offer: pd srccaps stays within it */
static const uint32_t offer_max[] = DT_PROP(PORT_NODE, source_pdos);
#else
static const struct gpio_dt_spec tcpp_vcc = GPIO_DT_SPEC_GET(USER_NODE, tcpp_vcc_gpios);
static const struct gpio_dt_spec tcpp_db = GPIO_DT_SPEC_GET(USER_NODE, tcpp_db_gpios);
#endif

static struct {
	uint32_t snk_caps[DT_PROP_LEN(PORT_NODE, sink_pdos)];

	/* Guards everything below: the stack thread and the shell share it */
	struct k_spinlock lock;
	uint32_t src_caps[PDO_MAX_DATA_OBJECTS];
	int src_cap_cnt;
	struct want want;
	/* The last Request sent, and the PDO it named */
	uint32_t sent_rdo;
	uint32_t sent_pdo;
	/* The Request the Source accepted and moved its supply for */
	bool contract;
	uint32_t contract_rdo;
	uint32_t contract_pdo;
	/* The next Request repeats the contract: a PPS keep-alive */
	bool refresh;
	/* Off, a PPS contract gets no keep-alive: the source's tPPSTimeout test */
	bool keepalive_off;
	bool pd_connected;
	int rp_ma;
	int rejects;
	int waits;
	/* Data roles (Table 7.4): the bench's answer, its role, its own last DR_Swap */
	enum dr_answer dr_answer;
	bool dfp;
	bool drs_pending;
	const char *drs_result;
	/* Power roles: a partner attached, the bench's role, its own last PR_Swap */
	bool attached;
	bool source;
	bool prs_pending;
	const char *prs_result;
	enum swap_kind awaiting;
	/* Until then a PR_Swap may be under way: VBUS going off is the swap, not a detach */
	int64_t swap_until;
	bool partner_swap;
	/* What `pd ask` waits for, and the partner's answers */
	enum ask_kind ask;
	bool ask_got;
	struct pd_sdb sdb;
	union pd_ppssdb ppssdb;
	union pd_bsdo bsdo;
	uint32_t partner_snk_caps[PDO_MAX_DATA_OBJECTS];
	int partner_snk_cap_cnt;
#ifdef CONFIG_USBC_CSM_DRP
	/* The Source_Capabilities the bench offers, within offer_max */
	uint32_t offer[PDO_MAX_DATA_OBJECTS];
	int offer_cnt;
	/* How the bench answers a Request and a PR_Swap */
	enum usbc_snk_req_reply_t req_answer;
	enum pr_answer pr_answer;
	int requests;
	const char *last_answer;
	/* The Request accepted, and the contract once the PS_RDY is due */
	uint32_t req_rdo;
	uint32_t req_pdo;
	bool src_contract;
	uint32_t src_rdo;
	uint32_t src_pdo;
	int faults;
	uint8_t fault_flags;
#endif

	struct k_work_delayable keepalive;
	struct k_sem ask_sem;
#ifdef CONFIG_USBC_CSM_DRP
	/* The stack thread's own: the caps last sent, and the source's supply timing */
	uint32_t sent_caps[PDO_MAX_DATA_OBJECTS];
	int sent_cap_cnt;
	int64_t ps_at;
	enum hard_reset hr;
	int64_t hr_at;
#endif
} self = {
	.snk_caps = DT_PROP(PORT_NODE, sink_pdos),
#ifdef CONFIG_USBC_CSM_DRP
	.offer = DT_PROP(PORT_NODE, source_pdos),
	.offer_cnt = DT_PROP_LEN(PORT_NODE, source_pdos),
	.req_answer = SNK_REQUEST_VALID,
	.pr_answer = PR_ACCEPT,
#endif
};

static bool pdo_is_fixed(uint32_t pdo)
{
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return fixed.type == PDO_FIXED;
}

static bool pdo_is_pps(uint32_t pdo)
{
	union pd_augmented_supply_pdo_source aug = {.raw_value = pdo};

	return aug.type == PDO_AUGMENTED &&
	       ((pdo >> PDO_APDO_TYPE_SHIFT) & PDO_APDO_TYPE_MASK) == APDO_TYPE_PPS;
}

static int fixed_mv(uint32_t pdo)
{
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(fixed.voltage);
}

static int fixed_ma(uint32_t pdo)
{
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(fixed.max_current);
}

static uint32_t rdo_fixed(int pos, int ma)
{
	union pd_rdo rdo = {.raw_value = 0};

	rdo.fixed.object_pos = pos;
	rdo.fixed.operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(ma);
	rdo.fixed.min_or_max_operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(ma);
	rdo.fixed.no_usb_suspend = 1;

	return rdo.raw_value;
}

static uint32_t rdo_pps(int pos, int mv, int ma)
{
	union pd_rdo rdo = {.raw_value = 0};

	rdo.augmented.object_pos = pos;
	rdo.augmented.output_voltage = mv / 20;
	rdo.augmented.operating_current = PD_CONVERT_MA_TO_AUGMENTED_PDO_CURRENT(ma);
	rdo.augmented.no_usb_suspend = 1;

	return rdo.raw_value;
}

static int rdo_pos(uint32_t rdo)
{
	union pd_rdo r = {.raw_value = rdo};

	return r.fixed.object_pos;
}

/* The highest power fixed PDO, at most 3 A; ties go to the lower voltage */
static uint32_t rdo_max_locked(void)
{
	int best_pos = 1;
	int best_mw = -1;
	int best_ma = SAFE_MA;

	for (int i = 0; i < self.src_cap_cnt; i++) {
		uint32_t pdo = self.src_caps[i];
		int ma;
		int mw;

		if (!pdo_is_fixed(pdo)) {
			continue;
		}
		ma = MIN(fixed_ma(pdo), CABLE_MA);
		mw = fixed_mv(pdo) * ma / 1000;
		if (mw > best_mw) {
			best_mw = mw;
			best_pos = i + 1;
			best_ma = ma;
		}
	}

	return rdo_fixed(best_pos, best_ma);
}

/* The RDO for a choice against the caps held, or 0 with the reason it is not valid */
static uint32_t rdo_for_locked(const struct want *w, const char **why)
{
	uint32_t pdo;

	if (self.src_cap_cnt == 0) {
		*why = "no source caps";
		return 0;
	}
	if (w->kind == WANT_MAX) {
		return rdo_max_locked();
	}
	if (w->pos < 1 || w->pos > self.src_cap_cnt) {
		*why = "position not offered";
		return 0;
	}

	pdo = self.src_caps[w->pos - 1];
	if (w->kind == WANT_FIXED) {
		if (!pdo_is_fixed(pdo)) {
			*why = "not a fixed PDO";
			return 0;
		}
		if (w->ma > fixed_ma(pdo)) {
			*why = "current above the PDO";
			return 0;
		}
		return rdo_fixed(w->pos, w->ma > 0 ? w->ma : MIN(fixed_ma(pdo), CABLE_MA));
	}

	if (!pdo_is_pps(pdo)) {
		*why = "not a PPS APDO";
		return 0;
	}

	union pd_augmented_supply_pdo_source aug = {.raw_value = pdo};

	if (w->mv < PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug.min_voltage) ||
	    w->mv > PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug.max_voltage)) {
		*why = "voltage outside the APDO";
		return 0;
	}
	if (w->ma > PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(aug.max_current)) {
		*why = "current above the APDO";
		return 0;
	}

	return rdo_pps(w->pos, w->mv, w->ma);
}

static int policy_get_snk_cap(const struct device *dev, uint32_t **pdos, int *num_pdos)
{
	ARG_UNUSED(dev);

	*pdos = self.snk_caps;
	*num_pdos = ARRAY_SIZE(self.snk_caps);

	return 0;
}

static void policy_set_src_cap(const struct device *dev, const uint32_t *pdos, const int num_pdos)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(dev);

	self.src_cap_cnt = MIN(num_pdos, PDO_MAX_DATA_OBJECTS);
	memcpy(self.src_caps, pdos, self.src_cap_cnt * sizeof(self.src_caps[0]));
	k_spin_unlock(&self.lock, key);
}

static uint32_t policy_get_rdo(const struct device *dev)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	int cpos = rdo_pos(self.contract_rdo);
	const char *why = NULL;
	uint32_t rdo;

	ARG_UNUSED(dev);

	if (self.refresh && self.contract && cpos >= 1 && cpos <= self.src_cap_cnt &&
	    self.src_caps[cpos - 1] == self.contract_pdo) {
		rdo = self.contract_rdo;
	} else {
		rdo = rdo_for_locked(&self.want, &why);
	}
	self.refresh = false;
	if (rdo == 0) {
		/* The caps changed under the choice: vSafe5V rather than no Request */
		rdo = rdo_fixed(1, self.src_cap_cnt > 0 ? MIN(SAFE_MA, fixed_ma(self.src_caps[0]))
							: SAFE_MA);
	}
	self.sent_rdo = rdo;
	self.sent_pdo = self.src_caps[rdo_pos(rdo) - 1];
	k_spin_unlock(&self.lock, key);

	if (why != NULL) {
		LOG_WRN("choice not valid (%s), asking vSafe5V", why);
	}
	LOG_INF("Request 0x%08x", rdo);

	return rdo;
}

static void keepalive_handler(struct k_work *work)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool pps = self.contract && pdo_is_pps(self.contract_pdo) && !self.keepalive_off;

	ARG_UNUSED(work);

	self.refresh = pps;
	k_spin_unlock(&self.lock, key);

	if (!pps) {
		return;
	}

	if (usbc_request(port_dev, REQUEST_PE_SNK_REQUEST) < 0) {
		LOG_WRN("PPS keep-alive Request not queued, retrying");
		k_work_reschedule(&self.keepalive, K_MSEC(100));
		return;
	}
	k_work_reschedule(&self.keepalive, K_MSEC(PPS_KEEPALIVE_MS));
}

static void contract_end(void)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	self.contract = false;
	self.refresh = false;
	k_spin_unlock(&self.lock, key);
	k_work_cancel_delayable(&self.keepalive);
}

/* Whether a partner terminates either CC line: a PR_Swap keeps it, a detach does not */
static bool partner_present(void)
{
	enum tc_cc_voltage_state cc1;
	enum tc_cc_voltage_state cc2;

	return tcpc_get_cc(tcpc_dev, &cc1, &cc2) == 0 &&
	       (cc1 != TC_CC_VOLT_OPEN || cc2 != TC_CC_VOLT_OPEN);
}

/* VBUS going off is a PR_Swap's while one is under way and the partner is still there */
static bool swapping(void)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool swap = k_uptime_get() < self.swap_until;

	k_spin_unlock(&self.lock, key);

	return swap && partner_present();
}

/* The partner is gone: forget what it told the bench */
static void detach(void)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	self.attached = false;
	self.source = false;
	self.dfp = false;
	self.drs_pending = false;
	self.prs_pending = false;
	self.swap_until = 0;
	self.partner_swap = false;
	self.rp_ma = 0;
	self.src_cap_cnt = 0;
	self.pd_connected = false;
#ifdef CONFIG_USBC_CSM_DRP
	self.src_contract = false;
#endif
	k_spin_unlock(&self.lock, key);
	contract_end();
	LOG_INF("detached");
}

/* The stack starts the Policy Engine in a power role: after an attach, a PR_Swap or a Hard Reset */
static void role_set(bool source)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool swapped = self.attached && source != self.source;

	if (!self.attached) {
		/* A new attach: the Source starts as the DFP (USB Type-C) */
		self.attached = true;
		self.dfp = source;
	} else if (swapped && self.partner_swap) {
		self.partner_swap = false;
	} else if (swapped && self.prs_pending) {
		self.prs_pending = false;
		self.prs_result = "accepted";
	}
	self.swap_until = 0;
	self.source = source;
	if (source) {
		/* The partner's Source_Capabilities no longer apply */
		self.src_cap_cnt = 0;
		self.rp_ma = 0;
	}
#ifdef CONFIG_USBC_CSM_DRP
	else {
		self.src_contract = false;
	}
#endif
	k_spin_unlock(&self.lock, key);

	if (source) {
		contract_end();
	}
	LOG_INF("power role %s%s", source ? "source" : "sink", swapped ? " after a PR_Swap" : "");
}

/* A PR_Swap under way ends in a reset or ErrorRecovery: the VBUS loss that follows is a detach */
static void swap_failed_locked(void)
{
	if (k_uptime_get() >= self.swap_until) {
		return;
	}
	self.swap_until = 0;
	self.partner_swap = false;
	if (self.prs_pending) {
		self.prs_pending = false;
		self.prs_result = "failed";
	}
}

/* A Reject or Not_Supported answers the swap the bench sent last that still waits */
static bool swap_answered_locked(const char *result)
{
	if (self.awaiting == SWAP_PR && self.prs_pending) {
		self.prs_pending = false;
		self.prs_result = result;
		self.swap_until = 0;
		return true;
	}
	if (self.drs_pending) {
		self.drs_pending = false;
		self.drs_result = result;
		return true;
	}

	return false;
}

/* What `pd ask` waits for has come, or the partner sent none */
static void ask_answered(enum ask_kind kind, bool got)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool waited = self.ask == kind;

	if (waited) {
		self.ask = ASK_NONE;
		self.ask_got = got;
	}
	k_spin_unlock(&self.lock, key);

	if (waited) {
		k_sem_give(&self.ask_sem);
	}
}

static void policy_set_status(const struct device *dev, const struct pd_sdb *sdb)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(dev);

	if (sdb != NULL) {
		self.sdb = *sdb;
	}
	k_spin_unlock(&self.lock, key);
	ask_answered(ASK_STATUS, sdb != NULL);
}

static void policy_set_pps_status(const struct device *dev, const union pd_ppssdb *ppssdb)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(dev);

	if (ppssdb != NULL) {
		self.ppssdb = *ppssdb;
	}
	k_spin_unlock(&self.lock, key);
	ask_answered(ASK_PPS_STATUS, ppssdb != NULL);
}

static void policy_set_battery_status(const struct device *dev, const union pd_bsdo *bsdo)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(dev);

	if (bsdo != NULL) {
		self.bsdo = *bsdo;
	}
	k_spin_unlock(&self.lock, key);
	ask_answered(ASK_BATT_STATUS, bsdo != NULL);
}

#ifdef CONFIG_USBC_CSM_DRP

/* The CN3 supply the provider path switches onto VBUS is at vSafe5V */
static bool supply_ok(void)
{
	int mv = 0;
	int ret = usbc_vbus_measure(vprov_dev, &mv);

	if (ret < 0 || mv < PD_V_SAFE_5V_MIN_MV || mv > PD_V_SAFE_5V_MAX_MV) {
		LOG_ERR("CN3 at %d mV (%d), not vSafe5V", mv, ret);
		return false;
	}

	return true;
}

/* VBUS off and discharged, then the TCPP03 idle */
static int source_off(void)
{
	int ret = tcpp03_source(false);
	int mv = 0;
	int idle;

	for (int t = 0; ret == 0; t += VBUS_POLL_MS) {
		if (usbc_vbus_measure(vbus_dev, &mv) == 0 && mv < DISCHARGED_MV) {
			break;
		}
		if (t >= PD_T_SAFE_0V_MAX_MS) {
			LOG_ERR("VBUS at %d mV %d ms after the source turned off", mv, t);
			ret = -ETIMEDOUT;
			break;
		}
		k_msleep(VBUS_POLL_MS);
	}
	idle = tcpp03_idle();

	return ret < 0 ? ret : idle;
}

/* VBUS on from CN3, only at vSafe5V, and returned once VBUS is there */
static int source_on(void)
{
	int ret;
	int mv = 0;

	if (!supply_ok()) {
		return -EIO;
	}

	ret = tcpp03_source(true);
	for (int t = 0; ret == 0; t += VBUS_POLL_MS) {
		if (usbc_vbus_measure(vbus_dev, &mv) == 0 && mv >= PD_V_SAFE_5V_MIN_MV) {
			LOG_INF("sourcing VBUS, %d mV", mv);
			return 0;
		}
		if (t >= TC_T_VBUS_ON_MAX_MS) {
			ret = -ETIMEDOUT;
			break;
		}
		k_msleep(VBUS_POLL_MS);
	}

	LOG_ERR("VBUS at %d mV, not sourcing (%d)", mv, ret);
	(void)source_off();

	return -EIO;
}

/* A fault opened both paths: start over from an unattached port */
static void tcpp_fault(uint8_t flags)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	self.faults++;
	self.fault_flags = flags;
	k_spin_unlock(&self.lock, key);

	(void)usbc_request(port_dev, REQUEST_TC_ERROR_RECOVERY);
}

static int policy_src_en(const struct device *dev, bool en)
{
	k_spinlock_key_t key;
	bool was_source;
	int ret;

	ARG_UNUSED(dev);

	if (en) {
		return source_on();
	}

	ret = source_off();
	key = k_spin_lock(&self.lock);
	was_source = self.attached && self.source;
	k_spin_unlock(&self.lock, key);
	if (was_source && !swapping()) {
		detach();
	}

	return ret;
}

/* The Type-C Current Rp advertises: as much as the 5 V PDO offers */
static int src_rp_ma_locked(void)
{
	int ma = fixed_ma(self.offer[0]);

	return ma >= 3000 ? 3000 : ma >= 1500 ? 1500 : 500;
}

static int policy_get_src_rp(const struct device *dev, enum tc_rp_value *rp)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	int ma = src_rp_ma_locked();

	ARG_UNUSED(dev);
	k_spin_unlock(&self.lock, key);

	*rp = ma == 3000 ? TC_RP_3A0 : ma == 1500 ? TC_RP_1A5 : TC_RP_USB;

	return 0;
}

static int policy_get_src_caps(const struct device *dev, const uint32_t **pdos, uint32_t *num_pdos)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(dev);

	self.sent_cap_cnt = self.offer_cnt;
	memcpy(self.sent_caps, self.offer, self.offer_cnt * sizeof(self.offer[0]));
	k_spin_unlock(&self.lock, key);

	*pdos = self.sent_caps;
	*num_pdos = self.sent_cap_cnt;

	return 0;
}

/* A Request fits a fixed PDO when its operating current is within the PDO's */
static bool request_fits(uint32_t pdo, uint32_t request)
{
	union pd_rdo rdo = {.raw_value = request};
	union pd_fixed_supply_pdo_source fixed = {.raw_value = pdo};

	return pdo != 0 && pdo_is_fixed(pdo) && rdo.fixed.operating_current <= fixed.max_current;
}

/* The answer the host set, but never Accept for more than the caps sent offer */
static enum usbc_snk_req_reply_t policy_check_sink_request(const struct device *dev,
							   const uint32_t request)
{
	int pos = rdo_pos(request);
	uint32_t pdo = pos >= 1 && pos <= self.sent_cap_cnt ? self.sent_caps[pos - 1] : 0;
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	enum usbc_snk_req_reply_t reply = self.req_answer;

	ARG_UNUSED(dev);

	if (reply == SNK_REQUEST_VALID && !request_fits(pdo, request)) {
		reply = SNK_REQUEST_REJECT;
	}
	if (reply == SNK_REQUEST_VALID) {
		self.req_rdo = request;
		self.req_pdo = pdo;
	}
	self.requests++;
	self.last_answer = req_answer_names[reply];
	k_spin_unlock(&self.lock, key);

	LOG_INF("Request 0x%08x: %s", request, req_answer_names[reply]);

	return reply;
}

static bool policy_present_contract_is_valid(const struct device *dev, const uint32_t contract)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	int pos = rdo_pos(contract);
	bool valid =
		pos >= 1 && pos <= self.offer_cnt && request_fits(self.offer[pos - 1], contract);

	ARG_UNUSED(dev);
	k_spin_unlock(&self.lock, key);

	return valid;
}

static bool policy_is_ps_ready(const struct device *dev)
{
	ARG_UNUSED(dev);

	return k_uptime_get() - self.ps_at >= SRC_TRANSITION_MS;
}

static void policy_set_port_partner_snk_cap(const struct device *dev, const uint32_t *pdos,
					    const int num_pdos)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(dev);

	self.partner_snk_cap_cnt = MIN(num_pdos, PDO_MAX_DATA_OBJECTS);
	memcpy(self.partner_snk_caps, pdos,
	       self.partner_snk_cap_cnt * sizeof(self.partner_snk_caps[0]));
	k_spin_unlock(&self.lock, key);
	ask_answered(ASK_SNK_CAPS, true);
}

/* Hard Reset as source: VBUS to vSafe0V now, back to vSafe5V after tSrcRecover */
static void src_hard_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	self.src_contract = false;
	swap_failed_locked();
	k_spin_unlock(&self.lock, key);

	LOG_WRN("Hard Reset: VBUS off for tSrcRecover");
	if (source_off() < 0) {
		self.hr = HR_FAILED;
		(void)usbc_request(port_dev, REQUEST_TC_ERROR_RECOVERY);
		return;
	}
	self.hr = HR_RECOVER;
	self.hr_at = k_uptime_get();
}

static bool src_recovered(void)
{
	switch (self.hr) {
	case HR_NONE:
		return true;
	case HR_RECOVER:
		if (k_uptime_get() - self.hr_at < SRC_RECOVER_MS) {
			return false;
		}
		if (source_on() < 0) {
			self.hr = HR_FAILED;
			(void)usbc_request(port_dev, REQUEST_TC_ERROR_RECOVERY);
			return false;
		}
		self.hr = HR_NONE;
		return true;
	default:
		/* Error recovery ends the Hard Reset */
		return false;
	}
}

/* A sink takes the source role only with CN3 at vSafe5V */
static bool pr_swap_check(void)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool accept = self.pr_answer == PR_ACCEPT;
	bool to_source = !self.source;

	k_spin_unlock(&self.lock, key);

	if (accept && to_source && !supply_ok()) {
		accept = false;
	}
	if (accept) {
		key = k_spin_lock(&self.lock);
		self.partner_swap = true;
		self.swap_until = k_uptime_get() + SWAP_WINDOW_MS;
		k_spin_unlock(&self.lock, key);
	}
	LOG_INF("PR_Swap from the partner: %s", accept ? "accept" : "reject");

	return accept;
}

#endif /* CONFIG_USBC_CSM_DRP */

static void policy_notify(const struct device *dev, const enum usbc_policy_notify_t notify)
{
	k_spinlock_key_t key;

	ARG_UNUSED(dev);

	switch (notify) {
	case POWER_ROLE_IS_SOURCE:
	case POWER_ROLE_IS_SINK:
		role_set(notify == POWER_ROLE_IS_SOURCE);
		break;
	case TRANSITION_PS:
		key = k_spin_lock(&self.lock);
#ifdef CONFIG_USBC_CSM_DRP
		if (self.source) {
			self.src_contract = true;
			self.src_rdo = self.req_rdo;
			self.src_pdo = self.req_pdo;
			self.ps_at = k_uptime_get();
			k_spin_unlock(&self.lock, key);
			LOG_INF("contract in force as source");
			break;
		}
#endif
		self.contract = true;
		self.contract_rdo = self.sent_rdo;
		self.contract_pdo = self.sent_pdo;
		k_spin_unlock(&self.lock, key);
		LOG_INF("contract in force");
		k_work_reschedule(&self.keepalive, K_MSEC(PPS_KEEPALIVE_MS));
		break;
	case PD_CONNECTED:
		key = k_spin_lock(&self.lock);
		self.pd_connected = true;
		k_spin_unlock(&self.lock, key);
		break;
	case NOT_PD_CONNECTED:
		key = k_spin_lock(&self.lock);
		self.pd_connected = false;
		k_spin_unlock(&self.lock, key);
		break;
	case POWER_CHANGE_0A0:
		/* A detach, or a PR_Swap to source whose partner turns its supply off */
		if (!swapping()) {
			detach();
			break;
		}
		key = k_spin_lock(&self.lock);
		self.rp_ma = 0;
		k_spin_unlock(&self.lock, key);
		contract_end();
		break;
	case POWER_CHANGE_DEF:
	case POWER_CHANGE_1A5:
	case POWER_CHANGE_3A0:
		key = k_spin_lock(&self.lock);
		self.rp_ma = notify == POWER_CHANGE_3A0 ? 3000 : notify == POWER_CHANGE_1A5 ? 1500 : 500;
		k_spin_unlock(&self.lock, key);
		break;
	case MSG_REJECTED_RECEIVED:
		key = k_spin_lock(&self.lock);
		if (!swap_answered_locked("rejected")) {
			self.rejects++;
		}
		k_spin_unlock(&self.lock, key);
		LOG_WRN("Rejected");
		break;
	case MSG_NOT_SUPPORTED_RECEIVED:
		key = k_spin_lock(&self.lock);
		(void)swap_answered_locked("not supported");
		k_spin_unlock(&self.lock, key);
		LOG_WRN("Not_Supported");
		break;
	case PROTOCOL_ERROR:
		key = k_spin_lock(&self.lock);
		swap_failed_locked();
		k_spin_unlock(&self.lock, key);
		break;
	case SENDER_RESPONSE_TIMEOUT:
		key = k_spin_lock(&self.lock);
		if (self.awaiting == SWAP_PR && self.prs_pending) {
			self.prs_pending = false;
			self.prs_result = "no response";
			self.swap_until = 0;
		}
		k_spin_unlock(&self.lock, key);
		break;
	case DATA_ROLE_IS_UFP:
	case DATA_ROLE_IS_DFP:
		key = k_spin_lock(&self.lock);
		self.dfp = notify == DATA_ROLE_IS_DFP;
		if (self.drs_pending) {
			self.drs_pending = false;
			self.drs_result = "accepted";
		}
		k_spin_unlock(&self.lock, key);
		LOG_INF("data role %s", notify == DATA_ROLE_IS_DFP ? "DFP" : "UFP");
		break;
	case SNK_TRANSITION_TO_DEFAULT:
		key = k_spin_lock(&self.lock);
		swap_failed_locked();
		k_spin_unlock(&self.lock, key);
		contract_end();
		LOG_WRN("Hard Reset");
		break;
#ifdef CONFIG_USBC_CSM_DRP
	case HARD_RESET_RECEIVED:
		src_hard_reset();
		break;
#endif
	case PORT_PARTNER_NOT_RESPONSIVE:
		LOG_WRN("partner is not PD capable");
		break;
	default:
		break;
	}
}

/* A Wait leaves the contract as it was: the stack repeats the Request after tSinkRequest */
static bool policy_wait_notify(const struct device *dev, const enum usbc_policy_wait_t wait)
{
	k_spinlock_key_t key;

	ARG_UNUSED(dev);

	if (wait == WAIT_DATA_ROLE_SWAP) {
		key = k_spin_lock(&self.lock);
		self.drs_pending = false;
		self.drs_result = "wait";
		k_spin_unlock(&self.lock, key);
		return false;
	}
	if (wait == WAIT_POWER_ROLE_SWAP) {
		key = k_spin_lock(&self.lock);
		if (self.prs_pending) {
			self.prs_pending = false;
			self.prs_result = "wait";
		}
		self.swap_until = 0;
		k_spin_unlock(&self.lock, key);
		return false;
	}
	if (wait != WAIT_SINK_REQUEST) {
		return false;
	}
	key = k_spin_lock(&self.lock);
	self.waits++;
	k_spin_unlock(&self.lock, key);
	LOG_INF("Request answered with Wait, repeating it");

	return true;
}

static bool policy_check(const struct device *dev, const enum usbc_policy_check_t check)
{
	enum dr_answer answer = self.dr_answer;

	ARG_UNUSED(dev);

	switch (check) {
	case CHECK_SNK_AT_DEFAULT_LEVEL:
		return true;
	case CHECK_DATA_ROLE_SWAP_TO_DFP:
	case CHECK_DATA_ROLE_SWAP_TO_UFP:
		return answer == DR_ACCEPT;
#ifdef CONFIG_USBC_CSM_DRP
	case CHECK_POWER_ROLE_SWAP:
		return pr_swap_check();
	case CHECK_SRC_PS_AT_DEFAULT_LEVEL:
		return src_recovered();
#endif
	default:
		return false;
	}
}

/* The Dual-Role Data bit of a 5 V PDO, set when the bench takes a DR_Swap */
static uint32_t drd_bit(uint32_t pdo, enum dr_answer answer)
{
	return answer == DR_NOT_SUPPORTED ? pdo & ~PDO_FIXED_DATA_SWAP : pdo | PDO_FIXED_DATA_SWAP;
}

/*
 * The 5 V PDOs' Dual-Role Data bit follows whether the bench takes a DR_Swap:
 * the stack answers Not_Supported when it is clear
 */
static void dr_answer_set(enum dr_answer answer)
{
	self.dr_answer = answer;
	self.snk_caps[0] = drd_bit(self.snk_caps[0], answer);
#ifdef CONFIG_USBC_CSM_DRP
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	self.offer[0] = drd_bit(self.offer[0], answer);
	k_spin_unlock(&self.lock, key);
#endif
}

static void role_get(struct role *r)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	r->attached = self.attached;
	r->source = self.source;
	r->dfp = self.dfp;
	r->contract = self.contract;
	r->rdo = self.contract_rdo;
	r->pdo = self.contract_pdo;
#ifdef CONFIG_USBC_CSM_DRP
	if (self.source) {
		r->contract = self.src_contract;
		r->rdo = self.src_rdo;
		r->pdo = self.src_pdo;
	}
#endif
	k_spin_unlock(&self.lock, key);
}

static const char *power_name(const struct role *r)
{
	return !r->attached ? "none" : r->source ? "source" : "sink";
}

static void print_contract(const struct shell *sh, const struct role *r)
{
	union pd_rdo rdo = {.raw_value = r->rdo};

	if (!r->contract) {
		shell_print(sh, "contract: none");
	} else if (pdo_is_pps(r->pdo)) {
		shell_print(sh, "contract: pps pos %d %d mV %d mA", rdo.augmented.object_pos,
			    rdo.augmented.output_voltage * 20,
			    PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(rdo.augmented.operating_current));
	} else {
		shell_print(sh, "contract: fixed pos %d %d mV %d mA", rdo.fixed.object_pos,
			    fixed_mv(r->pdo),
			    PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(rdo.fixed.operating_current));
	}
}

static void print_pdo(const struct shell *sh, int pos, uint32_t pdo)
{
	if (pdo_is_fixed(pdo)) {
		shell_print(sh, "%d: fixed %d mV %d mA", pos, fixed_mv(pdo), fixed_ma(pdo));
	} else if (pdo_is_pps(pdo)) {
		union pd_augmented_supply_pdo_source aug = {.raw_value = pdo};

		shell_print(sh, "%d: pps %d-%d mV %d mA%s", pos,
			    PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug.min_voltage),
			    PD_CONVERT_AUGMENTED_PDO_VOLTAGE_TO_MV(aug.max_voltage),
			    PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(aug.max_current),
			    aug.pps_power_limited ? " power-limited" : "");
	} else {
		shell_print(sh, "%d: other 0x%08x", pos, pdo);
	}
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct role r;
	k_spinlock_key_t key;
	bool pd;
	int rp_ma;
	int rejects;
	int waits;
	const char *drs;
	const char *prs;
	enum dr_answer answer;
	int vbus_mv = 0;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	role_get(&r);
	key = k_spin_lock(&self.lock);
	pd = self.pd_connected;
	rp_ma = self.rp_ma;
	rejects = self.rejects;
	waits = self.waits;
	drs = self.drs_pending ? "pending" : self.drs_result;
	prs = self.prs_pending ? "pending" : self.prs_result;
	answer = self.dr_answer;
#ifdef CONFIG_USBC_CSM_DRP
	const char *pr_answer = pr_answer_names[self.pr_answer];
	const char *req_answer = req_answer_names[self.req_answer];
	const char *last_answer = self.last_answer;
	int requests = self.requests;
	int faults = self.faults;
	uint8_t fault_flags = self.fault_flags;

	if (r.source) {
		rp_ma = src_rp_ma_locked();
	}
#else
	const char *pr_answer = "nosupport";
#endif
	k_spin_unlock(&self.lock, key);

	shell_print(sh, "rp: %d mA", rp_ma);
	shell_print(sh, "partner: %s", pd ? "pd" : "not pd");
	print_contract(sh, &r);
	shell_print(sh, "rejects: %d", rejects);
	shell_print(sh, "waits: %d", waits);
	shell_print(sh, "data: %s, answers dr_swap: %s, last dr_swap: %s", r.dfp ? "dfp" : "ufp",
		    dr_answer_names[answer], drs);
	shell_print(sh, "power: %s, answers pr_swap: %s, last pr_swap: %s", power_name(&r),
		    pr_answer, prs);
#ifdef CONFIG_USBC_CSM_DRP
	uint8_t ctrl;
	uint8_t ack;
	uint8_t flags;

	shell_print(sh, "requests: %d, answers request: %s, last answered: %s", requests,
		    req_answer, last_answer);
	ret = tcpp03_regs(&ctrl, &ack, &flags);
	if (ret < 0) {
		shell_print(sh, "pd: error tcpp03 read %d", ret);
		return ret;
	}
	shell_print(sh, "tcpp03: ctrl 0x%02x, ack 0x%02x, flags 0x%02x, faults %d, last 0x%02x",
		    ctrl, ack, flags, faults, fault_flags);
#endif

	ret = usbc_vbus_measure(vbus_dev, &vbus_mv);
	if (ret < 0) {
		shell_print(sh, "pd: error vbus read %d", ret);
		return ret;
	}
	shell_print(sh, "vbus: %d mV", vbus_mv);
	shell_print(sh, "pd: ok");

	return 0;
}

static int cmd_role(const struct shell *sh, size_t argc, char **argv)
{
	struct role r;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	role_get(&r);
	shell_print(sh, "power: %s", power_name(&r));
	shell_print(sh, "data: %s", !r.attached ? "none" : r.dfp ? "dfp" : "ufp");
	print_contract(sh, &r);
	shell_print(sh, "pd: ok");

	return 0;
}

static const char *cc_name(enum tc_cc_voltage_state cc)
{
	switch (cc) {
	case TC_CC_VOLT_OPEN:
		return "open";
	case TC_CC_VOLT_RA:
		return "ra";
	case TC_CC_VOLT_RD:
		return "rd";
	case TC_CC_VOLT_RP_DEF:
		return "rp-default";
	case TC_CC_VOLT_RP_1A5:
		return "rp-1.5a";
	case TC_CC_VOLT_RP_3A0:
		return "rp-3a";
	default:
		return "unknown";
	}
}

/* What each CC line sees from the partner: a cable or path fault shows as open on both */
static int cmd_cc(const struct shell *sh, size_t argc, char **argv)
{
	enum tc_cc_voltage_state cc1;
	enum tc_cc_voltage_state cc2;
	int ret = tcpc_get_cc(tcpc_dev, &cc1, &cc2);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ret < 0) {
		shell_print(sh, "pd: error cc read %d", ret);
		return ret;
	}
	shell_print(sh, "cc1: %s", cc_name(cc1));
	shell_print(sh, "cc2: %s", cc_name(cc2));
	shell_print(sh, "pd: ok");

	return 0;
}

static int cmd_caps(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t caps[PDO_MAX_DATA_OBJECTS];
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	int cnt = self.src_cap_cnt;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	memcpy(caps, self.src_caps, cnt * sizeof(caps[0]));
	k_spin_unlock(&self.lock, key);

	for (int i = 0; i < cnt; i++) {
		print_pdo(sh, i + 1, caps[i]);
	}
	shell_print(sh, "pd: ok");

	return 0;
}

static int dpm_request(const struct shell *sh, enum usbc_policy_request_t req)
{
	int ret = usbc_request(port_dev, req);

	if (ret < 0) {
		shell_print(sh, "pd: error request %d", ret);
		return ret;
	}
	shell_print(sh, "pd: ok");

	return 0;
}

/* A choice is checked against the caps held now, then asked for at once */
static int want_set(const struct shell *sh, const struct want *w)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool source = self.source;
	const char *why = NULL;
	uint32_t rdo = rdo_for_locked(w, &why);

	if (source) {
		rdo = 0;
		why = "the bench is the source";
	}
	if (rdo != 0) {
		self.want = *w;
	}
	k_spin_unlock(&self.lock, key);

	if (rdo == 0) {
		shell_print(sh, "pd: error %s", why);
		return -EINVAL;
	}

	return dpm_request(sh, REQUEST_PE_SNK_REQUEST);
}

static int parse_int(const struct shell *sh, const char *s, int *val)
{
	char *end;
	long v = strtol(s, &end, 10);

	if (*s == '\0' || *end != '\0' || v < 0 || v > 100000) {
		shell_print(sh, "pd: error bad number '%s'", s);
		return -EINVAL;
	}
	*val = (int)v;

	return 0;
}

static int cmd_req(const struct shell *sh, size_t argc, char **argv)
{
	struct want w = {.kind = WANT_FIXED};

	if (parse_int(sh, argv[1], &w.pos) < 0 ||
	    (argc > 2 && parse_int(sh, argv[2], &w.ma) < 0)) {
		return -EINVAL;
	}

	return want_set(sh, &w);
}

static int cmd_pps(const struct shell *sh, size_t argc, char **argv)
{
	struct want w = {.kind = WANT_PPS};

	ARG_UNUSED(argc);
	if (parse_int(sh, argv[1], &w.pos) < 0 || parse_int(sh, argv[2], &w.mv) < 0 ||
	    parse_int(sh, argv[3], &w.ma) < 0) {
		return -EINVAL;
	}

	return want_set(sh, &w);
}

static int cmd_max(const struct shell *sh, size_t argc, char **argv)
{
	struct want w = {.kind = WANT_MAX};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return want_set(sh, &w);
}

static int cmd_keepalive(const struct shell *sh, size_t argc, char **argv)
{
	bool off;

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "on") == 0) {
		off = false;
	} else if (strcmp(argv[1], "off") == 0) {
		off = true;
	} else {
		shell_print(sh, "pd: error keepalive is on or off");
		return -EINVAL;
	}
	self.keepalive_off = off;
	if (!off) {
		k_work_reschedule(&self.keepalive, K_NO_WAIT);
	}
	shell_print(sh, "pd: ok");

	return 0;
}

static int cmd_drswap(const struct shell *sh, size_t argc, char **argv)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	self.drs_pending = true;
	self.awaiting = SWAP_DR;
	k_spin_unlock(&self.lock, key);

	return dpm_request(sh, REQUEST_PE_DR_SWAP);
}

static int cmd_dr(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	for (int i = 0; i < ARRAY_SIZE(dr_answer_names); i++) {
		if (strcmp(argv[1], dr_answer_names[i]) == 0) {
			dr_answer_set(i);
			shell_print(sh, "pd: ok");
			return 0;
		}
	}
	shell_print(sh, "pd: error answer is accept, reject or nosupport");

	return -EINVAL;
}

static int cmd_getcaps(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return dpm_request(sh, REQUEST_PE_GET_SRC_CAPS);
}

static int cmd_hardreset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return dpm_request(sh, REQUEST_PE_HARD_RESET_SEND);
}

static void print_answer(const struct shell *sh, enum ask_kind kind)
{
	uint32_t caps[PDO_MAX_DATA_OBJECTS];
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	struct pd_sdb sdb = self.sdb;
	union pd_ppssdb pps = self.ppssdb;
	union pd_bsdo bsdo = self.bsdo;
	int cnt = self.partner_snk_cap_cnt;

	memcpy(caps, self.partner_snk_caps, cnt * sizeof(caps[0]));
	k_spin_unlock(&self.lock, key);

	switch (kind) {
	case ASK_STATUS:
		shell_print(sh,
			    "status: temp %u C, input 0x%02x, battery input 0x%02x, events 0x%02x, "
			    "temperature 0x%02x, power 0x%02x, state change 0x%02x",
			    sdb.internal_temp, sdb.present_input, sdb.present_battery_input,
			    sdb.event_flags, sdb.temperature_status, sdb.power_status,
			    sdb.power_state_change);
		break;
	case ASK_PPS_STATUS:
		/* -1 where the source does not report it */
		shell_print(sh, "ppsstatus: %d mV %d mA, ptf %u, omf %u",
			    pps.output_voltage == 0xffff ? -1 : (int)pps.output_voltage * 20,
			    pps.output_current == 0xff ? -1 : (int)pps.output_current * 50,
			    (unsigned int)pps.ptf, (unsigned int)pps.omf);
		break;
	case ASK_BATT_STATUS:
		if (bsdo.invalid_battery_reference) {
			shell_print(sh, "battstatus: invalid battery reference");
			break;
		}
		/* Capacity in 0.1 Wh, 65535 when unknown */
		shell_print(sh, "battstatus: present %u, charging %u, capacity %u",
			    (unsigned int)bsdo.battery_present,
			    (unsigned int)bsdo.battery_charging_status,
			    (unsigned int)bsdo.battery_present_capacity);
		break;
	case ASK_SNK_CAPS:
		for (int i = 0; i < cnt; i++) {
			print_pdo(sh, i + 1, caps[i]);
		}
		break;
	default:
		break;
	}
}

/* Asks the partner and waits for its answer, which the stack reports through a callback */
static int cmd_ask(const struct shell *sh, size_t argc, char **argv)
{
	enum ask_kind kind = ASK_NONE;
	k_spinlock_key_t key;
	bool attached;
	bool source;
	bool got;
	int ret;

	ARG_UNUSED(argc);

	for (int i = ASK_STATUS; i < ARRAY_SIZE(asks); i++) {
		if (strcmp(argv[1], asks[i].name) == 0) {
			kind = i;
		}
	}
	if (kind == ASK_NONE) {
		shell_print(sh, "pd: error ask status, ppsstatus, battstatus or snkcaps");
		return -EINVAL;
	}

	key = k_spin_lock(&self.lock);
	attached = self.attached;
	source = self.source;
	k_spin_unlock(&self.lock, key);
	if (!attached) {
		shell_print(sh, "pd: error no partner");
		return -ENOTCONN;
	}
	if (kind == ASK_PPS_STATUS && source) {
		shell_print(sh, "pd: error the bench is the source");
		return -EINVAL;
	}
	if (kind == ASK_SNK_CAPS && !source) {
		shell_print(sh, "pd: error the bench is the sink");
		return -EINVAL;
	}

	k_sem_reset(&self.ask_sem);
	key = k_spin_lock(&self.lock);
	self.ask = kind;
	k_spin_unlock(&self.lock, key);

	ret = usbc_request(port_dev, asks[kind].req);
	if (ret < 0) {
		key = k_spin_lock(&self.lock);
		self.ask = ASK_NONE;
		k_spin_unlock(&self.lock, key);
		shell_print(sh, "pd: error request %d", ret);
		return ret;
	}
	ret = k_sem_take(&self.ask_sem, K_MSEC(ASK_TIMEOUT_MS));
	key = k_spin_lock(&self.lock);
	self.ask = ASK_NONE;
	got = self.ask_got;
	k_spin_unlock(&self.lock, key);

	if (ret < 0) {
		/* The stack drops a request it cannot act on: no Ready state, or PD 2.0 */
		shell_print(sh, "pd: error no answer");
		return ret;
	}
	if (!got) {
		shell_print(sh, "pd: error the partner sent none");
		return -ENODATA;
	}
	print_answer(sh, kind);
	shell_print(sh, "pd: ok");

	return 0;
}

#ifdef CONFIG_USBC_CSM_DRP

static int answer_index(const char *arg, const char *const names[], size_t cnt)
{
	for (int i = 0; i < cnt; i++) {
		if (strcmp(arg, names[i]) == 0) {
			return i;
		}
	}

	return -EINVAL;
}

static int cmd_prswap(const struct shell *sh, size_t argc, char **argv)
{
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	bool attached = self.attached;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (attached) {
		self.prs_pending = true;
		self.awaiting = SWAP_PR;
		self.swap_until = k_uptime_get() + SWAP_WINDOW_MS;
	}
	k_spin_unlock(&self.lock, key);

	if (!attached) {
		shell_print(sh, "pd: error no partner");
		return -ENOTCONN;
	}

	ret = dpm_request(sh, REQUEST_PE_PR_SWAP);
	if (ret < 0) {
		key = k_spin_lock(&self.lock);
		self.prs_pending = false;
		self.swap_until = 0;
		k_spin_unlock(&self.lock, key);
	}

	return ret;
}

static int cmd_pr(const struct shell *sh, size_t argc, char **argv)
{
	int answer;

	ARG_UNUSED(argc);

	if (strcmp(argv[1], "wait") == 0) {
		shell_print(sh, "pd: error the stack answers PR_Swap with Accept or Reject only");
		return -ENOTSUP;
	}
	answer = answer_index(argv[1], pr_answer_names, ARRAY_SIZE(pr_answer_names));
	if (answer < 0) {
		shell_print(sh, "pd: error answer is accept or reject");
		return -EINVAL;
	}
	self.pr_answer = answer;
	shell_print(sh, "pd: ok");

	return 0;
}

static int cmd_answer(const struct shell *sh, size_t argc, char **argv)
{
	int answer;

	ARG_UNUSED(argc);

	answer = answer_index(argv[1], req_answer_names, ARRAY_SIZE(req_answer_names));
	if (answer < 0) {
		shell_print(sh, "pd: error answer is accept, reject or wait");
		return -EINVAL;
	}
	self.req_answer = answer;
	shell_print(sh, "pd: ok");

	return 0;
}

/* One <mV>:<mA> PDO, a fixed one of the devicetree's at no more than its current */
static int offer_pdo(const struct shell *sh, char *arg, int prev_mv, uint32_t *pdo, int *mv)
{
	char *colon = strchr(arg, ':');
	int ma;

	if (colon == NULL) {
		shell_print(sh, "pd: error a PDO is <mV>:<mA>");
		return -EINVAL;
	}
	*colon = '\0';
	if (parse_int(sh, arg, mv) < 0 || parse_int(sh, colon + 1, &ma) < 0) {
		return -EINVAL;
	}
	if (prev_mv == 0 && *mv != 5000) {
		shell_print(sh, "pd: error the first PDO is 5000 mV");
		return -EINVAL;
	}
	if (*mv <= prev_mv) {
		shell_print(sh, "pd: error voltages rise PDO by PDO");
		return -EINVAL;
	}
	for (int i = 0; i < ARRAY_SIZE(offer_max); i++) {
		if (!pdo_is_fixed(offer_max[i]) || fixed_mv(offer_max[i]) != *mv) {
			continue;
		}
		if (ma < 10 || ma > fixed_ma(offer_max[i])) {
			shell_print(sh, "pd: error %d mV offers 10 to %d mA", *mv,
				    fixed_ma(offer_max[i]));
			return -EINVAL;
		}
		*pdo = (offer_max[i] & ~(PDO_CURR_MASK << PDO_FIXED_CURR_SHIFT)) |
		       PDO_FIXED_CURR(ma);
		return 0;
	}
	shell_print(sh, "pd: error %d mV not in the devicetree's source-pdos", *mv);

	return -EINVAL;
}

/* Lists the bench's Source_Capabilities, or sets them and sends them to a sink attached */
static int cmd_srccaps(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t offer[PDO_MAX_DATA_OBJECTS];
	k_spinlock_key_t key;
	int cnt = argc - 1;
	int mv = 0;
	bool send;

	if (cnt == 0) {
		key = k_spin_lock(&self.lock);
		cnt = self.offer_cnt;
		memcpy(offer, self.offer, cnt * sizeof(offer[0]));
		k_spin_unlock(&self.lock, key);
		for (int i = 0; i < cnt; i++) {
			print_pdo(sh, i + 1, offer[i]);
		}
		shell_print(sh, "pd: ok");
		return 0;
	}

	for (int i = 0; i < cnt; i++) {
		if (offer_pdo(sh, argv[i + 1], mv, &offer[i], &mv) < 0) {
			return -EINVAL;
		}
	}
	offer[0] = drd_bit(offer[0], self.dr_answer);

	key = k_spin_lock(&self.lock);
	memcpy(self.offer, offer, cnt * sizeof(offer[0]));
	self.offer_cnt = cnt;
	send = self.attached && self.source;
	k_spin_unlock(&self.lock, key);

	if (!send) {
		shell_print(sh, "pd: ok");
		return 0;
	}

	return dpm_request(sh, REQUEST_PE_SEND_SRC_CAPS);
}

#define DRP_CMD(fn) fn

#else

static int cmd_not_drp(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "pd: error not a DRP build");

	return -ENOTSUP;
}

#define DRP_CMD(fn) cmd_not_drp

#endif /* CONFIG_USBC_CSM_DRP */

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pd,
	SHELL_CMD_ARG(status, NULL, "Rp, partner, contract, roles and VBUS", cmd_status, 1, 0),
	SHELL_CMD_ARG(role, NULL, "Power role, data role and contract", cmd_role, 1, 0),
	SHELL_CMD_ARG(caps, NULL, "The partner's Source_Capabilities", cmd_caps, 1, 0),
	SHELL_CMD_ARG(cc, NULL, "What each CC line sees from the partner", cmd_cc, 1, 0),
	SHELL_CMD_ARG(req, NULL, "Ask a fixed PDO: pd req <pos> [mA]", cmd_req, 2, 1),
	SHELL_CMD_ARG(pps, NULL, "Ask a PPS APDO: pd pps <pos> <mV> <mA>", cmd_pps, 4, 0),
	SHELL_CMD_ARG(max, NULL, "Ask the highest power fixed PDO (the default)", cmd_max, 1, 0),
	SHELL_CMD_ARG(getcaps, NULL, "Send Get_Source_Cap", cmd_getcaps, 1, 0),
	SHELL_CMD_ARG(hardreset, NULL, "Send Hard Reset Signaling", cmd_hardreset, 1, 0),
	SHELL_CMD_ARG(drswap, NULL, "Send DR_Swap", cmd_drswap, 1, 0),
	SHELL_CMD_ARG(keepalive, NULL, "PPS keep-alive Requests: pd keepalive <on|off>",
		      cmd_keepalive, 2, 0),
	SHELL_CMD_ARG(dr, NULL, "Answer a DR_Swap: pd dr <accept|reject|nosupport>", cmd_dr, 2,
		      0),
	SHELL_CMD_ARG(ask, NULL, "Ask the partner: pd ask <status|ppsstatus|battstatus|snkcaps>",
		      cmd_ask, 2, 0),
	SHELL_CMD_ARG(prswap, NULL, "Send PR_Swap (DRP build)", DRP_CMD(cmd_prswap), 1, 0),
	SHELL_CMD_ARG(pr, NULL, "Answer a PR_Swap: pd pr <accept|reject> (DRP build)",
		      DRP_CMD(cmd_pr), 2, 0),
	SHELL_CMD_ARG(answer, NULL,
		      "Answer a sink's Request: pd answer <accept|reject|wait> (DRP build)",
		      DRP_CMD(cmd_answer), 2, 0),
	SHELL_CMD_ARG(srccaps, NULL,
		      "List or set the offer: pd srccaps [<mV>:<mA> ...] (DRP build)",
		      DRP_CMD(cmd_srccaps), 1, PDO_MAX_DATA_OBJECTS),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pd, &sub_pd, "PD bench", NULL);

int main(void)
{
	int ret;

	__ASSERT(device_is_ready(port_dev), "USB-C port not ready");
	__ASSERT(device_is_ready(vbus_dev), "VBUS monitor not ready");

#ifdef CONFIG_USBC_CSM_DRP
	__ASSERT(device_is_ready(vprov_dev), "CN3 monitor not ready");
	/* The provider path switches CN3 onto VBUS unregulated: it can only be vSafe5V */
	for (int i = 0; i < ARRAY_SIZE(offer_max); i++) {
		__ASSERT(pdo_is_fixed(offer_max[i]) && fixed_mv(offer_max[i]) == 5000,
			 "source PDO %d is not a fixed 5 V PDO", i + 1);
	}
	/* The TCPP03 leaves its dead-battery Rd to the UCPD before the stack starts */
	ret = tcpp03_init(tcpp_fault);
	__ASSERT(ret == 0, "TCPP03: %d", ret);
#else
	__ASSERT(gpio_is_ready_dt(&tcpp_vcc) && gpio_is_ready_dt(&tcpp_db), "TCPP01 GPIOs not ready");

	/* The TCPP01 is powered and hands CC to the UCPD before the stack starts */
	ret = gpio_pin_configure_dt(&tcpp_vcc, GPIO_OUTPUT_ACTIVE);
	__ASSERT(ret == 0, "TCPP01 VCC: %d", ret);
	ret = gpio_pin_configure_dt(&tcpp_db, GPIO_OUTPUT_ACTIVE);
	__ASSERT(ret == 0, "TCPP01 DB: %d", ret);
#endif

	k_work_init_delayable(&self.keepalive, keepalive_handler);
	k_sem_init(&self.ask_sem, 0, 1);
	self.drs_result = "none";
	self.prs_result = "none";
#ifdef CONFIG_USBC_CSM_DRP
	self.last_answer = "none";
#endif
	dr_answer_set(DR_ACCEPT);

	usbc_set_policy_cb_check(port_dev, policy_check);
	usbc_set_policy_cb_notify(port_dev, policy_notify);
	usbc_set_policy_cb_wait_notify(port_dev, policy_wait_notify);
	usbc_set_policy_cb_get_snk_cap(port_dev, policy_get_snk_cap);
	usbc_set_policy_cb_set_src_cap(port_dev, policy_set_src_cap);
	usbc_set_policy_cb_get_rdo(port_dev, policy_get_rdo);
	usbc_set_policy_cb_set_status(port_dev, policy_set_status);
	usbc_set_policy_cb_set_pps_status(port_dev, policy_set_pps_status);
	usbc_set_policy_cb_set_battery_status(port_dev, policy_set_battery_status);
#ifdef CONFIG_USBC_CSM_DRP
	usbc_set_policy_cb_src_en(port_dev, policy_src_en);
	usbc_set_policy_cb_get_src_rp(port_dev, policy_get_src_rp);
	usbc_set_policy_cb_get_src_caps(port_dev, policy_get_src_caps);
	usbc_set_policy_cb_check_sink_request(port_dev, policy_check_sink_request);
	usbc_set_policy_cb_present_contract_is_valid(port_dev, policy_present_contract_is_valid);
	usbc_set_policy_cb_is_ps_ready(port_dev, policy_is_ps_ready);
	usbc_set_policy_cb_set_port_partner_snk_cap(port_dev, policy_set_port_partner_snk_cap);
#endif

	ret = usbc_start(port_dev);
	__ASSERT(ret == 0, "USB-C start: %d", ret);

	LOG_INF("PD bench ready");

	return 0;
}
