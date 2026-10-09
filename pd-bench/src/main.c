/* PD bench: a USB Power Delivery sink the host scripts over the shell.
 *
 * embarch-host-study-proposal.md, phase 1. This is not the dev-bench wire:
 * the board's console carries a Zephyr shell, and the host drives it with
 * Core's signal exchange (embarch-core decision 79). Every `pd` command ends
 * its output with one line, `pd: ok` or `pd: error <reason>`, the marker an
 * exchange waits for. What the sink asks for is set by the host and used the
 * next time the stack builds a Request; the contract that follows is read
 * back with `pd status`, never assumed.
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/usbc_pd.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/dt-bindings/usb-c/pd.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb_c/usbc.h>

LOG_MODULE_REGISTER(pd_bench, LOG_LEVEL_INF);

#define PORT_NODE DT_ALIAS(usbc_port0)
#define USER_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_ENUM_HAS_VALUE(PORT_NODE, power_role, sink), "the PD bench is a sink");

/* A PPS Sink sends a Request at least every tPPSRequest, 10 s (USB PD R3.2 p237) */
#define PPS_KEEPALIVE_MS 8000
/* What a 5 V Request asks for when nothing better is valid */
#define SAFE_MA 500
/* `pd max` never asks for more than a cable without an e-marker carries */
#define CABLE_MA 3000

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

static const struct device *const port_dev = DEVICE_DT_GET(PORT_NODE);
static const struct device *const vbus_dev = DEVICE_DT_GET(DT_PHANDLE(PORT_NODE, vbus));
static const struct gpio_dt_spec tcpp_vcc = GPIO_DT_SPEC_GET(USER_NODE, tcpp_vcc_gpios);
static const struct gpio_dt_spec tcpp_db = GPIO_DT_SPEC_GET(USER_NODE, tcpp_db_gpios);

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
	bool pd_connected;
	int rp_ma;
	int rejects;

	struct k_work_delayable keepalive;
} self = {
	.snk_caps = DT_PROP(PORT_NODE, sink_pdos),
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
	bool pps = self.contract && pdo_is_pps(self.contract_pdo);

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

static void policy_notify(const struct device *dev, const enum usbc_policy_notify_t notify)
{
	k_spinlock_key_t key;

	ARG_UNUSED(dev);

	switch (notify) {
	case TRANSITION_PS:
		key = k_spin_lock(&self.lock);
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
		key = k_spin_lock(&self.lock);
		self.rp_ma = 0;
		self.src_cap_cnt = 0;
		self.pd_connected = false;
		k_spin_unlock(&self.lock, key);
		contract_end();
		LOG_INF("detached");
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
		self.rejects++;
		k_spin_unlock(&self.lock, key);
		LOG_WRN("Request rejected");
		break;
	case SNK_TRANSITION_TO_DEFAULT:
		contract_end();
		LOG_WRN("Hard Reset");
		break;
	case PORT_PARTNER_NOT_RESPONSIVE:
		LOG_WRN("partner is not PD capable");
		break;
	default:
		break;
	}
}

static bool policy_check(const struct device *dev, const enum usbc_policy_check_t check)
{
	ARG_UNUSED(dev);

	return check == CHECK_SNK_AT_DEFAULT_LEVEL;
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
	k_spinlock_key_t key = k_spin_lock(&self.lock);
	union pd_rdo rdo = {.raw_value = self.contract_rdo};
	uint32_t pdo = self.contract_pdo;
	bool contract = self.contract;
	bool pd = self.pd_connected;
	int rp_ma = self.rp_ma;
	int rejects = self.rejects;
	int vbus_mv = 0;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_spin_unlock(&self.lock, key);

	shell_print(sh, "rp: %d mA", rp_ma);
	shell_print(sh, "partner: %s", pd ? "pd" : "not pd");
	if (!contract) {
		shell_print(sh, "contract: none");
	} else if (pdo_is_pps(pdo)) {
		shell_print(sh, "contract: pps pos %d %d mV %d mA", rdo.augmented.object_pos,
			    rdo.augmented.output_voltage * 20,
			    PD_CONVERT_AUGMENTED_PDO_CURRENT_TO_MA(rdo.augmented.operating_current));
	} else {
		shell_print(sh, "contract: fixed pos %d %d mV %d mA", rdo.fixed.object_pos,
			    fixed_mv(pdo),
			    PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(rdo.fixed.operating_current));
	}
	shell_print(sh, "rejects: %d", rejects);

	ret = usbc_vbus_measure(vbus_dev, &vbus_mv);
	if (ret < 0) {
		shell_print(sh, "pd: error vbus read %d", ret);
		return ret;
	}
	shell_print(sh, "vbus: %d mV", vbus_mv);
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
	const char *why = NULL;
	uint32_t rdo = rdo_for_locked(w, &why);

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

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pd,
	SHELL_CMD_ARG(status, NULL, "Rp, partner, contract and VBUS", cmd_status, 1, 0),
	SHELL_CMD_ARG(caps, NULL, "The partner's Source_Capabilities", cmd_caps, 1, 0),
	SHELL_CMD_ARG(req, NULL, "Ask a fixed PDO: pd req <pos> [mA]", cmd_req, 2, 1),
	SHELL_CMD_ARG(pps, NULL, "Ask a PPS APDO: pd pps <pos> <mV> <mA>", cmd_pps, 4, 0),
	SHELL_CMD_ARG(max, NULL, "Ask the highest power fixed PDO (the default)", cmd_max, 1, 0),
	SHELL_CMD_ARG(getcaps, NULL, "Send Get_Source_Cap", cmd_getcaps, 1, 0),
	SHELL_CMD_ARG(hardreset, NULL, "Send Hard Reset Signaling", cmd_hardreset, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pd, &sub_pd, "PD bench sink", NULL);

int main(void)
{
	int ret;

	__ASSERT(device_is_ready(port_dev), "USB-C port not ready");
	__ASSERT(device_is_ready(vbus_dev), "VBUS monitor not ready");
	__ASSERT(gpio_is_ready_dt(&tcpp_vcc) && gpio_is_ready_dt(&tcpp_db), "TCPP01 GPIOs not ready");

	/* The TCPP01 is powered and hands CC to the UCPD before the stack starts */
	ret = gpio_pin_configure_dt(&tcpp_vcc, GPIO_OUTPUT_ACTIVE);
	__ASSERT(ret == 0, "TCPP01 VCC: %d", ret);
	ret = gpio_pin_configure_dt(&tcpp_db, GPIO_OUTPUT_ACTIVE);
	__ASSERT(ret == 0, "TCPP01 DB: %d", ret);

	k_work_init_delayable(&self.keepalive, keepalive_handler);

	usbc_set_policy_cb_check(port_dev, policy_check);
	usbc_set_policy_cb_notify(port_dev, policy_notify);
	usbc_set_policy_cb_get_snk_cap(port_dev, policy_get_snk_cap);
	usbc_set_policy_cb_set_src_cap(port_dev, policy_set_src_cap);
	usbc_set_policy_cb_get_rdo(port_dev, policy_get_rdo);

	ret = usbc_start(port_dev);
	__ASSERT(ret == 0, "USB-C start: %d", ret);

	LOG_INF("PD bench ready");

	return 0;
}
