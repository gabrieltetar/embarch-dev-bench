/* PD bench: the pure half of the app, PDO and RDO arithmetic and the shell's choice rules.
 *
 * No shell, kernel or device call lives here, so tests/logic runs it on
 * native_sim. main.c keeps the locking, the stack callbacks and every line the
 * shell prints.
 */

#ifndef PD_BENCH_PD_LOGIC_H_
#define PD_BENCH_PD_LOGIC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/usb_c/usbc_pd.h>

/* `pd max` never asks for more than a cable without an e-marker carries */
#define PD_LOGIC_CABLE_MA     3000
/* What a 5 V Request asks for when nothing better is valid */
#define PD_LOGIC_SAFE_MA      500
/* A Source_Capabilities' first PDO is the vSafe5V one */
#define PD_LOGIC_FIRST_PDO_MV 5000
/* The least current `pd srccaps` offers in a PDO */
#define PD_LOGIC_OFFER_MIN_MA 10
/* The largest number the shell takes */
#define PD_LOGIC_NUMBER_MAX   100000

/* A PPS RDO's Output Voltage is in 20 mV units; usbc_pd.h has no macro for it */
#define PD_LOGIC_PPS_RDO_MV_PER_UNIT       20
#define PD_LOGIC_MV_TO_PPS_RDO_VOLTAGE(mv) ((mv) / PD_LOGIC_PPS_RDO_MV_PER_UNIT)
#define PD_LOGIC_PPS_RDO_VOLTAGE_TO_MV(v)  ((v) * PD_LOGIC_PPS_RDO_MV_PER_UNIT)

/* What the bench asks for as a sink */
enum pd_logic_want_kind {
	/* The highest power fixed PDO, at most PD_LOGIC_CABLE_MA: `pd max` */
	PD_LOGIC_WANT_HIGHEST_POWER,
	/* A fixed PDO, at its current or less: `pd req` */
	PD_LOGIC_WANT_FIXED,
	/* A PPS APDO at a voltage and current: `pd pps` */
	PD_LOGIC_WANT_PPS,
};

struct pd_logic_want {
	enum pd_logic_want_kind kind;
	/* 1-based object position; 0 mA for a fixed PDO means its own current */
	int pos;
	int mv;
	int ma;
};

/* Why `pd srccaps` refused one <mV>:<mA> PDO */
enum pd_logic_offer_err {
	PD_LOGIC_OFFER_OK,
	PD_LOGIC_OFFER_NO_COLON,
	PD_LOGIC_OFFER_BAD_NUMBER,
	PD_LOGIC_OFFER_FIRST_NOT_5V,
	PD_LOGIC_OFFER_NOT_RISING,
	PD_LOGIC_OFFER_CURRENT_OUT_OF_RANGE,
	PD_LOGIC_OFFER_NOT_IN_DEVICETREE,
	PD_LOGIC_OFFER_ERR_MAX,
};

/* A refusal: the reason, the text that was not a number, the voltage and the most it offers */
struct pd_logic_offer_refusal {
	enum pd_logic_offer_err err;
	const char *bad_number;
	int mv;
	int max_ma;
};

/* How the bench answers a partner's DR_Swap, PR_Swap or a sink's Request */
enum pd_logic_reply {
	PD_LOGIC_REPLY_ACCEPT,
	PD_LOGIC_REPLY_REJECT,
};

/* What `pd idle` puts back: every setting a test may have changed, as a build leaves them at boot */
struct pd_logic_idle {
	/* A PPS contract gets its keep-alive Request (`pd keepalive on`) */
	bool keepalive_on;
	/* The answers to DR_Swap, PR_Swap and a Request (`pd dr`, `pd pr`, `pd answer`) */
	enum pd_logic_reply dr_reply;
	enum pd_logic_reply pr_reply;
	enum pd_logic_reply request_reply;
	/* What the sink asks for: the highest power fixed PDO, as `pd max` */
	enum pd_logic_want_kind want_kind;
	/* The Source_Capabilities are the devicetree's, as `pd srccaps` with no PDO would list */
	bool offer_from_devicetree;
	/* Any contract in force is dropped */
	bool drop_contract;
	/* VBUS is off and discharged (the DRP1M1, which sources) */
	bool vbus_off;
};

/* The settings `pd idle` restores; `drp` is a build that sources, which also turns VBUS off */
void PdLogicIdle(bool drp, struct pd_logic_idle *idle);

/* Whether a PDO is a fixed supply PDO */
bool PdLogicPdoIsFixed(uint32_t pdo);

/* Whether a PDO is a PPS APDO */
bool PdLogicPdoIsPps(uint32_t pdo);

/* A fixed PDO's voltage in mV */
int PdLogicFixedMv(uint32_t pdo);

/* A fixed PDO's maximum current in mA */
int PdLogicFixedMa(uint32_t pdo);

/* The RDO asking fixed PDO `pos` for `ma` operating and maximum current, No USB Suspend set */
uint32_t PdLogicRdoFixed(int pos, int ma);

/* The RDO asking PPS APDO `pos` for `mv` and `ma`, No USB Suspend set */
uint32_t PdLogicRdoPps(int pos, int mv, int ma);

/* The 1-based object position an RDO names */
int PdLogicRdoPos(uint32_t rdo);

/* `pd max`: the highest power fixed PDO at most PD_LOGIC_CABLE_MA, ties to the lower voltage */
uint32_t PdLogicRdoHighestPower(const uint32_t *caps, int cnt);

/* The RDO for a choice against the caps held, or 0 with the reason it is not valid in `why` */
uint32_t PdLogicRdoFor(const uint32_t *caps, int cnt, const struct pd_logic_want *want,
		       const char **why);

/* The source's Request check: a fixed PDO whose current covers the Request's operating current */
bool PdLogicRequestFits(uint32_t pdo, uint32_t request);

/* A shell number: decimal digits only, 0 to PD_LOGIC_NUMBER_MAX; -EINVAL otherwise */
int PdLogicParseInt(const char *s, int *val);

/*
 * One `pd srccaps` PDO, <mV>:<mA>: the first at PD_LOGIC_FIRST_PDO_MV, each above `prev_mv`
 * (0 for the first), at a voltage `offer_max` lists and within its current. Writes the PDO
 * and its voltage, or returns -EINVAL with the refusal. Cuts `arg` at its colon.
 */
int PdLogicOfferPdo(char *arg, int prev_mv, const uint32_t *offer_max, size_t offer_max_cnt,
		    uint32_t *pdo, int *mv, struct pd_logic_offer_refusal *refusal);

/* A PD event the stack reports to the app, stamped with the bench's own clock as it is seen */
enum pd_logic_event {
	/* A partner attached: the Policy Engine started in a power role */
	PD_LOGIC_EVENT_ATTACH,
	/* Source_Capabilities received (sink role) */
	PD_LOGIC_EVENT_SRC_CAPS,
	/* The stack asked the app for a Request, which it sends */
	PD_LOGIC_EVENT_REQUEST_SENT,
	/* The Request was answered with Accept */
	PD_LOGIC_EVENT_ACCEPT,
	/* PS_RDY: the contract is in force */
	PD_LOGIC_EVENT_PS_RDY,
	/* A Hard Reset was requested of the stack */
	PD_LOGIC_EVENT_HARD_RESET_SENT,
	/* A Hard Reset arrived from the partner */
	PD_LOGIC_EVENT_HARD_RESET_RECEIVED,
	/* The PPS keep-alive Request was due and skipped (`pd keepalive off`) */
	PD_LOGIC_EVENT_KEEPALIVE_STOPPED,
	PD_LOGIC_EVENT_MAX,
};

/* The intervals a PD timing test needs, each from the last event of one kind to the last of another */
enum pd_logic_interval {
	PD_LOGIC_INTERVAL_ATTACH_TO_SRC_CAPS,
	PD_LOGIC_INTERVAL_REQUEST_TO_PS_RDY,
	PD_LOGIC_INTERVAL_KEEPALIVE_STOPPED_TO_HARD_RESET,
	PD_LOGIC_INTERVAL_MAX,
};

/* How many of the latest events `pd times` lists */
#define PD_LOGIC_TIMES_KEPT 8

struct pd_logic_stamp {
	enum pd_logic_event event;
	uint64_t us;
};

/* The latest events in order, and the last stamp of each kind */
struct pd_logic_times {
	struct pd_logic_stamp kept[PD_LOGIC_TIMES_KEPT];
	/* Events added since the clear, which may exceed what is kept */
	int total;
	uint64_t last_us[PD_LOGIC_EVENT_MAX];
	bool seen[PD_LOGIC_EVENT_MAX];
};

/* Forgets every event (`pd idle`) */
void PdLogicTimesClear(struct pd_logic_times *t);

/* Records `event` seen at `us` microseconds of the bench's uptime */
void PdLogicTimesAdd(struct pd_logic_times *t, enum pd_logic_event event, uint64_t us);

/* How many events `PdLogicTimesGet` has */
int PdLogicTimesCount(const struct pd_logic_times *t);

/* The `i`th kept event, 0 the oldest */
struct pd_logic_stamp PdLogicTimesGet(const struct pd_logic_times *t, int i);

/* An event's name as `pd times` prints it */
const char *PdLogicEventName(enum pd_logic_event event);

/* An interval's name as `pd times` prints it */
const char *PdLogicIntervalName(enum pd_logic_interval interval);

/* An interval in microseconds; false when an end is missing or the start is after the end */
bool PdLogicInterval(const struct pd_logic_times *t, enum pd_logic_interval interval,
		     uint64_t *us);

#endif /* PD_BENCH_PD_LOGIC_H_ */
