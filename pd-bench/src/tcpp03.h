/* TCPP03-M20 USB Type-C port protection, driven over I2C: the X-NUCLEO-DRP1M1's
 * provider (source) and consumer (sink) paths, VBUS discharge and fault flags.
 */

#ifndef PD_BENCH_TCPP03_H_
#define PD_BENCH_TCPP03_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

/* Register 2 fault flags; any of them opens both paths until the recovery word */
#define TCPP03_FLAG_OCP_VCONN BIT(0)
#define TCPP03_FLAG_OCP_VBUS  BIT(1)
#define TCPP03_FLAG_OVP_VBUS  BIT(2)
#define TCPP03_FLAG_OVP_CC    BIT(3)
#define TCPP03_FLAG_OTP       BIT(4)
#define TCPP03_FLAG_VBUS_OK   BIT(5)

/* Called from the system work queue after a fault, the flags already cleared */
typedef void (*tcpp03_fault_cb_t)(uint8_t flags);

/* Enable the TCPP03, clear any fault and leave it idle in Normal mode */
int tcpp03_init(tcpp03_fault_cb_t cb);

/*
 * On, the consumer path opens, then the provider path closes onto VBUS. Off,
 * the provider path opens and VBUS discharges until tcpp03_idle().
 */
int tcpp03_source(bool on);

/* No sourcing, no discharge, the consumer path closed: the sink's and the toggling port's state */
int tcpp03_idle(void);

/* The control byte last written, its acknowledge register and the fault flags */
int tcpp03_regs(uint8_t *ctrl, uint8_t *ack, uint8_t *flags);

#endif /* PD_BENCH_TCPP03_H_ */
