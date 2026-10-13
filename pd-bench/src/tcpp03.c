/* TCPP03-M20 on the X-NUCLEO-DRP1M1, over I2C.
 *
 * Register map and sequences from ST's stm32-tcpp0203 driver (tcpp0203.h) and
 * the X-CUBE-TCPP DRP1M1 BSP (drp1m1_usbpd_pwr.c). Register 0 is the control
 * byte, written whole; register 1 acknowledges it; register 2 holds the fault
 * flags. Normal mode throughout: the TCPP03's dead-battery Rd is off, so the
 * UCPD toggles Rp and Rd itself, and FLGn falls only on a fault. A fault opens
 * both paths; only the recovery word clears it (UM2891 1.9).
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "tcpp03.h"

LOG_MODULE_REGISTER(tcpp03, LOG_LEVEL_INF);

#define TCPP03_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(st_tcpp03)

#define REG_CTRL  0x00
#define REG_ACK   0x01
#define REG_FLAGS 0x02

/* Register 0: GDP closed connects the provider path, GDC set opens the consumer path */
#define CTRL_GDP_CLOSED     BIT(2)
#define CTRL_GDC_OPEN       BIT(3)
#define CTRL_MODE_NORMAL    (1 << 4)
#define CTRL_VBUS_DISCHARGE BIT(6)

/* Both paths open: also the recovery word for Normal mode (UM2891 1.9) */
#define CTRL_OPEN       (CTRL_MODE_NORMAL | CTRL_GDC_OPEN)
#define CTRL_IDLE       CTRL_MODE_NORMAL
#define CTRL_SOURCE     (CTRL_MODE_NORMAL | CTRL_GDC_OPEN | CTRL_GDP_CLOSED)
#define CTRL_SOURCE_OFF (CTRL_MODE_NORMAL | CTRL_GDC_OPEN | CTRL_VBUS_DISCHARGE)

/* Register 2 bit 7 is set on a TCPP02, which has no consumer gate driver */
#define FLAG_TCPP02 BIT(7)
#define FLAG_FAULTS                                                                                \
	(TCPP03_FLAG_OCP_VCONN | TCPP03_FLAG_OCP_VBUS | TCPP03_FLAG_OVP_VBUS |                     \
	 TCPP03_FLAG_OVP_CC | TCPP03_FLAG_OTP)

/* ST's BSP waits this long between opening the consumer path and closing the provider path */
#define GATE_SWAP_MS 2

static const struct i2c_dt_spec i2c = I2C_DT_SPEC_GET(TCPP03_NODE);
static const struct gpio_dt_spec enable = GPIO_DT_SPEC_GET(TCPP03_NODE, enable_gpios);
static const struct gpio_dt_spec flg = GPIO_DT_SPEC_GET(TCPP03_NODE, flg_gpios);

static K_MUTEX_DEFINE(lock);
static uint8_t ctrl;
static tcpp03_fault_cb_t fault_cb;
static struct gpio_callback flg_cb;
static struct k_work fault_work;

static int ctrl_write(uint8_t val)
{
	int ret = i2c_reg_write_byte_dt(&i2c, REG_CTRL, val);

	if (ret == 0) {
		ctrl = val;
	}

	return ret;
}

static void fault_handler(struct k_work *work)
{
	uint8_t flags = 0;
	int ret;

	ARG_UNUSED(work);

	k_mutex_lock(&lock, K_FOREVER);
	ret = i2c_reg_read_byte_dt(&i2c, REG_FLAGS, &flags);
	if (ret == 0 && (flags & FLAG_FAULTS) != 0) {
		ret = ctrl_write(CTRL_OPEN);
		if (ret == 0) {
			ret = ctrl_write(CTRL_IDLE);
		}
	}
	k_mutex_unlock(&lock);

	if (ret == 0 && (flags & FLAG_FAULTS) == 0) {
		LOG_WRN("FLGn fell without a fault flag (0x%02x)", flags);
		return;
	}
	if (ret < 0) {
		LOG_ERR("fault flags not read or cleared: %d", ret);
	} else {
		LOG_ERR("fault, flags 0x%02x: both paths open", flags);
	}
	fault_cb(flags & FLAG_FAULTS);
}

static void flg_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_work_submit(&fault_work);
}

int tcpp03_init(tcpp03_fault_cb_t cb)
{
	uint8_t flags;
	int ret;

	if (!i2c_is_ready_dt(&i2c) || !gpio_is_ready_dt(&enable) || !gpio_is_ready_dt(&flg)) {
		return -ENODEV;
	}

	fault_cb = cb;
	k_work_init(&fault_work, fault_handler);

	ret = gpio_pin_configure_dt(&enable, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}
	/* The I2C interface is powered from ENABLE */
	k_msleep(1);

	ret = i2c_reg_read_byte_dt(&i2c, REG_FLAGS, &flags);
	if (ret < 0) {
		return ret;
	}
	if (flags & FLAG_TCPP02) {
		LOG_ERR("a TCPP02, not a TCPP03");
		return -ENODEV;
	}

	/* A fault latched before an MCU reset survives it: the recovery word clears it */
	ret = ctrl_write(CTRL_OPEN);
	if (ret == 0) {
		ret = ctrl_write(CTRL_IDLE);
	}
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&flg, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}
	gpio_init_callback(&flg_cb, flg_isr, BIT(flg.pin));
	ret = gpio_add_callback(flg.port, &flg_cb);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&flg, GPIO_INT_EDGE_TO_ACTIVE);
}

int tcpp03_source(bool on)
{
	int ret;

	k_mutex_lock(&lock, K_FOREVER);
	if (on) {
		/* The TCPP03 refuses both paths closed; ST's BSP opens the consumer path first */
		ret = ctrl_write(CTRL_OPEN);
		if (ret == 0) {
			k_msleep(GATE_SWAP_MS);
			ret = ctrl_write(CTRL_SOURCE);
		}
	} else {
		ret = ctrl_write(CTRL_SOURCE_OFF);
	}
	k_mutex_unlock(&lock);

	return ret;
}

int tcpp03_idle(void)
{
	int ret;

	k_mutex_lock(&lock, K_FOREVER);
	ret = ctrl_write(CTRL_IDLE);
	k_mutex_unlock(&lock);

	return ret;
}

int tcpp03_regs(uint8_t *ctrl_out, uint8_t *ack, uint8_t *flags)
{
	int ret;

	k_mutex_lock(&lock, K_FOREVER);
	*ctrl_out = ctrl;
	ret = i2c_reg_read_byte_dt(&i2c, REG_ACK, ack);
	if (ret == 0) {
		ret = i2c_reg_read_byte_dt(&i2c, REG_FLAGS, flags);
	}
	k_mutex_unlock(&lock);

	return ret;
}
