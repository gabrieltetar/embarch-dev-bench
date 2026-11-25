/*
 * The dev bench's native USB device, on Zephyr's "next" USB device stack
 * (CONFIG_USB_DEVICE_STACK_NEXT, subsys/usb/device_next).
 *
 * One composite device, three CDC ACM ports. Each port's interface string is
 * the `label` of its `zephyr,cdc-acm-uart` node in the board overlay, which
 * is also where the order is fixed:
 *   cdc_acm_uart0  control   (the Core link, `embarch,control-link`)
 *   cdc_acm_uart1  samples
 *   cdc_acm_uart2  SMP       (`zephyr,uart-mcumgr`, image updates through
 *                             MCUboot)
 *
 * Identity: usb_ids.h. Serial = the chip's unique ID; the PID's low byte is
 * that ID's last byte, so the PID is only known at run time, after the
 * hardware ID is read, and is set with usbd_device_set_pid() before init.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

#include "usb_device.h"
#include "usb_ids.h"

LOG_MODULE_REGISTER(usb_device, LOG_LEVEL_INF);

#define USB_MANUFACTURER "EmbArch"
#define USB_PRODUCT "EmbArch dev bench"
#define USB_MAX_POWER_2MA 125 /* bMaxPower counts 2 mA units: 250 mA */

/* Miscellaneous class, common subclass, Interface Association Descriptor
 * protocol: the device class triple USB-IF assigns to composite devices that
 * use IADs. */
#define USB_MISC_SUBCLASS_COMMON 0x02
#define USB_MISC_PROTOCOL_IAD 0x01

/* The VID is fixed; the PID's low byte is filled in by usb_device_start(). */
USBD_DEVICE_DEFINE(dev_bench_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), EMBARCH_USB_VID,
		   (EMBARCH_USB_DEVICE_TYPE_DEV_BENCH << 8));

USBD_DESC_LANG_DEFINE(dev_bench_lang);
USBD_DESC_MANUFACTURER_DEFINE(dev_bench_mfr, USB_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(dev_bench_product, USB_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(dev_bench_sn);
USBD_DESC_CONFIG_DEFINE(dev_bench_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(dev_bench_fs_config, 0, USB_MAX_POWER_2MA, &dev_bench_fs_cfg_desc);

/* Hardware IDs are up to this long (hwinfo_get_device_id() truncates). The
 * serial the stack builds is the same ID, which is why they must agree. */
#define USB_HWID_MAX_BYTES 8U

static int add_descriptors(void)
{
	struct usbd_desc_node *const descriptors[] = {
		&dev_bench_lang, &dev_bench_mfr, &dev_bench_product, &dev_bench_sn,
	};

	for (size_t i = 0; i < ARRAY_SIZE(descriptors); i++) {
		int err = usbd_add_descriptor(&dev_bench_usbd, descriptors[i]);

		if (err != 0) {
			LOG_ERR("%s: descriptor %zu failed (%d)", __func__, i, err);
			return err;
		}
	}

	return 0;
}

static int register_ports(void)
{
	static const char *const ports[] = {"cdc_acm_0", "cdc_acm_1", "cdc_acm_2"};

	for (size_t i = 0; i < ARRAY_SIZE(ports); i++) {
		int err = usbd_register_class(&dev_bench_usbd, ports[i], USBD_SPEED_FS, 1);

		if (err != 0) {
			LOG_ERR("%s: registering %s failed (%d)", __func__, ports[i], err);
			return err;
		}
	}

	return 0;
}

/* On a controller that detects VBUS (the nRF52840's does), the stack reports
 * VBUS arriving and leaving, and the device is enabled and disabled on those
 * messages, as Zephyr's own CDC ACM sample does: usbd_enable() before VBUS is
 * ready leaves the controller off (USBD.ENABLE read 0 on the first nRF52840
 * dev bench, 2026-11-25). Other messages need nothing from the bench. */
static void usb_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	int err;

	if (!usbd_can_detect_vbus(ctx)) {
		return;
	}

	switch (msg->type) {
		case USBD_MSG_VBUS_READY:
			err = usbd_enable(ctx);
			if (err != 0) {
				LOG_ERR("%s: usbd_enable failed (%d)", __func__, err);
			}
			break;
		case USBD_MSG_VBUS_REMOVED:
			err = usbd_disable(ctx);
			if (err != 0) {
				LOG_ERR("%s: usbd_disable failed (%d)", __func__, err);
			}
			break;
		default:
			break;
	}
}

int usb_device_start(void)
{
	uint8_t hwid[USB_HWID_MAX_BYTES];
	ssize_t hwid_len = hwinfo_get_device_id(hwid, sizeof(hwid));
	int err;

	if (hwid_len <= 0) {
		LOG_ERR("%s: no hardware ID (%d)", __func__, (int)hwid_len);
		return -ENODEV;
	}

	err = usbd_device_set_pid(&dev_bench_usbd,
				  embarch_usb_pid(EMBARCH_USB_DEVICE_TYPE_DEV_BENCH, hwid,
						  (size_t)hwid_len));
	if (err != 0) {
		LOG_ERR("%s: setting the PID failed (%d)", __func__, err);
		return err;
	}

	err = add_descriptors();
	if (err != 0) {
		return err;
	}

	err = usbd_add_configuration(&dev_bench_usbd, USBD_SPEED_FS, &dev_bench_fs_config);
	if (err != 0) {
		LOG_ERR("%s: adding the configuration failed (%d)", __func__, err);
		return err;
	}

	err = register_ports();
	if (err != 0) {
		return err;
	}

	/* Several functions behind Interface Association Descriptors: the
	 * composite device's class triple. */
	err = usbd_device_set_code_triple(&dev_bench_usbd, USBD_SPEED_FS, USB_BCC_MISCELLANEOUS,
					  USB_MISC_SUBCLASS_COMMON, USB_MISC_PROTOCOL_IAD);
	if (err != 0) {
		LOG_ERR("%s: setting the class triple failed (%d)", __func__, err);
		return err;
	}

	err = usbd_msg_register_cb(&dev_bench_usbd, usb_msg_cb);
	if (err != 0) {
		LOG_ERR("%s: registering the message callback failed (%d)", __func__, err);
		return err;
	}

	err = usbd_init(&dev_bench_usbd);
	if (err != 0) {
		LOG_ERR("%s: usbd_init failed (%d)", __func__, err);
		return err;
	}

	/* A controller that detects VBUS is enabled by usb_msg_cb() instead. */
	if (!usbd_can_detect_vbus(&dev_bench_usbd)) {
		err = usbd_enable(&dev_bench_usbd);
		if (err != 0) {
			LOG_ERR("%s: usbd_enable failed (%d)", __func__, err);
			return err;
		}
	}

	return 0;
}
