/* usb_ids.h: the VID, the PID's two bytes and the serial string. */
#include <string.h>

#include <zephyr/ztest.h>

#include "usb_ids.h"

static const uint8_t id[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

ZTEST(usb_ids, test_vid_is_the_local_use_vendor_id)
{
	zassert_equal(EMBARCH_USB_VID, 0xEBAC);
}

ZTEST(usb_ids, test_dev_bench_is_type_one)
{
	zassert_equal(EMBARCH_USB_DEVICE_TYPE_DEV_BENCH, 0x01);
}

ZTEST(usb_ids, test_pid_high_byte_is_the_device_type_low_byte_is_the_last_id_byte)
{
	zassert_equal(embarch_usb_pid(EMBARCH_USB_DEVICE_TYPE_DEV_BENCH, id, sizeof(id)), 0x01EF);
}

ZTEST(usb_ids, test_pid_low_byte_follows_the_id_length_given)
{
	zassert_equal(embarch_usb_pid(EMBARCH_USB_DEVICE_TYPE_DEV_BENCH, id, 1U), 0x0101);
	zassert_equal(embarch_usb_pid(EMBARCH_USB_DEVICE_TYPE_DEV_BENCH, id, 4U), 0x0167);
}

ZTEST(usb_ids, test_serial_is_the_id_in_uppercase_hex)
{
	char sn[2 * sizeof(id) + 1];

	zassert_equal(embarch_usb_serial_hex(id, sizeof(id), sn, sizeof(sn)), 16U);
	zassert_str_equal(sn, "0123456789ABCDEF");
}

ZTEST(usb_ids, test_pid_low_byte_is_the_serials_last_two_digits)
{
	char sn[2 * sizeof(id) + 1];
	uint16_t pid = embarch_usb_pid(EMBARCH_USB_DEVICE_TYPE_DEV_BENCH, id, sizeof(id));
	char low[3];

	(void)embarch_usb_serial_hex(id, sizeof(id), sn, sizeof(sn));
	snprintk(low, sizeof(low), "%02X", pid & 0xFF);
	zassert_str_equal(low, &sn[strlen(sn) - 2U]);
}

ZTEST(usb_ids, test_serial_refuses_a_buffer_without_room_for_the_nul)
{
	char sn[2 * sizeof(id)];

	zassert_equal(embarch_usb_serial_hex(id, sizeof(id), sn, sizeof(sn)), 0U);
}

ZTEST_SUITE(usb_ids, NULL, NULL, NULL, NULL, NULL);
