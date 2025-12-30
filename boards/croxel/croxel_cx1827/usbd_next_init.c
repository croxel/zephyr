#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <sample_usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_next_init, LOG_LEVEL_INF);

struct usbd_context *usb_next_init_ctx;

static void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}
}

static int enable_usb_device_next(void)
{
	int err;

	usb_next_init_ctx = sample_usbd_init_device(sample_msg_cb);
	if (usb_next_init_ctx == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(usb_next_init_ctx)) {
		err = usbd_enable(usb_next_init_ctx);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");

	return 0;
}

static int usbd_next_init(void)
{
	int ret;

	ret = enable_usb_device_next();
	if (ret < 0) {
		LOG_ERR("Failed to enable USB device: %d", ret);
	}

	return ret;
}

SYS_INIT(usbd_next_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
