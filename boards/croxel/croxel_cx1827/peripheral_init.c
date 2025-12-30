#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(croxel_cx1827, LOG_LEVEL_INF);

/* Device Tree node identifiers for peripherals */
#define LED_NODES(n) DT_ALIAS(led##n)
#define BTN_NODES(n) DT_ALIAS(sw##n)

/* GPIO specifications for peripherals */
static const struct gpio_dt_spec leds[] = {
#if DT_NODE_HAS_STATUS(LED_NODES(0), okay)
	GPIO_DT_SPEC_GET(LED_NODES(0), gpios),
#endif
#if DT_NODE_HAS_STATUS(LED_NODES(1), okay)
	GPIO_DT_SPEC_GET(LED_NODES(1), gpios),
#endif
#if DT_NODE_HAS_STATUS(LED_NODES(2), okay)
	GPIO_DT_SPEC_GET(LED_NODES(2), gpios),
#endif
};

static const struct gpio_dt_spec buttons[] = {
#if DT_NODE_HAS_STATUS(BTN_NODES(0), okay)
	GPIO_DT_SPEC_GET(BTN_NODES(0), gpios),
#endif
#if DT_NODE_HAS_STATUS(BTN_NODES(1), okay)
	GPIO_DT_SPEC_GET(BTN_NODES(1), gpios),
#endif
};

#define CAN_STB_NODE DT_ALIAS(can_stb)
BUILD_ASSERT(DT_NODE_HAS_PROP(CAN_STB_NODE, gpios), "CAN_STB node must have a gpios property");
static const struct gpio_dt_spec can_stb = GPIO_DT_SPEC_GET(CAN_STB_NODE, gpios);

#define BUZZER_NODE DT_ALIAS(buzzer)
BUILD_ASSERT(DT_NODE_HAS_PROP(BUZZER_NODE, pwms), "Buzzer node must have a pwms property");
static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(BUZZER_NODE);

/**
 * @brief Initialize all board peripherals
 *
 * This function initializes the LED, button, and buzzer to their default states:
 * - LED: Configured as output, initially off
 * - Button: Configured as input
 * - Buzzer: Configured as PWM output, initially off
 *
 * @return 0 if successful, negative errno code if failure
 */
static int peripherals_init(void)
{
	int ret;
	int i;

	/* Initialize LEDs */
	for (i = 0; i < ARRAY_SIZE(leds); i++) {
		if (leds[i].port != NULL) {
			if (!gpio_is_ready_dt(&leds[i])) {
				return -ENODEV;
			}

			ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
			if (ret < 0) {
				return ret;
			}
		}
	}

	/* Initialize CAN Standby Pin */
	if (can_stb.port != NULL) {
		if (!gpio_is_ready_dt(&can_stb)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&can_stb, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	/* Initialize Buttons */
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (buttons[i].port != NULL) {
			if (!gpio_is_ready_dt(&buttons[i])) {
				return -ENODEV;
			}

			ret = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
			if (ret < 0) {
				return ret;
			}
		}
	}

	/* Initialize Buzzer */
	if (!device_is_ready(buzzer.dev)) {
		return -ENODEV;
	}

	/* Set buzzer to 0% duty cycle (off) */
	ret = pwm_set_dt(&buzzer, 0, 0);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* Register initialization function to run during system initialization */
SYS_INIT(peripherals_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
