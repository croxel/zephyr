/**
 * @file
 * @brief LP55XX LED driver
 *
 * The LP55XX is a 4-channel LED driver that communicates over I2C. The four
 * channels are expected to be connected to a red, green, blue and white LED.
 * Each LED can be driven by two different sources.
 *
 * 1. The brightness of each LED can be configured directly by setting a
 * register that drives the PWM of the connected LED.
 *
 * 2. A program can be transferred to the driver and run by one of the three
 * available execution engines. Up to 16 commands can be defined in each
 * program. Possible commands are:
 *	- Set the brightness.
 *	- Fade the brightness over time.
 *	- Loop parts of the program or the whole program.
 *	- Add delays.
 *	- Synchronize between the engines.
 *
 * After the program has been transferred, it can run infinitely without
 * communication between the host MCU and the driver.
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/led/led.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lp55xx);

#include "led_context.h"

/* Common Registers */
#define LP55XX_ENABLE		0x00
#define LP55XX_OP_MODE		0x01
#define LP55XX_RGB_PWM_BASE	0x02
#define LP55XX_RGB_CURRENT_BASE 0x05
#define LP55XX_CONFIG		0x08
#define LP55XX_RESET		0x0D
/* Program memory definitions */
#define LP55XX_PROG_MEM_END	 0x50
#define LP55XX_PROG_MEM_SIZE	 0x20
#define LP55XX_PROG_MAX_COMMANDS 16

/* Program memory helper function */
#define LP55XX_ENG_PROG_ADDR(eng) (LP55XX_PROG_MEM_END - ((eng) * LP55XX_PROG_MEM_SIZE))
/* Engine control bit definitions and helper macro */
#define LP55XX_ENG_BIT_MASK		0x03
#define LP55XX_ENG_SHIFT(eng)		((eng) << 1)
#define LP55XX_ENG_CTRL_VALUE(eng, val) ((val) << LP55XX_ENG_SHIFT(eng))
#define LP55XX_ENG_CTRL_MASK(eng)	LP55XX_ENG_CTRL_VALUE((eng), LP55XX_ENG_BIT_MASK)

#define LP5562_LED_MAP	 0x70
#define LP5562_W_PWM	 0x0E
#define LP5562_W_CURRENT 0x0F

/* Values for ENABLE register. */
#define LP55XX_ENABLE_CHIP_EN_BIT  6
#define LP55XX_ENABLE_CHIP_EN_MASK (1 << LP55XX_ENABLE_CHIP_EN_BIT)
#define LP55XX_ENABLE_CHIP_EN_SET  (1 << LP55XX_ENABLE_CHIP_EN_BIT)
#define LP55XX_ENABLE_CHIP_EN_CLR  (0 << LP55XX_ENABLE_CHIP_EN_BIT)
#define LP55XX_ENABLE_LOG_EN_BIT   7
#define LP55XX_ENABLE_LOG_EN_MASK  (1 << LP55XX_ENABLE_LOG_EN_BIT)
#define LP55XX_ENABLE_LOG_EN_SET   (1 << LP55XX_ENABLE_LOG_EN_BIT)
#define LP55XX_ENABLE_LOG_EN_CLR   (0 << LP55XX_ENABLE_LOG_EN_BIT)

/* Values for CONFIG register. */
#define LP55XX_CONFIG_EXTERNAL_CLOCK	     0x00
#define LP55XX_CONFIG_INTERNAL_CLOCK	     0x01
#define LP55XX_CONFIG_CLOCK_AUTOMATIC_SELECT 0x02
#define LP55XX_CONFIG_CLK_SEL(m)	     (((m) & 0x03) << 0)
#define LP55XX_CONFIG_PWRSAVE_EN	     (1 << 5)
/* Enable 558 Hz frequency for PWM. Default is 256. */
#define LP55XX_CONFIG_PWM_HW_FREQ_558 (1 << 6)

/* Helper definitions. */
#define LP5521_CONFIG_R_TO_BATT_BIT 2
#define LP5521_CONFIG_R_TO_BATT	    (1 << LP5521_CONFIG_R_TO_BATT_BIT)
#define LP5521_CONFIG_CP_MODE(m)    (((m) & 0x03) << 3)

#define LP5562_LED_MAP_TO_ENG(map, id) (((map) >> LP55XX_ENG_SHIFT(id)) & LP55XX_ENG_BIT_MASK)

/*
 * The wait command has six bits for the number of steps (max 63) with up to
 * 15.6ms per step if the prescaler is set to 1. We round the step length
 * however to 16ms for easier handling, so the maximum blinking period is
 * therefore (16 * 63) = 1008ms. We round it down to 1000ms to be on the safe
 * side.
 */
#define LP55XX_MAX_BLINK_PERIOD 1000
/*
 * The minimum waiting period is 0.49ms with the prescaler set to 0 and one
 * step. We round up to a full millisecond.
 */
#define LP55XX_MIN_BLINK_PERIOD 1

/* Brightness limits in percent */
#define LP55XX_MIN_BRIGHTNESS 0
#define LP55XX_MAX_BRIGHTNESS 100

/* Output current limits in 0.1 mA */
#define LP55XX_MIN_CURRENT_SETTING 0
#define LP55XX_MAX_CURRENT_SETTING 255

/* Values for execution engine programs. */
#define LP55XX_PROG_COMMAND_SET_PWM(pwm) {(1 << 6), (pwm)}
/*
 * One step with the prescaler set to 0 takes 0.49ms. The max value for
 * step_time is 63, so we just double the millisecond value. That way
 * the step_time value never goes above the allowed 63.
 *
 * With a prescaler value set to 1 one step takes 15.6ms. So by dividing
 * through 16 we get a decent enough result with low effort.
 */
#define LP55XX_PROG_COMMAND_WAIT(delay) \
	{(((delay) < 31) ? ((delay) << 1) : (0x40 | ((delay) >> 4))), 0x00}

#define LP55XX_PROG_COMMAND_GO_TO_START() {0, 0}

/* Operational modes of the execution engines. */
enum lp55xx_engine_op_modes {
	LP55XX_OP_MODE_DISABLED = 0x00,
	LP55XX_OP_MODE_LOAD = 0x01,
	LP55XX_OP_MODE_RUN = 0x02,
	LP55XX_OP_MODE_DIRECT_CTRL = 0x03,
};

/* Execution state of the engines. */
enum lp55xx_engine_exec_states {
	LP55XX_ENGINE_MODE_HOLD = 0x00,
	LP55XX_ENGINE_MODE_STEP = 0x01,
	LP55XX_ENGINE_MODE_RUN = 0x02,
	LP55XX_ENGINE_MODE_EXEC = 0x03,
};

enum lp55xx_chip_id {
	LP55XX_CHIP_ID_5521,
	LP55XX_CHIP_ID_5562,
	LP55XX_CHIP_ID_MAX,
};

enum lp55xx_led_mode {
	LP55XX_MODE_PWM,
	LP55XX_MODE_ENG,
};

static inline int lp55xx_set_engine_op_mode(const struct i2c_dt_spec *bus, uint8_t eng_id,
					    enum lp55xx_engine_op_modes op_mode)
{
	int ret = i2c_reg_update_byte_dt(bus, LP55XX_OP_MODE, LP55XX_ENG_CTRL_MASK(eng_id),
					 LP55XX_ENG_CTRL_VALUE(eng_id, op_mode));
	if (ret == 0) {
		/* Must wait for next i2c write on OP_MODE reg */
		k_msleep(1);
	}
	return ret;
}

static inline int lp55xx_get_engine_op_mode(const struct i2c_dt_spec *bus, uint8_t eng_id,
					    enum lp55xx_engine_op_modes *op_mode)
{
	int err = i2c_reg_read_byte_dt(bus, LP55XX_OP_MODE, op_mode);
	if (err == 0) {
		(*op_mode) &= LP55XX_ENG_CTRL_MASK(eng_id);
		(*op_mode) >>= LP55XX_ENG_SHIFT(eng_id);
	}
	return err;
}

static inline int lp55xx_set_engine_exec_state(const struct i2c_dt_spec *bus, uint8_t eng_id,
					       enum lp55xx_engine_exec_states state)
{
	int ret = i2c_reg_update_byte_dt(bus, LP55XX_ENABLE, LP55XX_ENG_CTRL_MASK(eng_id),
					 LP55XX_ENG_CTRL_VALUE(eng_id, state));
	if (ret == 0) {
		/*
		 * Delay between consecutive I2C writes to
		 * ENABLE register (00h) need to be longer than 488μs (typ.).
		 */
		k_msleep(1);
	}
	return ret;
}

static inline int lp55xx_get_engine_exec_state(const struct i2c_dt_spec *bus, uint8_t eng_id,
					       enum lp55xx_engine_exec_states *state)
{
	int err = i2c_reg_read_byte_dt(bus, LP55XX_OP_MODE, state);
	if (err == 0) {
		(*state) &= LP55XX_ENG_CTRL_MASK(eng_id);
		(*state) >>= LP55XX_ENG_SHIFT(eng_id);
	}
	return err;
}

int lp5521_set_led_mode(const struct i2c_dt_spec *bus, uint8_t color, enum lp55xx_led_mode mode,
			uint8_t *eng_id)
{
	int err = -ENOTSUP;
	if ((color < LED_COLOR_ID_RED) || (color > LED_COLOR_ID_BLUE)) {
		return -ENOTSUP;
	}
	/* color to eng mapping is simple 1 to 1 */
	*eng_id = (LED_COLOR_ID_BLUE - color);

	switch (mode) {
	case LP55XX_MODE_PWM: {
		err = lp55xx_set_engine_op_mode(bus, *eng_id, LP55XX_OP_MODE_DIRECT_CTRL);
	} break;
	case LP55XX_MODE_ENG: {
		/* put eng state to run, because we can get here through blink api
		 * and after flashing program it will auto restart
		 */
		err = lp55xx_set_engine_exec_state(bus, *eng_id, LP55XX_ENGINE_MODE_RUN);
		if (err == 0) {
			err = lp55xx_set_engine_op_mode(bus, *eng_id, LP55XX_OP_MODE_RUN);
		}
	} break;
	}
	return err;
}

int lp5521_get_led_mode(const struct i2c_dt_spec *bus, uint8_t color, enum lp55xx_led_mode *mode,
			uint8_t *eng_id)
{
	int err;
	enum lp55xx_engine_op_modes op_mode;
	enum lp55xx_engine_exec_states state;

	if ((color < LED_COLOR_ID_RED) || (color > LED_COLOR_ID_BLUE)) {
		return -ENOTSUP;
	}
	/* color to eng mapping is simple 1 to 1 */
	*eng_id = (LED_COLOR_ID_BLUE - color);

	err = lp55xx_get_engine_op_mode(bus, *eng_id, &op_mode);
	if (err) {
		return err;
	}
	if (op_mode == LP55XX_OP_MODE_DIRECT_CTRL) {
		*mode = LP55XX_MODE_PWM;
	} else {
		err = lp55xx_get_engine_exec_state(bus, *eng_id, &state);
		if ((op_mode == LP55XX_OP_MODE_RUN) && (state == LP55XX_ENGINE_MODE_RUN)) {
			*mode = LP55XX_MODE_ENG;
		} else {
			*mode = LP55XX_MODE_PWM;
			err = lp5521_set_led_mode(bus, color, *mode, eng_id);
		}
	}
	return err;
}

static int lp5562_find_free_engine(uint8_t led_map, uint8_t *eng_id)
{
	uint8_t eng_link_mask = 0;
	uint8_t tmp_eng_id;
	/* prepare a link map */
	for (int led_id = 0; led_id < 4; led_id++) {
		tmp_eng_id = LP5562_LED_MAP_TO_ENG(led_map, led_id);
		if (tmp_eng_id) {
			eng_link_mask |= BIT(tmp_eng_id - 1);
		}
	}
	/* identify first engine that is not linked*/
	for (tmp_eng_id = 0; tmp_eng_id < 3; tmp_eng_id++) {
		if ((eng_link_mask & BIT(tmp_eng_id)) == 0) {
			*eng_id = tmp_eng_id;
			return 0;
		}
	}
	return -EOVERFLOW;
}

int lp5562_set_led_mode(const struct i2c_dt_spec *bus, uint8_t color, enum lp55xx_led_mode mode,
			uint8_t *eng_id)
{
	uint8_t led_id = LED_COLOR_ID_BLUE - color;
	uint8_t led_map;
	int err;

	if (color > LED_COLOR_ID_BLUE) {
		return -ENOTSUP;
	}

	err = i2c_reg_read_byte_dt(bus, LP5562_LED_MAP, &led_map);
	if (err) {
		return err;
	}

	/* parse eng id value */
	*eng_id = LP5562_LED_MAP_TO_ENG(led_map, led_id);
	/* clear map, basically configure it in pwm mode */
	led_map &= ~LP55XX_ENG_CTRL_MASK(led_id);
	/* if need to put it in engine mode, identify free engine and configure it */
	if (mode == LP55XX_MODE_ENG) {
		/* if no engine is linked, identify a free engine and link it */
		if (*eng_id == 0) {
			/* identify free engine id */
			err = lp5562_find_free_engine(led_map, eng_id);
			if (err) {
				return err;
			}
		}
		/* link eng_id in led map*/
		led_map |= LP55XX_ENG_CTRL_VALUE(led_id, *eng_id);
		/* put eng state to run, because we can get here through blink api
		 * and after flashing program it will auto restart
		 */
		err = lp55xx_set_engine_exec_state(bus, *eng_id, LP55XX_ENGINE_MODE_RUN);
		if (err) {
			return err;
		}
		err = lp55xx_set_engine_op_mode(bus, *eng_id, LP55XX_OP_MODE_RUN);
		if (err) {
			return err;
		}
	}
	/* update led map */
	return i2c_reg_write_byte_dt(bus, LP5562_LED_MAP, led_map);
}

int lp5562_get_led_mode(const struct i2c_dt_spec *bus, uint8_t color, enum lp55xx_led_mode *mode,
			uint8_t *eng_id)
{
	uint8_t led_id = LED_COLOR_ID_BLUE - color;
	uint8_t led_map;
	enum lp55xx_engine_op_modes op_mode;
	enum lp55xx_engine_exec_states state;
	int err;

	if (color > LED_COLOR_ID_BLUE) {
		return -ENOTSUP;
	}

	err = i2c_reg_read_byte_dt(bus, LP5562_LED_MAP, &led_map);
	if (err) {
		return err;
	}

	/* parse eng id value */
	*eng_id = LP5562_LED_MAP_TO_ENG(led_map, led_id);

	if (*eng_id == 0) {
		*mode = LP55XX_MODE_PWM;
	} else {
		(*eng_id)--;
		err = lp55xx_get_engine_op_mode(bus, *eng_id, &op_mode);
		if (err) {
			return err;
		}
		err = lp55xx_get_engine_exec_state(bus, *eng_id, &state);
		if (err) {
			return err;
		}
		if ((op_mode == LP55XX_OP_MODE_RUN) && (state == LP55XX_ENGINE_MODE_RUN)) {
			*mode = LP55XX_MODE_ENG;
		} else {
			*mode = LP55XX_MODE_PWM;
			err = lp5562_set_led_mode(bus, color, *mode, eng_id);
		}
	}
	return err;
}

typedef int (*set_led_mode_fn)(const struct i2c_dt_spec *bus, uint8_t color,
			       enum lp55xx_led_mode mode, uint8_t *eng_id);

typedef int (*get_led_mode_fn)(const struct i2c_dt_spec *bus, uint8_t color,
			       enum lp55xx_led_mode *mode, uint8_t *eng_id);

struct lp55xx_interface {
	enum lp55xx_chip_id id;
	uint8_t config_mask;
	set_led_mode_fn set_mode;
	get_led_mode_fn get_mode;
	uint8_t wrgb_current_addr[4];
	uint8_t wrgb_pwm_addr[4];
};

static const struct lp55xx_interface _lp55xx_ifaces[LP55XX_CHIP_ID_MAX] = {
    /* lp5521 */
    {
	.id = LP55XX_CHIP_ID_5521,
	.config_mask = 0x7F,
	.set_mode = lp5521_set_led_mode,
	.get_mode = lp5521_get_led_mode,
	.wrgb_current_addr =
	    {
		0, /* not available on 5521 */
		LP55XX_RGB_CURRENT_BASE,
		LP55XX_RGB_CURRENT_BASE + 1,
		LP55XX_RGB_CURRENT_BASE + 2,
	    },
	.wrgb_pwm_addr =
	    {
		0, /* not available on 5521 */
		LP55XX_RGB_PWM_BASE,
		LP55XX_RGB_PWM_BASE + 1,
		LP55XX_RGB_PWM_BASE + 2,
	    },
    },
    /* lp5562 */
    {
	.id = LP55XX_CHIP_ID_5562,
	.config_mask = 0x63,
	.set_mode = lp5562_set_led_mode,
	.get_mode = lp5562_get_led_mode,
	.wrgb_current_addr =
	    {
		LP5562_W_CURRENT,
		LP55XX_RGB_CURRENT_BASE + 2,
		LP55XX_RGB_CURRENT_BASE + 1,
		LP55XX_RGB_CURRENT_BASE,
	    },
	.wrgb_pwm_addr =
	    {
		LP5562_W_PWM,
		LP55XX_RGB_PWM_BASE + 2,
		LP55XX_RGB_PWM_BASE + 1,
		LP55XX_RGB_PWM_BASE,
	    },
    },
};

struct lp55xx_config {
	/* Bus is first parameter to facilitate custom user APIs */
	struct i2c_dt_spec bus;
	const struct lp55xx_interface *iface;
	uint8_t wrgb_current[4];
	uint8_t clk_sel;
	bool hf_pwm_dis;
	bool power_save_dis;
	bool active_low;
	bool log_scale_en;
	uint8_t charge_pump_mode;
	bool red_charge_pump_bypass;
	const struct gpio_dt_spec en_pin;
};

struct lp55xx_data {
	struct led_data dev_data;
};

static int lp55xx_led_blink(const struct device *dev, uint32_t color, uint32_t delay_on,
			    uint32_t delay_off)
{
	const struct lp55xx_config *config = dev->config;
	const struct lp55xx_interface *iface = config->iface;
	uint8_t eng_id;
	int ret;

	if ((delay_on > LP55XX_MAX_BLINK_PERIOD) || (delay_on < LP55XX_MIN_BLINK_PERIOD) ||
	    (delay_off > LP55XX_MAX_BLINK_PERIOD) || (delay_off < LP55XX_MIN_BLINK_PERIOD)) {
		return -EINVAL;
	}

	ret = iface->set_mode(&config->bus, color, LP55XX_MODE_ENG, &eng_id);
	if (ret) {
		return ret;
	}

	uint8_t program_code[5][2] = {
	    /* */
	    LP55XX_PROG_COMMAND_SET_PWM((config->active_low) ? 0 : 255),
	    /* */
	    LP55XX_PROG_COMMAND_WAIT(delay_on),
	    /* */
	    LP55XX_PROG_COMMAND_SET_PWM((config->active_low) ? 255 : 0),
	    /* */
	    LP55XX_PROG_COMMAND_WAIT(delay_off),
	    /* */
	    LP55XX_PROG_COMMAND_GO_TO_START(),
	};

	ret = lp55xx_set_engine_op_mode(&config->bus, eng_id, LP55XX_OP_MODE_LOAD);
	if (ret) {
		return ret;
	}

	ret = i2c_burst_write_dt(&config->bus, LP55XX_ENG_PROG_ADDR(eng_id), (uint8_t *)program_code,
				 sizeof(program_code));
	if (ret) {
		return ret;
	}

	ret = lp55xx_set_engine_op_mode(&config->bus, eng_id, LP55XX_OP_MODE_RUN);
	if (ret) {
		return -EIO;
	}

	return 0;
}

static inline int lp55xx_set_pwm_ctrl_value(const struct device *dev, uint8_t color, uint8_t duty)
{
	const struct lp55xx_config *config = dev->config;
	const struct lp55xx_interface *iface = config->iface;
	uint8_t reg_val = (duty * 0xFF) / LP55XX_MAX_BRIGHTNESS;
	int err;

	if (iface->wrgb_pwm_addr[color] == 0) {
		return -ENOTSUP;
	}
	if (config->active_low) {
		reg_val = ~reg_val;
	}
	err = i2c_reg_write_byte_dt(&config->bus, iface->wrgb_pwm_addr[color], reg_val);
	if (err) {
		LOG_ERR("Reg[%x]=%x write err: %d", iface->wrgb_pwm_addr[color], reg_val, err);
	}

	return err;
}

static int lp55xx_set_pwm_eng_value(const struct device *dev, uint8_t color, uint8_t duty,
				    uint8_t eng_id)
{
	const struct lp55xx_config *config = dev->config;
	uint8_t reg_val = (duty * 0xFF) / LP55XX_MAX_BRIGHTNESS;
	int ret;

	uint8_t program_code[1][2] = {
	    LP55XX_PROG_COMMAND_SET_PWM((config->active_low) ? ~reg_val : reg_val),
	};

	ret = lp55xx_set_engine_op_mode(&config->bus, eng_id, LP55XX_OP_MODE_LOAD);
	if (ret) {
		return ret;
	}

	ret = i2c_burst_write_dt(&config->bus, LP55XX_ENG_PROG_ADDR(eng_id), (uint8_t *)program_code,
				 sizeof(program_code));
	if (ret) {
		return ret;
	}

	ret = lp55xx_set_engine_op_mode(&config->bus, eng_id, LP55XX_OP_MODE_RUN);
	if (ret) {
		return -EIO;
	}
	return ret;
}

static int lp55xx_led_set_brightness(const struct device *dev, uint32_t color, uint8_t value)
{
	const struct lp55xx_config *config = dev->config;
	uint8_t eng_id;
	enum lp55xx_led_mode mode;
	int err;

	if ((value < LP55XX_MIN_BRIGHTNESS) || (value > LP55XX_MAX_BRIGHTNESS)) {
		return -EINVAL;
	}

	err = config->iface->get_mode(&config->bus, color, &mode, &eng_id);
	if (err) {
		return err;
	}

	if (mode == LP55XX_MODE_PWM) {
		err = lp55xx_set_pwm_ctrl_value(dev, color, value);
	} else {
		err = lp55xx_set_pwm_eng_value(dev, color, value, eng_id);
	}
	return err;
}

static inline int lp55xx_led_on(const struct device *dev, uint32_t color)
{
	const struct lp55xx_config *config = dev->config;
	uint8_t eng_id;

	int ret = config->iface->set_mode(&config->bus, color, LP55XX_MODE_PWM, &eng_id);
	if (ret) {
		return ret;
	}
	return lp55xx_set_pwm_ctrl_value(dev, color, LP55XX_MAX_BRIGHTNESS);
}

static inline int lp55xx_led_off(const struct device *dev, uint32_t color)
{
	const struct lp55xx_config *config = dev->config;
	uint8_t eng_id;

	int ret = config->iface->set_mode(&config->bus, color, LP55XX_MODE_PWM, &eng_id);
	if (ret) {
		return ret;
	}
	return lp55xx_set_pwm_ctrl_value(dev, color, LP55XX_MIN_BRIGHTNESS);
}

static int lp55xx_led_update_current(const struct device *dev)
{
	const struct lp55xx_config *config = dev->config;
	const struct lp55xx_interface *iface = config->iface;
	const uint8_t *reg_addr = iface->wrgb_current_addr;
	const uint8_t *reg_val = config->wrgb_current;
	int err = 0;

	for (int color = LED_COLOR_ID_WHITE; color <= LED_COLOR_ID_BLUE; color++) {
		if (reg_addr[color] == 0) {
			// skip if not supported
			continue;
		}
		err = i2c_reg_write_byte_dt(&config->bus, reg_addr[color], reg_val[color]);
		if (err) {
			LOG_ERR("Reg[%x]=%x write err: %d", reg_addr[color], reg_val[color], err);
			break;
		}
	}
	return err;
}

static int lp55xx_enable(const struct device *dev)
{
	const struct lp55xx_config *config = dev->config;
	const struct gpio_dt_spec *en_pin = &config->en_pin;
	int err = 0;

	/* if gpio control is enabled, we need to enable both
	 * en_io and en bit in register configurations
	 */
	if ((en_pin->port != NULL) && (gpio_pin_get_dt(en_pin) != 1)) {
		err = gpio_pin_set_dt(en_pin, 1);
		if (err) {
			LOG_ERR("en_gpio set low err: %d", err);
			return err;
		}
		k_msleep(5);
	}
	err = i2c_reg_write_byte_dt(&config->bus, LP55XX_RESET, 0xFF);
	if (err) {
		return err;
	}
	k_msleep(1);

	if (config->log_scale_en) {
		err = i2c_reg_write_byte_dt(&config->bus, LP55XX_ENABLE,
					    LP55XX_ENABLE_CHIP_EN_SET | LP55XX_ENABLE_LOG_EN_SET);
	} else {
		err = i2c_reg_write_byte_dt(&config->bus, LP55XX_ENABLE, LP55XX_ENABLE_CHIP_EN_SET);
	}
	if (err) {
		return err;
	}
	k_msleep(1);
	return err;
}

#ifdef CONFIG_PM_DEVICE
static int lp55xx_disable(const struct device *dev)
{
	const struct lp55xx_config *config = dev->config;
	const struct gpio_dt_spec *en_pin = &config->en_pin;
	int err = 0;

	/* if gpio control is enabled, we can disable through io only
	 * else need to disable using register configuration
	 */
	if (en_pin->port != NULL) {
		err = gpio_pin_set_dt(en_pin, 0);
		if (err) {
			LOG_ERR("en_gpio set low err: %d", err);
		}
	} else {
		err = i2c_reg_update_byte_dt(&config->bus, LP55XX_ENABLE,
					     LP55XX_ENABLE_CHIP_EN_MASK, LP55XX_ENABLE_CHIP_EN_CLR);
		if (err == 0) {
			k_msleep(1);
		}
	}
	return err;
}
#endif

static int lp55xx_led_init(const struct device *dev)
{
	const struct lp55xx_config *config = dev->config;
	const struct gpio_dt_spec *en_pin = &config->en_pin;
	int ret;
	int config_byte = 0;

	if (en_pin->port != NULL) {
		if (!device_is_ready(en_pin->port)) {
			return -ENODEV;
		}
		if (gpio_pin_configure_dt(en_pin, GPIO_OUTPUT_ACTIVE)) {
			return -ENODEV;
		}
	}

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	if (lp55xx_enable(dev) != 0) {
		LOG_ERR("LP55XX Enable failed");
		return -EIO;
	}

	ret = lp55xx_led_update_current(dev);
	if (ret) {
		LOG_ERR("Setting current setting LP55XX LED chip failed.");
		return ret;
	}

	/* Fill all configurations */
	config_byte |= (config->red_charge_pump_bypass ? LP5521_CONFIG_R_TO_BATT : 0);
	config_byte |= LP5521_CONFIG_CP_MODE(config->charge_pump_mode);
	config_byte |= LP55XX_CONFIG_CLK_SEL(config->clk_sel);
	config_byte |= (config->hf_pwm_dis ? 0 : LP55XX_CONFIG_PWM_HW_FREQ_558);
	config_byte |= (config->power_save_dis ? 0 : LP55XX_CONFIG_PWRSAVE_EN);
	/* keep only supported config bits */
	config_byte &= config->iface->config_mask;
	if (i2c_reg_write_byte_dt(&config->bus, LP55XX_CONFIG, config_byte)) {
		LOG_ERR("Configuring LP55XX LED chip failed.");
		return -EIO;
	}
	return 0;
}

static const struct led_driver_api lp55xx_led_api = {
    .blink = lp55xx_led_blink,
    .set_brightness = lp55xx_led_set_brightness,
    .on = lp55xx_led_on,
    .off = lp55xx_led_off,
};

#ifdef CONFIG_PM_DEVICE
static int lp55xx_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		return lp55xx_disable(dev);
	case PM_DEVICE_ACTION_RESUME:
		return lp55xx_enable(dev);
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

#define LP55XX_DEFINE(id, chip_id_val)                                                            \
	BUILD_ASSERT(DT_INST_PROP(id, red_output_current) <= LP55XX_MAX_CURRENT_SETTING,          \
		     "Red channel current must be between 0 and 25.5 mA.");                       \
	BUILD_ASSERT(DT_INST_PROP(id, green_output_current) <= LP55XX_MAX_CURRENT_SETTING,        \
		     "Green channel current must be between 0 and 25.5 mA.");                     \
	BUILD_ASSERT(DT_INST_PROP(id, blue_output_current) <= LP55XX_MAX_CURRENT_SETTING,         \
		     "Blue channel current must be between 0 and 25.5 mA.");                      \
	BUILD_ASSERT(DT_INST_PROP_OR(id, white_output_current, 0) <= LP55XX_MAX_CURRENT_SETTING,  \
		     "White channel current must be between 0 and 25.5 mA.");                     \
	static const struct lp55xx_config lp55xx_config_##id = {                                  \
	    .iface = &_lp55xx_ifaces[chip_id_val],                                                \
	    .bus = I2C_DT_SPEC_INST_GET(id),                                                      \
	    .wrgb_current =                                                                       \
		{                                                                                 \
		    DT_INST_PROP_OR(id, white_output_current, 0),                                 \
		    DT_INST_PROP(id, red_output_current),                                         \
		    DT_INST_PROP(id, green_output_current),                                       \
		    DT_INST_PROP(id, blue_output_current),                                        \
		},                                                                                \
	    .clk_sel = DT_INST_PROP(id, clk_sel),                                                 \
	    .hf_pwm_dis = DT_INST_PROP(id, hf_pwm_dis),                                           \
	    .active_low = DT_INST_PROP(id, active_low),                                           \
	    .log_scale_en = DT_INST_PROP(id, log_scale_en),                                       \
	    .power_save_dis = DT_INST_PROP(id, power_save_dis),                                   \
	    .charge_pump_mode = DT_INST_PROP_OR(id, charge_pump_mode, 0),                         \
	    .red_charge_pump_bypass = DT_INST_PROP_OR(id, red_charge_pump_bypass, 0),             \
	    .en_pin = GPIO_DT_SPEC_INST_GET_OR(id, en_gpios, {0}),                                \
	};                                                                                        \
                                                                                                  \
	PM_DEVICE_DT_INST_DEFINE(id, lp55xx_pm_action);                                           \
                                                                                                  \
	struct lp55xx_data lp55xx_data_##id;                                                      \
	DEVICE_DT_INST_DEFINE(id, &lp55xx_led_init, NULL, &lp55xx_data_##id, &lp55xx_config_##id, \
			      POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &lp55xx_led_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_lp5562
DT_INST_FOREACH_STATUS_OKAY_VARGS(LP55XX_DEFINE, LP55XX_CHIP_ID_5562)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_lp5521
DT_INST_FOREACH_STATUS_OKAY_VARGS(LP55XX_DEFINE, LP55XX_CHIP_ID_5521)
