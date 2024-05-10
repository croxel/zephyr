/*
 * File: ws2812_uart.c
 * Author: Ayush kothari
 * Company: Croxel.com
 * Date: 09/05/2024
 * Description: Uart Driver file for the ws2812 rgb led
 */


#define DT_DRV_COMPAT worldsemi_ws2812_uart

#include <zephyr/drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_uart);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>


struct ws2812_uart_cfg {
	const struct device *uart_dev;       // UART device.
	uint8_t *px_buf;
	size_t px_buf_size;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	uint16_t reset_delay;
};

/*This lookup table maps 3 bits of information into (7+startBit+stopBit = 9)-bits which will
	be used to send ws2812 rgb led*/
static const uint8_t ws2812_uart_lut[8] ={
	0b1011011,
	0b0011011,
	0b1010011,
	0b0010011,
	0b1011010,
	0b0011010,
	0b1010010,
	0b0010010,
};
/*
 * Serialize an 24-bit color channel value into an equivalent sequence
 * of 8 bytes of uart frames,.
 */
static inline void ws2812_uart_ser(uint8_t *buf, uint8_t color[3])
{
    int i, j;
    int buff_indx = 0;
    uint8_t serialized_val = 0;

    for (i = 0; i < 3; i++) {
        for(j = 0; j < 8; j++){
            serialized_val = (serialized_val << 1) | ((color[i]>>(7-j))&0x01);
            if(((i*8)+j) % 3 == 2){
                buf[buff_indx++] = ws2812_uart_lut[serialized_val];
                serialized_val = 0;
            }
        }
    }
}
/*
 * Returns true if and only if cfg->px_buf is big enough to convert
 * num_pixels RGB color values into UART frames.
 */
static inline bool num_pixels_ok(const struct ws2812_uart_cfg *cfg,
				 size_t num_pixels)
{
	size_t nbytes;
	bool overflow;

	overflow = size_mul_overflow(num_pixels, cfg->num_colors * 8, &nbytes);
	return !overflow && (nbytes <= cfg->px_buf_size);
}

/*
 * Latch current color values on strip and reset its state machines.
 */
static inline void ws2812_reset_delay(uint16_t delay)
{
	k_usleep(delay);
}

static void ws2812_uart_tx(const struct ws2812_uart_cfg *cfg, uint8_t *tx, size_t len)
{
	for(size_t i =0; i < len; i++){
		uart_poll_out(cfg->uart_dev, tx[i]);
	}
}
static int ws2812_strip_update_rgb(const struct device *dev,
				   struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_uart_cfg *cfg = dev->config;
	
	uint8_t *px_buf = cfg->px_buf;
	size_t px_buf_len = cfg->px_buf_size;
	size_t uart_data_len = 0;
	size_t i;
	int rc=0;
	uint8_t color[3];

	if (!num_pixels_ok(cfg, num_pixels)) {
		return -ENOMEM;
	}

	/*
	 * Convert pixel data into Uart frames. Each frame has pixel data
	 * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
	 */

	for (i = 0; i < num_pixels; i++) {
		uint8_t j;

		for (j = 0; j < cfg->num_colors; j++) {
			switch (cfg->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				color[j] = 0;
				break;
			case LED_COLOR_ID_RED:
				color[j] = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				color[j] = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				color[j] = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
		}
		//convert 24 bits of rgb data into 8 bytes of uart frame
		ws2812_uart_ser(px_buf, &color[0]); 
		uart_data_len += 8;
		px_buf += uart_data_len;
	}
	ws2812_uart_tx(cfg, cfg->px_buf, uart_data_len);
	ws2812_reset_delay(51);

	return rc;
}

static int ws2812_strip_update_channels(const struct device *dev,
					uint8_t *channels,
					size_t num_channels)
{
	LOG_ERR("update_channels not implemented");
	return -ENOTSUP;
}

static int ws2812_uart_init(const struct device *dev)
{
	const struct ws2812_uart_cfg *cfg = dev->config;
	uint8_t i;

    if(!device_is_ready(cfg->uart_dev)){
        LOG_ERR("Uart device not ready");
        return -ENODEV;
    }

	for (i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct led_strip_driver_api ws2812_uart_api = {
	.update_rgb = ws2812_strip_update_rgb,
	.update_channels = ws2812_strip_update_channels,
};

#define WS2812_UART_NUM_PIXELS(idx) \
	(DT_INST_PROP(idx, chain_length))
#define WS2812_UART_BUFSZ(idx) \
	(WS2812_NUM_COLORS(idx) * 8 * WS2812_UART_NUM_PIXELS(idx))

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)				  \
	static const uint8_t ws2812_uart_##idx##_color_mapping[] = \
		DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define WS2812_RESET_DELAY(idx) DT_INST_PROP(idx, reset_delay)

#define WS2812_UART_DEVICE(idx)						 \
									 \
	static uint8_t ws2812_uart_##idx##_px_buf[WS2812_UART_BUFSZ(idx)]; \
									 \
	WS2812_COLOR_MAPPING(idx);					 \
									 \
	static const struct ws2812_uart_cfg ws2812_uart_##idx##_cfg = {	 \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(idx)),	 \
		.px_buf = ws2812_uart_##idx##_px_buf,			 \
		.px_buf_size = WS2812_UART_BUFSZ(idx),			 \
		.num_colors = WS2812_NUM_COLORS(idx),			 \
		.color_mapping = ws2812_uart_##idx##_color_mapping,	 \
		.reset_delay = WS2812_RESET_DELAY(idx),			 \
	};								 \
									 \
	DEVICE_DT_INST_DEFINE(idx,					 \
			      ws2812_uart_init,				 \
			      NULL,					 \
			      NULL,					 \
			      &ws2812_uart_##idx##_cfg,			 \
			      POST_KERNEL,				 \
			      CONFIG_LED_STRIP_INIT_PRIORITY,		 \
			      &ws2812_uart_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_UART_DEVICE)
