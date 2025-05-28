/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control/max32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT adi_max32_i2s

LOG_MODULE_REGISTER(i2s_max32, CONFIG_I2S_LOG_LEVEL);

struct queue_item {
	void *buffer;
	size_t size;
};

/* Minimal number of RX DMA buffers to achieve continuous data flow */
#define NUM_DMA_BLOCKS_RX_PREP 3
#define MAX_TX_DMA_BLOCKS      CONFIG_DMA_TCD_QUEUE_SIZE

/* I2S supported sample formats */
#define I2S_WORD_SIZE_BITS_MIN 8U
#define I2S_WORD_SIZE_BITS_MAX 32U

/* Per-stream configuration data */
struct stream {
	int32_t state;
	uint32_t dma_channel;
	uint8_t num_channels;
	uint8_t word_size_bytes;
	bool active;
	uint8_t start_channel;
	struct i2s_config cfg;
	struct k_msgq in_queue;  /* Input buffer queue */
	struct k_msgq out_queue; /* Output buffer queue */
	struct dma_config dma_cfg;
	struct dma_block_config dma_block;
	uint8_t free_tx_dma_blocks; /* TX blocks empty and ready for loading */
	enum i2s_trigger_cmd last_cmd;
	void *mem_block;
};

/* Driver instance data */
struct i2s_max32_data {
	struct stream tx;
	struct stream rx;
	const struct device *dev_dma;
};

struct max32_i2s_dma_config {
	const struct device *dev;
	const uint32_t channel;
	const uint32_t slot;
};

/* Driver instance configuration */
struct i2s_max32_config {
	mxc_i2s_regs_t *regs;
	const struct device *clock_dev;
	const struct pinctrl_dev_config *pincfg;
	struct max32_perclk perclk;
	void (*irq_config)();
	const struct max32_i2s_dma_config rx_dma;
	const struct max32_i2s_dma_config tx_dma;
};

static void i2s_max32_update_stream_state(const struct device *dev, struct stream *strm,
					  enum i2s_state new_state)
{
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *other_strm;
	uint32_t key;

	key = irq_lock();

	/* If we're setting error state, also set the other stream to error state */
	if (new_state == I2S_STATE_ERROR) {
		other_strm = (strm == &dev_data->tx) ? &dev_data->rx : &dev_data->tx;
		if (other_strm->state == I2S_STATE_RUNNING) {
			other_strm->state = I2S_STATE_ERROR;
		}
	}

	strm->state = new_state;
	irq_unlock(key);
}

static void i2s_dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				int status)
{
	const struct device *dev = (const struct device *)arg;
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	void *buffer;
	int ret;

	if (strm->state == I2S_STATE_ERROR) {
		LOG_DBG("TX stream in error state");
		return;
	}

	if (status < 0) {
		LOG_ERR("TX DMA error: %d", status);
		i2s_max32_update_stream_state(dev, strm, I2S_STATE_ERROR);
		return;
	}

	buffer = strm->mem_block;
	ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("TX buffer queue put error");
		i2s_max32_update_stream_state(dev, strm, I2S_STATE_ERROR);
		return;
	}

	strm->mem_block = NULL;
}

static void i2s_dma_rx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				int status)
{
	const struct device *dev = (const struct device *)arg;
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *strm = &dev_data->rx;
	void *buffer;
	int ret;

	if (strm->state == I2S_STATE_ERROR) {
		LOG_DBG("RX stream in error state");
		return;
	}

	if (status < 0) {
		LOG_ERR("RX DMA error: %d", status);
		i2s_max32_update_stream_state(dev, strm, I2S_STATE_ERROR);
		return;
	}

	buffer = strm->mem_block;
	ret = k_msgq_put(&strm->in_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("RX buffer queue put error");
		i2s_max32_update_stream_state(dev, strm, I2S_STATE_ERROR);
		return;
	}

	strm->mem_block = NULL;
}

static int i2s_max32_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	const struct i2s_max32_config *dev_cfg = dev->config;
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *tx_strm = &dev_data->tx;
	struct stream *rx_strm = &dev_data->rx;
	uint32_t key;
	int ret = 0;

	key = irq_lock();

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (dir & I2S_DIR_TX) {
			if (tx_strm->state != I2S_STATE_READY) {
				ret = -EIO;
				break;
			}
			ret = dma_start(dev_data->dev_dma, tx_strm->dma_channel);
			if (ret < 0) {
				LOG_ERR("Failed to start TX DMA channel");
				ret = -EIO;
				break;
			}
			tx_strm->state = I2S_STATE_RUNNING;
		}
		if (dir & I2S_DIR_RX) {
			if (rx_strm->state != I2S_STATE_READY) {
				/* If TX already started, stop it */
				if ((dir & I2S_DIR_TX) && (tx_strm->state == I2S_STATE_RUNNING)) {
					dma_stop(dev_data->dev_dma, tx_strm->dma_channel);
					tx_strm->state = I2S_STATE_READY;
				}
				ret = -EIO;
				break;
			}
			ret = dma_start(dev_data->dev_dma, rx_strm->dma_channel);
			if (ret < 0) {
				LOG_ERR("Failed to start RX DMA channel");
				/* If TX already started, stop it */
				if ((dir & I2S_DIR_TX) && (tx_strm->state == I2S_STATE_RUNNING)) {
					dma_stop(dev_data->dev_dma, tx_strm->dma_channel);
					tx_strm->state = I2S_STATE_READY;
				}
				ret = -EIO;
				break;
			}
			rx_strm->state = I2S_STATE_RUNNING;
		}
		break;

	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
	case I2S_TRIGGER_DROP:
		if (dir & I2S_DIR_TX) {
			if (tx_strm->state != I2S_STATE_RUNNING &&
			    tx_strm->state != I2S_STATE_READY) {
				ret = -EIO;
				break;
			}
			ret = dma_stop(dev_data->dev_dma, tx_strm->dma_channel);
			if (ret < 0) {
				LOG_ERR("Failed to stop TX DMA channel");
				ret = -EIO;
				break;
			}
			tx_strm->state = I2S_STATE_READY;
		}
		if (dir & I2S_DIR_RX) {
			if (rx_strm->state != I2S_STATE_RUNNING &&
			    rx_strm->state != I2S_STATE_READY) {
				ret = -EIO;
				break;
			}
			ret = dma_stop(dev_data->dev_dma, rx_strm->dma_channel);
			if (ret < 0) {
				LOG_ERR("Failed to stop RX DMA channel");
				ret = -EIO;
				break;
			}
			rx_strm->state = I2S_STATE_READY;
		}
		break;

	case I2S_TRIGGER_PREPARE:
		if (dir & I2S_DIR_TX) {
			if (tx_strm->state != I2S_STATE_ERROR) {
				ret = -EIO;
				break;
			}
			tx_strm->state = I2S_STATE_READY;
		}
		if (dir & I2S_DIR_RX) {
			if (rx_strm->state != I2S_STATE_ERROR) {
				ret = -EIO;
				break;
			}
			rx_strm->state = I2S_STATE_READY;
		}
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		ret = -EINVAL;
	}

	irq_unlock(key);
	return ret;
}

static int i2s_max32_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *strm = &dev_data->rx;
	void *buffer;
	int ret;

	if (strm->state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state %d", strm->state);
		return -EIO;
	}

	ret = k_msgq_get(&strm->in_queue, &buffer, SYS_TIMEOUT_MS(strm->cfg.timeout));
	if (ret != 0) {
		return -EAGAIN;
	}

	*mem_block = buffer;
	*size = strm->cfg.block_size;
	return 0;
}

static int i2s_max32_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *strm = &dev_data->tx;
	int ret;

	if (strm->state != I2S_STATE_RUNNING && strm->state != I2S_STATE_READY) {
		LOG_DBG("invalid state %d", strm->state);
		return -EIO;
	}

	ret = k_msgq_put(&strm->in_queue, &mem_block, SYS_TIMEOUT_MS(strm->cfg.timeout));
	if (ret != 0) {
		LOG_ERR("k_msgq_put failed %d", ret);
		return -EAGAIN;
	}

	return ret;
}

static int i2s_max32_config_verify(const struct i2s_config *i2s_cfg)
{
	if (i2s_cfg->word_size < I2S_WORD_SIZE_BITS_MIN ||
	    i2s_cfg->word_size > I2S_WORD_SIZE_BITS_MAX) {
		LOG_ERR("Unsupported I2S word size");
		return -EINVAL;
	}

	if (i2s_cfg->channels != 2U) {
		LOG_ERR("Only 2 channels are supported");
		return -EINVAL;
	}

	if (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	return 0;
}

static int i2s_max32_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct i2s_max32_config *dev_cfg = dev->config;
	struct i2s_max32_data *dev_data = dev->data;
	struct stream *tx_strm = &dev_data->tx;
	struct stream *rx_strm = &dev_data->rx;
	mxc_i2s_req_t i2s_req;
	int ret;

	if (!i2s_cfg) {
		LOG_ERR("Configuration is NULL");
		return -EINVAL;
	}

	ret = i2s_max32_config_verify(i2s_cfg);
	if (ret != 0) {
		return ret;
	}

	/* For I2S_DIR_BOTH, both streams must be in valid state */
	if (dir == I2S_DIR_BOTH) {
		if ((tx_strm->state != I2S_STATE_NOT_READY && tx_strm->state != I2S_STATE_READY) ||
		    (rx_strm->state != I2S_STATE_NOT_READY && rx_strm->state != I2S_STATE_READY)) {
			LOG_ERR("invalid state for one or both streams");
			return -EINVAL;
		}
	}

	/* Configure TX stream if needed */
	if ((dir = I2S_DIR_TX) || (dir == I2S_DIR_BOTH)) {

		i2s_req.wordSize = MXC_I2S_WSIZE_HALFWORD;
		i2s_req.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
		i2s_req.bitsWord = 16;
		i2s_req.adjust = MXC_I2S_ADJUST_LEFT;
		i2s_req.justify = MXC_I2S_LSB_JUSTIFY;
		i2s_req.wsPolarity = MXC_I2S_POL_NORMAL;
		i2s_req.channelMode = MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS;
		i2s_req.stereoMode = MXC_I2S_MONO_RIGHT_CH;
		i2s_req.bitOrder = MXC_I2S_LSB_FIRST;
		i2s_req.clkdiv = MXC_I2S_CalculateClockDiv(SMPL_RATE, i2s_req.wordSize) + 1;
		i2s_req.rawData = NULL;
		i2s_req.txData = NULL;
		i2s_req.rxData = buf0;
		i2s_req.length = sizeof(buf0);

		ret = MXC_I2S_Init(&i2s_req);

		ret = MXC_I2S_ConfigData(dev_cfg->regs, &i2s_config);
		if (ret != 0) {
			LOG_ERR("Failed to configure I2S TX channel");
			return ret;
		}

		if (tx_strm->state != I2S_STATE_NOT_READY && tx_strm->state != I2S_STATE_READY) {
			LOG_ERR("TX stream in invalid state");
			return -EINVAL;
		}

		tx_strm->num_channels = i2s_cfg->channels;
		tx_strm->word_size_bytes = i2s_cfg->word_size / 8U;
		memcpy(&tx_strm->cfg, i2s_cfg, sizeof(struct i2s_config));

		/* Configure TX DMA */
		memset(&tx_strm->dma_cfg, 0, sizeof(tx_strm->dma_cfg));
		tx_strm->dma_cfg.dma_slot = dev_cfg->tx_dma.slot;
		tx_strm->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		tx_strm->dma_cfg.source_data_size = tx_strm->word_size_bytes;
		tx_strm->dma_cfg.dest_data_size = tx_strm->word_size_bytes;
		tx_strm->dma_cfg.source_burst_length = 1;
		tx_strm->dma_cfg.dest_burst_length = 1;
		tx_strm->dma_cfg.dma_callback = i2s_dma_tx_callback;
		tx_strm->dma_cfg.complete_callback_en = 1;
		tx_strm->dma_cfg.error_callback_en = 1;
		tx_strm->dma_cfg.block_count = 1;
		tx_strm->dma_cfg.head_block = &tx_strm->dma_block;
		tx_strm->dma_channel = dev_cfg->tx_dma.channel;

		tx_strm->state = I2S_STATE_READY;
	}

	/* Configure RX stream if needed */
	if (dir & I2S_DIR_RX) {
		if (rx_strm->state != I2S_STATE_NOT_READY && rx_strm->state != I2S_STATE_READY) {
			LOG_ERR("RX stream in invalid state");
			return -EINVAL;
		}

		rx_strm->num_channels = i2s_cfg->channels;
		rx_strm->word_size_bytes = i2s_cfg->word_size / 8U;
		memcpy(&rx_strm->cfg, i2s_cfg, sizeof(struct i2s_config));

		i2s_req.ch = 1;     /* Channel 1 for RX */
		i2s_req.clkdiv = 4; /* Default divider */
		i2s_req.rawdata = 0;
		i2s_req.len = rx_strm->cfg.block_size;
		i2s_req.callback = NULL;

		ret = MXC_I2S_ConfigData(dev_cfg->regs, &i2s_config);
		if (ret != 0) {
			LOG_ERR("Failed to configure I2S RX channel");
			return ret;
		}

		/* Configure RX DMA */
		memset(&rx_strm->dma_cfg, 0, sizeof(rx_strm->dma_cfg));
		rx_strm->dma_cfg.dma_slot = dev_cfg->rx_dma.slot;
		rx_strm->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		rx_strm->dma_cfg.source_data_size = rx_strm->word_size_bytes;
		rx_strm->dma_cfg.dest_data_size = rx_strm->word_size_bytes;
		rx_strm->dma_cfg.source_burst_length = 1;
		rx_strm->dma_cfg.dest_burst_length = 1;
		rx_strm->dma_cfg.dma_callback = i2s_dma_rx_callback;
		rx_strm->dma_cfg.complete_callback_en = 1;
		rx_strm->dma_cfg.error_callback_en = 1;
		rx_strm->dma_cfg.block_count = 1;
		rx_strm->dma_cfg.head_block = &rx_strm->dma_block;
		rx_strm->dma_channel = dev_cfg->rx_dma.channel;

		rx_strm->state = I2S_STATE_READY;
	}

	return 0;
}

static int i2s_max32_init(const struct device *dev)
{
	const struct i2s_max32_config *dev_cfg = dev->config;
	struct i2s_max32_data *dev_data = dev->data;
	mxc_i2s_regs_t *base = dev_cfg->regs;
	int ret;

	/* Initialize DMA */
	dev_data->dev_dma = dev_cfg->tx_dma.dev;
	if (!device_is_ready(dev_data->dev_dma)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	/* Initialize pins */
	ret = pinctrl_apply_state(dev_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static const struct i2s_driver_api i2s_max32_driver_api = {
	.configure = i2s_max32_configure,
	.write = i2s_max32_write,
	.read = i2s_max32_read,
	.trigger = i2s_max32_trigger,
};

/* Device Instantiation */
#define MAX32_I2S_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2s_max32_irq_config_##n(void)                                                 \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2s_max32_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct max32_i2s_dma_config i2s_max32_dma_rx_config_##n = {                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),                            \
		.channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),                              \
		.slot = ADI_ADI_MAX32_DMA_SLOT_I2S_RX,                                             \
	};                                                                                         \
	static const struct max32_i2s_dma_config i2s_max32_dma_tx_config_##n = {                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                            \
		.channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                              \
		.slot = ADI_ADI_MAX32_DMA_SLOT_I2S_TX,                                             \
	};                                                                                         \
	static const struct i2s_max32_config i2s_max32_config_##n = {                              \
		.regs = (mxc_i2s_regs_t *)DT_INST_REG_ADDR(n),                                     \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.perclk =                                                                          \
			{                                                                          \
				.bus = DT_INST_CLOCKS_CELL(n, offset),                             \
				.bit = DT_INST_CLOCKS_CELL(n, bit),                                \
			},                                                                         \
		.irq_config = i2s_max32_irq_config_##n,                                            \
		.rx_dma = i2s_max32_dma_rx_config_##n,                                             \
		.tx_dma = i2s_max32_dma_tx_config_##n,                                             \
	};                                                                                         \
	static struct i2s_max32_data i2s_max32_data_##n = {                                        \
		.tx = {.state = I2S_STATE_NOT_READY},                                              \
		.rx = {.state = I2S_STATE_NOT_READY},                                              \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, i2s_max32_init, NULL, &i2s_max32_data_##n, &i2s_max32_config_##n, \
			      POST_KERNEL, CONFIG_I2S_INIT_PRIORITY, &i2s_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_I2S_INIT)
