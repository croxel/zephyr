/*
 * Copyright (c) 2025 Analog Devices
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_i2s

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/types.h>
#include <soc.h>
#include <Wrap_max32_i2s.h>

LOG_MODULE_REGISTER(i2s_max32, CONFIG_I2S_LOG_LEVEL);

#define USE_DMA_RELOAD 1

#define I2S_WORD_SIZE_TO_BITS(ws) ((ws) * 8)
#define I2S_DMA_BUF_SIZE(ws, ch)  ((ws) * (ch))

struct i2s_mem_block {
	void *block;
	size_t size;
};

struct i2s_max32_stream_data {
	volatile enum i2s_state state;
	volatile bool drain;
	volatile struct i2s_mem_block cur_block;
	struct k_msgq *queue;
	struct i2s_config i2s_cfg;
};

struct i2s_max32_stream {
	struct i2s_max32_stream_data *data;
	struct {
		mxc_i2s_regs_t *reg;
		const struct device *dev;
	} i2s;
	struct {
		const struct device *dev;
		uint32_t channel;
		uint32_t slot;
	} dma;
};

struct i2s_max32_cfg {
	const struct i2s_max32_stream tx;
	const struct i2s_max32_stream rx;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	uint32_t clk_freq;
};

static void i2s_max32_tx_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status);
static void i2s_max32_rx_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status);

/**
 * @brief configure the linked dma channel for the stream and start transfer.
 * in case of RX stream, it allocates a new memory block from the slab,
 * in case of TX stream, it gets the next block from the queue.
 *
 * @param stream Stream to start
 * @param dir Direction of the stream (RX or TX)
 * @return 0 on success, negative error code on failure.
 */
static inline int start_stream(const struct i2s_max32_stream *stream, enum i2s_dir dir)
{
	int ret;
	struct i2s_max32_stream_data *stream_data = stream->data;
	struct dma_block_config dma_block = {
		.block_size = stream_data->i2s_cfg.block_size,
	};
	struct dma_config dma_cfg = {
		.dma_slot = stream->dma.slot,
		.source_data_size = stream_data->i2s_cfg.word_size / 8,
		.dest_data_size = stream_data->i2s_cfg.word_size / 8,
		.source_burst_length = stream_data->i2s_cfg.word_size / 8,
		.dest_burst_length = stream_data->i2s_cfg.word_size / 8,
		.block_count = 1,
		.user_data = (void *)stream,
		.head_block = &dma_block,
	};

	if ((dir != I2S_DIR_RX) && (dir != I2S_DIR_TX)) {
		LOG_ERR("Invalid I2S direction: %d", (int)dir);
		return -EINVAL;
	}

	if (dir == I2S_DIR_RX) {
		if (stream_data->cur_block.block != NULL) {
			LOG_ERR("RX stream already running");
			return -EIO;
		}
		ret = k_mem_slab_alloc(stream_data->i2s_cfg.mem_slab, &stream_data->cur_block.block,
				       K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to allocate RX mem block: %d", ret);
			return -ENOMEM;
		}
		stream_data->cur_block.size = dma_block.block_size;
		/* Configure DMA Block */
		dma_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_block.dest_address = (uint32_t)stream_data->cur_block.block;
		dma_block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		/* Configure DMA */
		dma_cfg.source_data_size = 4; // always word
		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.dma_callback = i2s_max32_rx_dma_callback;
		// RX DMA Threshold
		stream->i2s.reg->dmach0 |= (4 << MXC_F_I2S_DMACH0_DMA_RX_THD_VAL_POS);

	} else {
		if (k_msgq_num_used_get(stream_data->queue) == 0) {
			LOG_ERR("TX queue empty");
			return -EIO;
		}
		ret = k_msgq_get(stream_data->queue, &stream_data->cur_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to get item from TX queue: %d", ret);
			return ret;
		}
		stream_data->cur_block.size = dma_block.block_size;
		/* Configure DMA Block */
		dma_block.source_address = (uint32_t)stream_data->cur_block.block;
		dma_block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		/* Configure DMA */
		dma_cfg.dest_data_size = 4; // always word
		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.dma_callback = i2s_max32_tx_dma_callback;
		// RX DMA Threshold
		stream->i2s.reg->dmach0 |= (4 << MXC_F_I2S_DMACH0_DMA_TX_THD_VAL_POS);
	}

	/* Configure DMA channel */
	ret = dma_config(stream->dma.dev, stream->dma.channel, &dma_cfg);
	if (ret < 0) {
		LOG_ERR("DMA config failed with error: %d", ret);
		goto start_stream_error;
	}

	if (dir == I2S_DIR_RX) {
		Wrap_MXC_I2S_RXEnable(stream->i2s.reg);
	} else {
		Wrap_MXC_I2S_TXEnable(stream->i2s.reg);
	}

	/* Start DMA transfer */
	ret = dma_start(stream->dma.dev, stream->dma.channel);
	if (ret < 0) {
		LOG_ERR("DMA start failed with error: %d", ret);
		goto start_stream_error;
	}

	stream_data->state = I2S_STATE_RUNNING;
	return 0;

start_stream_error:
	/* we have either allocated for rx or user provided tx buffer that we failed to send,
	 * Thus in both cases this need to free the block */
	k_mem_slab_free(stream_data->i2s_cfg.mem_slab, stream_data->cur_block.block);
	stream_data->cur_block.block = NULL;
	stream_data->cur_block.size = 0;
	return ret;
}

/**
 * @brief it does not stop the stream immediately, but only signals it to stop.
 * The actual stop is handled in the DMA callback.
 * transition to stopping state.
 *
 * @param stream Stream to stop
 * @param drain only used for TX stream, if true, the only current block is transferred,
 * otherwise, the stream all blocks in the queue are transferred before stopping.
 */
static inline int stop_stream(const struct i2s_max32_stream *stream, bool drain)
{
	struct i2s_max32_stream_data *stream_data = stream->data;
	/* signal stopping to handle in the dma callback */
	stream_data->state = I2S_STATE_STOPPING;
	/* use to control drain/drop behaviour */
	stream_data->drain = drain;
	/* */
	LOG_DBG("Stopping stream %p, drain: %d", (void *)stream, drain);
	return 0;
}

/**
 * @brief clean the stream, it will free memory blocks from the queue.
 * in case of TX stream, it will free the blocks that are not sent yet.
 * in case of RX stream, it will free the blocks that are not yet read.
 *
 * @param stream Stream to clear
 * @return 0 on success, negative error code on failure.
 */
static inline void purge_stream(const struct i2s_max32_stream *stream)
{
	struct i2s_max32_stream_data *stream_data = stream->data;
	struct i2s_mem_block block;

	/* Clear transient block */
	if (stream->data->cur_block.block != NULL) {
		k_mem_slab_free(stream_data->i2s_cfg.mem_slab, stream->data->cur_block.block);
		stream->data->cur_block.block = NULL;
		stream->data->cur_block.size = 0;
	}

	/* Clear the queue */
	while (k_msgq_get(stream_data->queue, &block, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream_data->i2s_cfg.mem_slab, block.block);
	}
}

/**
 * @brief terminate the stream, it will stop the DMA immediately, and clear the queue.
 *
 * @param stream Stream to terminate
 * @return 0 on success, negative error code on failure.
 */
static int terminate_stream(const struct i2s_max32_stream *stream)
{
	struct i2s_max32_stream_data *stream_data = stream->data;

	if (stream_data->state != I2S_STATE_RUNNING) {
		LOG_ERR("Stream not running");
		return -EIO;
	}

	stream_data->state = I2S_STATE_STOPPING;
	/* stop dma immediately */
	dma_stop(stream->dma.dev, stream->dma.channel);
	/* Clear the queue */
	purge_stream(stream);
	/* reset state */
	stream_data->state = I2S_STATE_READY;
	return 0;
}

static inline int restart_stream(const struct i2s_max32_stream *stream, enum i2s_dir dir)
{
	int err;
	struct i2s_max32_stream_data *stream_data = stream->data;
	void *src = NULL, *dst = NULL;

	if ((stream_data->state != I2S_STATE_RUNNING) &&
	    (stream_data->state != I2S_STATE_STOPPING)) {
		LOG_ERR("Stream not running");
		return -EIO;
	}

	/* reload dma with new address */
	if (dir == I2S_DIR_RX) {
		dst = stream_data->cur_block.block;
	} else if (dir == I2S_DIR_TX) {
		src = stream_data->cur_block.block;
	} else {
		LOG_ERR("Invalid DMA direction");
		return -EINVAL;
	}
	err = dma_reload(stream->dma.dev, stream->dma.channel, (uint32_t)src, (uint32_t)dst,
			 stream_data->i2s_cfg.block_size);
	if (err < 0) {
		LOG_ERR("Error reloading DMA channel[%d]: %d", (int)stream->dma.channel, err);
		return -EIO;
	}

	err = dma_start(stream->dma.dev, stream->dma.channel);
	if (err < 0) {
		LOG_ERR("Error starting DMA channel[%d]: %d", (int)stream->dma.channel, err);
		return -EIO;
	}
	return 0;
}

static void i2s_max32_tx_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status)
{
	int err;
	const struct i2s_max32_stream *stream = (const struct i2s_max32_stream *)arg;
	struct i2s_max32_stream_data *stream_data = stream->data;
	LOG_DBG("TX DMA callback, status: %d, of %d", status,
		(uint32_t)stream_data->cur_block.block);
	/* irrespective of success/fail free the block we are working with */
	if (stream_data->cur_block.block != NULL) {
		k_mem_slab_free(stream_data->i2s_cfg.mem_slab, stream_data->cur_block.block);
		stream_data->cur_block.block = NULL;
	}
	/* check the status of the DMA transfer */
	if (status < 0) {
		LOG_ERR("TX DMA status bad: %d", status);
		goto tx_cb_handle_error;
	}
	/* if we are stop was requested
	 * without draining, or the queue drained completely,
	 * then we can stop the stream */
	if ((stream_data->state == I2S_STATE_STOPPING) &&
	    ((stream_data->drain == false) || (k_msgq_num_used_get(stream_data->queue) == 0))) {
		stream_data->state = I2S_STATE_READY;
		LOG_DBG("TX stream stopped, state: %d", (int)stream_data->state);
		return;
	}
#if USE_DMA_RELOAD
	/* get next block and restart stream */
	err = k_msgq_get(stream_data->queue, &stream_data->cur_block, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("Failed to get item from TX queue: %d", err);
		goto tx_cb_handle_error;
	}
	err = restart_stream(stream, I2S_DIR_TX);
	if (err < 0) {
		LOG_ERR("Failed to restart TX transfer: %d", err);
		goto tx_cb_handle_error;
	}
#else
	/* stop the DMA and start it again with a new block */
	err = dma_stop(stream->dma.dev, stream->dma.channel);
	if (err < 0) {
		LOG_ERR("Failed to stop TX DMA: %d", err);
		goto tx_cb_handle_error;
	}
	err = start_stream(stream, I2S_DIR_TX);
	if (err < 0) {
		LOG_ERR("Failed to restart TX transfer: %d", err);
		goto tx_cb_handle_error;
	}
#endif
	return;
tx_cb_handle_error:
	LOG_ERR("TX DMA callback error, status: %d", status);
	stream_data->state = I2S_STATE_ERROR;
	if (stream_data->cur_block.block != NULL) {
		k_mem_slab_free(stream_data->i2s_cfg.mem_slab, stream_data->cur_block.block);
		stream_data->cur_block.block = NULL;
	}
	return;
}

static void i2s_max32_rx_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status)
{
	int err;
	const struct i2s_max32_stream *stream = (const struct i2s_max32_stream *)arg;
	struct i2s_max32_stream_data *stream_data = stream->data;
	if (status < 0) {
		LOG_ERR("RX DMA status bad: %d", status);
		goto rx_cb_handle_error;
	}
	/* we have complexly received the block, push to queue to let user handle and free it*/
	err = k_msgq_put(stream_data->queue, &stream_data->cur_block, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("Failed to put item to RX queue: %d", err);
		goto rx_cb_handle_error;
	}
	if (stream_data->state == I2S_STATE_STOPPING) {
		stream_data->state = I2S_STATE_READY;
		return;
	} else {

#if USE_DMA_RELOAD
		/* allocate new buffer and continue receiving */
		err = k_mem_slab_alloc(stream_data->i2s_cfg.mem_slab, &stream_data->cur_block.block,
				       K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Failed to allocate RX mem block: %d", err);
			goto rx_cb_handle_error;
		}

		err = restart_stream(stream, I2S_DIR_RX);
		if (err < 0) {
			LOG_ERR("Failed to restart RX transfer: %d", err);
			goto rx_cb_handle_error;
		}
#else
		/* stop the DMA and start it again with a new block */
		stream_data->cur_block.block = NULL;
		err = dma_stop(stream->dma.dev, stream->dma.channel);
		if (err < 0) {
			LOG_ERR("Failed to stop RX DMA: %d", err);
			goto rx_cb_handle_error;
		}
		err = start_stream(stream, I2S_DIR_RX);
		if (err < 0) {
			LOG_ERR("Failed to restart RX transfer: %d", err);
			goto rx_cb_handle_error;
		}
#endif
	}
	return;

rx_cb_handle_error:
	LOG_ERR("RX DMA callback error, status: %d", status);
	stream_data->state = I2S_STATE_ERROR;
	if (stream_data->cur_block.block != NULL) {
		k_mem_slab_free(stream_data->i2s_cfg.mem_slab, stream_data->cur_block.block);
		stream_data->cur_block.block = NULL;
	}
	return;
}

static int i2s_max32_trigger_single(const struct device *dev, enum i2s_dir dir,
				    enum i2s_trigger_cmd cmd, const struct i2s_max32_stream *stream)
{
	struct i2s_max32_stream_data *stream_data = stream->data;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream_data->state != I2S_STATE_READY) {
			LOG_ERR("START - Invalid state: %d", (int)stream_data->state);
			return -EIO;
		}
		return start_stream(stream, dir);

	case I2S_TRIGGER_STOP:
		if (stream_data->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP - Invalid state: %d", (int)stream_data->state);
			return -EIO;
		}
		return stop_stream(stream, false);
	case I2S_TRIGGER_DRAIN:
		if (stream_data->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN - Invalid state: %d", (int)stream_data->state);
			return -EIO;
		}
		return stop_stream(stream, true);
	case I2S_TRIGGER_DROP:
		if (stream_data->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP - Invalid state: %d", (int)stream_data->state);
			return -EIO;
		}
		terminate_stream(stream);
		stream_data->state = I2S_STATE_READY;
		return 0;
	case I2S_TRIGGER_PREPARE:
		if (stream_data->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE - Invalid state: %d", (int)stream_data->state);
			return -EIO;
		}
		return terminate_stream(stream);
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int i2s_max32_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	int ret;
	const struct i2s_max32_cfg *cfg = dev->config;

	LOG_DBG("i2s_max32_trigger: dir=%d, cmd=%d", (int)dir, (int)cmd);

	switch (dir) {
	case I2S_DIR_TX:
		return i2s_max32_trigger_single(dev, dir, cmd, &cfg->tx);
	case I2S_DIR_RX:
		return i2s_max32_trigger_single(dev, dir, cmd, &cfg->rx);
	case I2S_DIR_BOTH:

		ret = i2s_max32_trigger_single(dev, I2S_DIR_RX, cmd, &cfg->rx);
		if (ret < 0) {
			return ret;
		}

		ret = i2s_max32_trigger_single(dev, I2S_DIR_TX, cmd, &cfg->tx);
		if (ret < 0) {
			return ret;
		}

		return 0;
	default:
		LOG_ERR("Invalid I2S direction: %d", (int)dir);
		return -EINVAL;
	}
}

static int i2s_cfg_to_max32_cfg(const struct i2s_config *i2s_cfg, mxc_i2s_req_t *max_cfg)
{
	/* Input validation */
	__ASSERT(i2s_cfg != NULL, "i2s_cfg cannot be NULL");
	__ASSERT(max_cfg != NULL, "max_cfg cannot be NULL");

	/* Clear configuration struct */
	memset(max_cfg, 0, sizeof(mxc_i2s_req_t));

	/* Validate word size */
	switch (i2s_cfg->word_size) {
	case 8:
		max_cfg->wordSize = MXC_I2S_WSIZE_BYTE;
		max_cfg->sampleSize = MXC_I2S_SAMPLESIZE_EIGHT;
		break;
	case 16:
		max_cfg->wordSize = MXC_I2S_WSIZE_HALFWORD;
		max_cfg->sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
		break;
	case 32:
		max_cfg->wordSize = MXC_I2S_WSIZE_WORD;
		max_cfg->sampleSize = MXC_I2S_SAMPLESIZE_THIRTYTWO;
		break;
	default:
		LOG_ERR("Unsupported word size: %u", i2s_cfg->word_size);
		return -EINVAL;
	}

	/* Validate channels */
	if (i2s_cfg->channels == 2) {
		max_cfg->stereoMode = MXC_I2S_STEREO;
	} else if (i2s_cfg->channels == 1) {
		max_cfg->stereoMode = MXC_I2S_MONO_RIGHT_CH;
	} else {
		LOG_ERR("Unsupported number of channels: %u", i2s_cfg->channels);
		return -EINVAL;
	}

	/* Validate format */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		max_cfg->justify = MXC_I2S_LSB_JUSTIFY;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		max_cfg->justify = MXC_I2S_MSB_JUSTIFY;
		break;
	default:
		LOG_ERR("Unsupported data format: 0x%02x", i2s_cfg->format);
		return -EINVAL;
	}

	/* Check unsupported format options */
	if (i2s_cfg->format &
	    (I2S_FMT_DATA_ORDER_LSB | I2S_FMT_BIT_CLK_INV | I2S_FMT_FRAME_CLK_INV)) {
		LOG_ERR("Unsupported format options: 0x%02x", i2s_cfg->format);
		return -EINVAL;
	}

	/* Set master/slave mode */
	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) {
		max_cfg->channelMode = MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS;
	} else {
		max_cfg->channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
	}

	/* Check unsupported options */
	if (i2s_cfg->options & (I2S_OPT_LOOPBACK | I2S_OPT_PINGPONG)) {
		LOG_ERR("Unsupported options: 0x%02x", i2s_cfg->options);
		return -EINVAL;
	}

	/* Set standard values */
	max_cfg->bitOrder = MXC_I2S_LSB_FIRST;
	max_cfg->wsPolarity = MXC_I2S_POL_NORMAL;
	max_cfg->bitsWord = i2s_cfg->word_size;
	max_cfg->adjust = MXC_I2S_ADJUST_LEFT;

	/* Calculate clock divider for sample rate */
	max_cfg->clkdiv =
		Wrap_MXC_I2S_CalculateClockDiv(i2s_cfg->frame_clk_freq, max_cfg->wordSize);
	if (max_cfg->clkdiv < 0) {
		LOG_ERR("Invalid frame clock frequency: %u", i2s_cfg->frame_clk_freq);
		return -EINVAL;
	}

	return 0;
}

static int i2s_max32_configure_single(const struct device *dev, enum i2s_dir dir,
				      const struct i2s_config *i2s_cfg,
				      const struct i2s_max32_stream *stream)
{
	struct i2s_max32_stream_data *stream_data = stream->data;
	mxc_i2s_req_t mxc_cfg = {0};
	int ret;

	if (stream_data->state != I2S_STATE_NOT_READY && stream_data->state != I2S_STATE_READY) {
		LOG_ERR("Invalid state: %d", (int)stream_data->state);
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		terminate_stream(stream);
		stream_data->state = I2S_STATE_NOT_READY;
		return 0;
	}

	ret = i2s_cfg_to_max32_cfg(i2s_cfg, &mxc_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to convert I2S config to MAX32 config");
		return ret;
	}

	// TODO: Implement Wrap_MXC_I2S_Init and Wrap_MXC_I2S_SetIntEn
	// ret = MXC_I2S_Init(config->i2s, &mxc_cfg);

	uint32_t buf0;
	mxc_cfg.rxData = &buf0;
	mxc_cfg.length = sizeof(buf0) / sizeof(uint32_t);

	ret = MXC_I2S_Init(&mxc_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to initialize I2S: %d", ret);
		return ret;
	}

	stream_data->i2s_cfg = *i2s_cfg;
	stream_data->state = I2S_STATE_READY;
	return 0;
}

static int i2s_max32_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	int ret;
	const struct i2s_max32_cfg *cfg = dev->config;

	LOG_DBG("i2s_max32_configure: dir=%d, word_size=%u, channels=%u, frame_clk_freq=%u",
		(int)dir, i2s_cfg->word_size, i2s_cfg->channels, i2s_cfg->frame_clk_freq);

	switch (dir) {
	case I2S_DIR_TX:
		return i2s_max32_configure_single(dev, dir, i2s_cfg, &cfg->tx);
	case I2S_DIR_RX:
		return i2s_max32_configure_single(dev, dir, i2s_cfg, &cfg->rx);
	case I2S_DIR_BOTH:
		/* If user requests both TX and RX configuration, we can configure both streams
		 * with the same configuration. This is useful for full-duplex operation.
		 * Failing to configure any stream will result in an error.
		 * This could mean one has been configured successfully and the other has failed.
		 * This is not an error condition, as the user can choose to configure only one
		 * stream at a time.
		 */

		ret = i2s_max32_configure_single(dev, I2S_DIR_RX, i2s_cfg, &cfg->rx);
		if (ret < 0) {
			return ret;
		}

		ret = i2s_max32_configure_single(dev, I2S_DIR_TX, i2s_cfg, &cfg->tx);
		if (ret < 0) {
			return ret;
		}
		return 0;
	default:
		LOG_ERR("Invalid I2S direction: %d", (int)dir);
		return -EINVAL;
	}
}

static int i2s_max32_read(const struct device *dev, void **mem_block, size_t *size)
{
	const struct i2s_max32_cfg *cfg = dev->config;
	const struct i2s_max32_stream *stream = &cfg->rx;
	struct i2s_max32_stream_data *stream_data = stream->data;
	struct i2s_mem_block block;
	int err;

	if (stream_data->state == I2S_STATE_NOT_READY) {
		LOG_ERR("RX invalid state: %d", (int)stream_data->state);
		return -EIO;
	} else if (stream_data->state == I2S_STATE_ERROR &&
		   k_msgq_num_used_get(stream_data->queue) == 0) {
		LOG_ERR("RX queue empty");
		return -EIO;
	}

	err = k_msgq_get(stream_data->queue, &block, K_MSEC(stream_data->i2s_cfg.timeout));
	if (err < 0) {
		LOG_ERR("RX queue empty");
		return err;
	}

	LOG_DBG("RX got block: %p, size: %zu", block.block, block.size);

	*mem_block = block.block;
	*size = block.size;

	return 0;
}

static int i2s_max32_write(const struct device *dev, void *mem_block, size_t size)
{
	const struct i2s_max32_cfg *cfg = dev->config;
	const struct i2s_max32_stream *stream = &cfg->tx;
	struct i2s_max32_stream_data *stream_data = stream->data;
	struct i2s_mem_block block = {.block = mem_block, .size = size};
	int err;

	if (stream_data->state != I2S_STATE_READY && stream_data->state != I2S_STATE_RUNNING) {
		LOG_ERR("TX Invalid state: %d", (int)stream_data->state);
		return -EIO;
	}

	if (size > stream_data->i2s_cfg.block_size) {
		LOG_ERR("Max write size is: %u", (unsigned int)stream_data->i2s_cfg.block_size);
		return -EINVAL;
	}

	err = k_msgq_put(stream_data->queue, &block, K_MSEC(stream_data->i2s_cfg.timeout));
	if (err < 0) {
		LOG_ERR("TX queue full");
		return err;
	}

	return 0;
}

static const struct i2s_driver_api i2s_max32_driver_api = {
	.read = i2s_max32_read,
	.write = i2s_max32_write,
	.configure = i2s_max32_configure,
	.trigger = i2s_max32_trigger,
};

static int i2s_max32_init(const struct device *dev)
{
	const struct i2s_max32_cfg *config = dev->config;
	int err;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	config->irq_config_func(dev);

	return 0;
}

static void i2s_max32_isr(const struct device *dev)
{
}

#define I2S_MAX32_CONFIG_IRQ_FUNC(n) .irq_config_func = i2s_max32_irq_config_func_##n

#define MAX32_DT_INST_DMA_CTLR(n, name) DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, name))

#define MAX32_DT_INST_DMA_CELL(n, name, cell) DT_INST_DMAS_CELL_BY_NAME(n, name, cell)

#define I2S_MAX32_IRQ_CONFIG_FUNC(n)                                                               \
	static void i2s_max32_irq_config_func_##n(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2s_max32_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define I2S_MAX32_STREAM_DATA_CREATE_AND_INIT(n, dir)                                              \
	K_MSGQ_DEFINE(i2s_max32_##dir##_q_##n, sizeof(struct i2s_mem_block),                       \
		      CONFIG_I2S_MAX32_QUEUE_SIZE, 1);                                             \
	static struct i2s_max32_stream_data i2s_max32_##dir##_data_##n = {                         \
		.state = I2S_STATE_NOT_READY,                                                      \
		.drain = false,                                                                    \
		.queue = &i2s_max32_##dir##_q_##n,                                                 \
	};

#define I2S_MAX32_STREAM_INIT(n, dir)                                                              \
	{                                                                                          \
		.data = &i2s_max32_##dir##_data_##n,                                               \
		.i2s =                                                                             \
			{                                                                          \
				.reg = (mxc_i2s_regs_t *)DT_INST_REG_ADDR(n),                      \
				.dev = DEVICE_DT_INST_GET(n),                                      \
			},                                                                         \
		.dma =                                                                             \
			{                                                                          \
				.dev = MAX32_DT_INST_DMA_CTLR(n, dir),                             \
				.channel = MAX32_DT_INST_DMA_CELL(n, dir, channel),                \
				.slot = MAX32_DT_INST_DMA_CELL(n, dir, slot),                      \
			},                                                                         \
	}

#define I2S_MAX32_INIT(n)                                                                          \
	PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                                         \
	I2S_MAX32_IRQ_CONFIG_FUNC(n)                                                               \
                                                                                                   \
	I2S_MAX32_STREAM_DATA_CREATE_AND_INIT(n, tx);                                              \
	I2S_MAX32_STREAM_DATA_CREATE_AND_INIT(n, rx);                                              \
                                                                                                   \
	static const struct i2s_max32_cfg i2s_max32_cfg_##n = {                                    \
		.tx = I2S_MAX32_STREAM_INIT(n, tx),                                                \
		.rx = I2S_MAX32_STREAM_INIT(n, rx),                                                \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(n)),                                 \
		.clk_freq = DT_INST_PROP(n, clock_frequency),                                      \
		I2S_MAX32_CONFIG_IRQ_FUNC(n)};                                                     \
	DEVICE_DT_INST_DEFINE(n, i2s_max32_init, NULL, NULL, &i2s_max32_cfg_##n, POST_KERNEL,      \
			      CONFIG_I2S_INIT_PRIORITY, &i2s_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_MAX32_INIT)
