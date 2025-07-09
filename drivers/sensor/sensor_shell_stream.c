/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "sensor_shell.h"

/* Create a single common config for streaming */
static struct sensor_shell_processing_context ctx;
static struct rtio_sqe *current_streaming_handle;
static struct sensor_stream_trigger iodev_sensor_shell_trigger;
static struct sensor_read_config iodev_sensor_shell_stream_config = {
	.sensor = NULL,
	.is_streaming = true,
	.triggers = &iodev_sensor_shell_trigger,
	.count = 0,
	.max = 1,
};
RTIO_IODEV_DEFINE(iodev_sensor_shell_stream, &__sensor_iodev_api,
		  &iodev_sensor_shell_stream_config);

static void sensor_shell_processing_entry_point(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (true) {
		struct rtio_cqe *cqe = rtio_cqe_consume_block(&sensor_read_rtio);
		if (!cqe) {
			continue;
		}

		int result = cqe->result;
		void *userdata = cqe->userdata;
		uint8_t *buf = NULL;
		uint32_t buf_len = 0;

		rtio_cqe_get_mempool_buffer(&sensor_read_rtio, cqe, &buf, &buf_len);

		sensor_shell_processing_callback(result, buf, buf_len, userdata);

		rtio_release_buffer(&sensor_read_rtio, buf, buf_len);

		if (cqe->flags | RTIO_CQE_FLAG_MULTISHOT_STOPPED) {
			rtio_sqe_cancel(current_streaming_handle);

			int rc = sensor_stream(&iodev_sensor_shell_stream, &sensor_read_rtio, &ctx,
					&current_streaming_handle);

			if (rc != 0) {
				shell_error(ctx.sh, "Failed to restart stream: %d", rc);
			} else {
				shell_info(ctx.sh, "Restarted stream");
			}
		}
		rtio_cqe_release(&sensor_read_rtio, cqe);
	}
}
K_THREAD_DEFINE(sensor_shell_processing_tid, CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE,
		sensor_shell_processing_entry_point, NULL, NULL, NULL, 0, 0, 0);

int cmd_sensor_stream(const struct shell *sh, size_t argc, char *argv[])
{
	const struct device *dev = device_get_binding(argv[1]);

	if (argc != 5 && argc != 3) {
		shell_error(sh, "Wrong number of arguments (%zu)", argc);
		return -EINVAL;
	}

	if (dev == NULL) {
		shell_error(sh, "Device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	if (current_streaming_handle != NULL) {
		shell_info(sh, "Disabling existing stream");
		rtio_sqe_cancel(current_streaming_handle);
	}

	if (strcmp("off", argv[2]) == 0) {
		return 0;
	}

	if (strcmp("on", argv[2]) != 0) {
		shell_error(sh, "Unknown streaming operation (%s)", argv[2]);
		return -EINVAL;
	}

	if (strcmp("double_tap", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_DOUBLE_TAP;
	} else if (strcmp("data_ready", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_DATA_READY;
	} else if (strcmp("delta", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_DELTA;
	} else if (strcmp("freefall", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_FREEFALL;
	} else if (strcmp("motion", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_MOTION;
	} else if (strcmp("near_far", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_NEAR_FAR;
	} else if (strcmp("stationary", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_STATIONARY;
	} else if (strcmp("threshold", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_THRESHOLD;
	} else if (strcmp("fifo_wm", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_FIFO_WATERMARK;
	} else if (strcmp("fifo_full", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_FIFO_FULL;
	} else if (strcmp("tap", argv[3]) == 0) {
		iodev_sensor_shell_trigger.trigger = SENSOR_TRIG_TAP;
	} else {
		shell_error(sh, "Invalid trigger (%s)", argv[3]);
		return -EINVAL;
	}

	if (strcmp("incl", argv[4]) == 0) {
		iodev_sensor_shell_trigger.opt = SENSOR_STREAM_DATA_INCLUDE;
	} else if (strcmp("drop", argv[4]) == 0) {
		iodev_sensor_shell_trigger.opt = SENSOR_STREAM_DATA_DROP;
	} else if (strcmp("nop", argv[4]) == 0) {
		iodev_sensor_shell_trigger.opt = SENSOR_STREAM_DATA_NOP;
	} else {
		shell_error(sh, "Unknown trigger op (%s)", argv[4]);
		return -EINVAL;
	}

	shell_print(sh, "Enabling stream...");
	iodev_sensor_shell_stream_config.sensor = dev;

	iodev_sensor_shell_stream_config.count = 1;

	ctx.dev = dev;
	ctx.sh = sh;

	int rc = sensor_stream(&iodev_sensor_shell_stream, &sensor_read_rtio, &ctx,
			       &current_streaming_handle);

	if (rc != 0) {
		shell_error(sh, "Failed to start stream");
	}
	return rc;
}
