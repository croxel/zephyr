/*
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/spinlock.h>
#include "rtio_sched.h"

static struct k_spinlock rio_sched_lock;
static sys_slist_t rtio_sched_list = SYS_SLIST_STATIC_INIT(&rtio_sched_list);

static inline void add_alarm_to_list(struct rtio_sqe *sqe)
{
	struct rtio_sqe *curr_sqe, *prev_sqe = NULL;

	/** We assume we have a sorted list: earlier alarms go first */
	SYS_SLIST_FOR_EACH_CONTAINER(&rtio_sched_list, curr_sqe, delay.node) {
		if (sys_timepoint_cmp(sqe->delay.deadline, curr_sqe->delay.deadline) <= 0) {
			sys_slist_insert(&rtio_sched_list,
					 (prev_sqe ? &prev_sqe->delay.node : NULL),
					 &sqe->delay.node);
			return;
		}
		prev_sqe = curr_sqe;
	}

	sys_slist_append(&rtio_sched_list, &sqe->delay.node);
}

static inline void remove_alarm_from_list(struct rtio_sqe *sqe)
{
	sys_slist_find_and_remove(&rtio_sched_list, &sqe->delay.node);
}

static inline struct rtio_sqe *get_next_alarm_from_list(void)
{
	sys_snode_t *node = sys_slist_peek_head(&rtio_sched_list);

	if (node == NULL) {
		return NULL;
	}

	struct rtio_sqe *sqe = CONTAINER_OF(node, struct rtio_sqe, delay.node);

	return sqe;
}

static void rtio_sched_alarm_expired(struct k_timer *timer)
{
	struct rtio_iodev_sqe *iodev_sqe;

	K_SPINLOCK(&rio_sched_lock) {
		struct rtio_sqe *sqe = get_next_alarm_from_list();

		iodev_sqe = CONTAINER_OF(sqe, struct rtio_iodev_sqe, sqe);

		remove_alarm_from_list(sqe);

		sqe = get_next_alarm_from_list();
		if (sqe != NULL) {
			k_timer_start(timer,
				      sys_timepoint_timeout(sqe->delay.deadline),
				      K_NO_WAIT);
		}
	}

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

static K_TIMER_DEFINE(rtio_delay_timer, rtio_sched_alarm_expired, NULL);

void rtio_sched_alarm(struct rtio_iodev_sqe *iodev_sqe, k_timeout_t timeout)
{
	struct rtio_sqe *sqe = &iodev_sqe->sqe;
	k_timepoint_t *sqe_deadline = &sqe->delay.deadline;

	*sqe_deadline = sys_timepoint_calc(timeout);

	K_SPINLOCK(&rio_sched_lock) {
		struct rtio_sqe *next_sqe;

		add_alarm_to_list(sqe);

		next_sqe = get_next_alarm_from_list();

		/** We assume we have an alarm to schedule, since we just added a new one */
		k_timer_start(&rtio_delay_timer,
			      sys_timepoint_timeout(next_sqe->delay.deadline),
			      K_NO_WAIT);
	}
}
