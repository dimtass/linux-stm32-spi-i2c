/*
 * dev_timer.h
 *
 *  Created on: 13 May 2018
 *      Author: dimtass
 */

#ifndef DEV_TIMER_H_
#define DEV_TIMER_H_

#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "list.h"

struct dev_timer {
	void * parent;
	uint16_t timeout_ms;
	volatile uint16_t counter;
	void (*fp_timeout_cb)(void *);
	struct list_head list;
};

#define TIMER_EXISTS(TMR, ITTERATOR) ( (TMR->fp_timeout_cb == ITTERATOR->fp_timeout_cb) && (TMR->timeout_ms == ITTERATOR->timeout_ms) )

static inline struct dev_timer * dev_timer_find_timer(struct dev_timer * tmr, struct list_head * timer_list)
{
	if (!list_empty(timer_list)) {
		struct dev_timer * tmr_it = NULL;
		list_for_each_entry(tmr_it, timer_list, list) {
			if TIMER_EXISTS(tmr, tmr_it) {
				/* found */
				return(tmr_it);
			}
		}
	}
	return NULL;
}

static inline void dev_timer_add(void * object, uint16_t timeout, void * obj_callback, struct list_head * timer_list)
{
	struct dev_timer timer = {
		.parent = object,
		.timeout_ms = timeout,
		.fp_timeout_cb = obj_callback,
		.counter = 0
	};
	/* Check if already exists */
	if (!timer_list) return;
	if (!dev_timer_find_timer(&timer, timer_list)) {
		struct dev_timer * new_timer = (struct dev_timer *) malloc(sizeof(struct dev_timer));
		memcpy(new_timer, &timer, sizeof(struct dev_timer));
		TRACE(("Timer add: %d/%d\n", new_timer->timeout_ms, new_timer->counter));
		INIT_LIST_HEAD(&new_timer->list);
		list_add(&new_timer->list, timer_list);
	}
}

static inline void dev_timer_del(struct dev_timer * timer, struct list_head * timer_list)
{
	struct dev_timer * found_timer = dev_timer_find_timer(timer, timer_list);
	if (found_timer) {
		/* remove */
		list_del(&found_timer->list);
		free(found_timer);
	}
}

static inline void dev_timer_polling(struct list_head * timer_list)
{
	if (!list_empty(timer_list)) {
		struct dev_timer * tmr_it = NULL;
		list_for_each_entry(tmr_it, timer_list, list) {
			if ((++tmr_it->counter) >= tmr_it->timeout_ms) {
				tmr_it->counter = 0;
				tmr_it->fp_timeout_cb(tmr_it->parent);
			}
		}
	}
}

#endif /* DEV_TIMER_H_ */
