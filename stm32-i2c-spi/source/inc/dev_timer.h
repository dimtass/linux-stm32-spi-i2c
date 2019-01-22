/**
 * dev_timer.h
 *
 * Copyright 2018 Dimitris Tassopoulos <dimtass@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Notes:
 * This is a header library for dynamic timer objects. The timer objects
 * use malloc to be created, so your toolchain and arch needs to support this.
 * 
 * Usage:
 * This header can be used to initialize multiple timers that each one hosts
 * many different timer objects. To use it properly you need first to create
 * a header list in your main code. E.g.:
 * 	static LIST_HEAD(timer1_list);
 * 	static LIST_HEAD(timer2_list);
 * 
 * The above creates two different timers that each one can host different
 * timer objects. To create a new timer object use this in your main code:
 * 
 * 	// Example led structure
 * 	struct led_t {
 *  	uint16_t tick_ms;
 * 		uint8_ pattern;
 *  };
 * 
 * 	// Instances of leds
 * 	struct led_t led2 = {100, 0b00110011};
 * 	struct led_t led3 = {500, 0b00110011};
 * 	struct led_t led4 = {1000, 0b00110011};
 * 	struct led_t led5 = {2000, 0b00110011};
 * 
 * 	// Callback function that handles all leds in this case (it could be different)
 * 	void led_update(truct led_t * led)
 * 	{
 * 		// Handle led
 * 	}
 * 
 * 	// In your main
 * 	dev_timer_add((void*) &led1, led1.tick_ms, (void*) &led_update, &timer1_list);
 * 	dev_timer_add((void*) &led2, led2.tick_ms, (void*) &led_update, &timer1_list);
 * 	dev_timer_add((void*) &led3, led3.tick_ms, (void*) &led_update, &timer2_list);
 * 	dev_timer_add((void*) &led4, led4.tick_ms, (void*) &led_update, &timer2_list);
 * 
 * 	The above code will add two led objects in each timer and the callback function
 * 	will be triggered for each one.
 * 
 *  Last and most important is that you need a read hw timer to run the timer's
 * 	internal clock. Therefore, for your platform you can setup a HW timer to triggered
 * 	every 1ms and in this timer routine you just need to run these two functions.
 * 
 * 	dev_timer_polling(timer1_list);
 * 	dev_timer_polling(timer2_list);
 * 
 * 	It's just simple as that.
 */


#ifndef DEV_TIMER_H_
#define DEV_TIMER_H_

#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "list.h"	// This is just the Linux kernel's double linked list implementation

/**
 * @brief: Basic timer object.
 * @parent void* This is a pointer to the object that will use the timer
 * @timeout uint16_t This is the number or the main clock ticks for the timer to be triggered
 * @counter uint16_t The count up value that is compared with the timeout_ticks
 * @fp_timeout_cb void(*)(void*) The callback function to call when the timer triggers
 * @list list_head The timer list
 */
struct dev_timer {
	void * parent;
	uint16_t timeout_ticks;
	volatile uint16_t counter;
	void (*fp_timeout_cb)(void *);
	struct list_head list;
};

#define TIMER_EXISTS(TMR, ITTERATOR) ( (TMR->fp_timeout_cb == ITTERATOR->fp_timeout_cb) && (TMR->timeout_ticks == ITTERATOR->timeout_ticks) )

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

static inline struct dev_timer * dev_timer_add(void * object, uint16_t timeout, void * obj_callback, struct list_head * timer_list)
{
	if (!timer_list) return NULL;

	struct dev_timer timer = {
		.parent = object,
		.timeout_ticks = timeout,
		.fp_timeout_cb = obj_callback,
		.counter = 0
	};
	struct dev_timer * new_timer = NULL;
	/* Check if already exists */
	if (!dev_timer_find_timer(&timer, timer_list)) {
		new_timer = (struct dev_timer *) malloc(sizeof(struct dev_timer));
		memcpy(new_timer, &timer, sizeof(struct dev_timer));
		TRACE(("Timer add: %d/%d\n", new_timer->timeout_ticks, new_timer->counter));
		INIT_LIST_HEAD(&new_timer->list);
		list_add(&new_timer->list, timer_list);
	}
	return new_timer;
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
			if ((++tmr_it->counter) >= tmr_it->timeout_ticks) {
				tmr_it->counter = 0;
				tmr_it->fp_timeout_cb(tmr_it->parent);
			}
		}
	}
}

#endif /* DEV_TIMER_H_ */
