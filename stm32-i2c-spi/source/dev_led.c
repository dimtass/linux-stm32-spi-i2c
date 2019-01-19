/*
 * led_pattern.c
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */
#include "dev_led.h"

#define LED_EXISTS(LED, ITTERATOR) ( (LED->port == ITTERATOR->port) && (LED->pin == ITTERATOR->pin) )

void dev_led_module_init(struct dev_led_module * dev)
{
	INIT_LIST_HEAD(&dev->led_list);
}


static inline struct dev_led * dev_led_find_led(struct dev_led_module * dev, struct dev_led * led)
{
	if (!dev || !led) return NULL;
	if (!list_empty(&dev->led_list)) {
		struct dev_led * led_it = NULL;
		list_for_each_entry(led_it, &dev->led_list, list) {
			if LED_EXISTS(led, led_it) {
				/* found */
				return(led_it);
			}
		}
	}
	return NULL;
}


void* dev_led_probe(struct dev_led * led)
{
	struct dev_led_module * dev = led->owner;
	/* do not allow duplicates */
	if (dev_led_find_led(dev, led)) {
		return NULL;
	}

	struct dev_led * new_led = (struct dev_led *) malloc(sizeof(struct dev_led));
	memcpy(new_led, led, sizeof(struct dev_led));

	/* init dev head list */
	INIT_LIST_HEAD(&new_led->list);
	/* Add to led_list */
	list_add(&new_led->list, &dev->led_list);

	if (new_led->port == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	else if (new_led->port == GPIOB) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if (new_led->port == GPIOC) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}

//	new_led->pattern = LED_PATTERN_OFF;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

	/* Configure LED */
	GPIO_InitStructure.GPIO_Pin = new_led->pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(new_led->port, &GPIO_InitStructure);
	new_led->port->ODR &= ~new_led->pin;	//set to 0

	dev_led_set_pattern(new_led, new_led->pattern);
	return (void*)new_led;
}


void dev_led_remove(struct dev_led * led)
{
	struct dev_led_module * dev = led->owner;
	struct dev_led * found_led = dev_led_find_led(dev, led);
	if (found_led) {
		list_del(&found_led->list);
		found_led->port->BRR = found_led->pin;	//set to 0
		/* remove */
		free(found_led);
	}
}


void dev_led_set_pattern(struct dev_led * led, int pattern)
{
	struct dev_led_module * dev = led->owner;
	struct dev_led * found_led = dev_led_find_led(dev, led);
	if (found_led)
		found_led->pattern = (uint8_t) pattern;
}


void dev_led_update(struct dev_led_module * dev)
{
	if (!list_empty(&dev->led_list)) {
		struct dev_led * led_it;
		list_for_each_entry(led_it, &dev->led_list, list) {
			if (led_it->pattern & (1 << dev->led_pattern_index) )
				led_it->port->ODR |= led_it->pin;
			else
				led_it->port->ODR &= ~led_it->pin;
			if ((++dev->led_pattern_index) == 8)
				dev->led_pattern_index = 0;
		}
	}
}

