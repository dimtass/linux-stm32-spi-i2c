/*
 * led_pattern.h
 *
 *	This code handles a LED indicator and can play various
 *	8-bit patterns. You can add your own custom patterns.
 *
 *	Insert the led_update() in any timer IRQ. This will update
 *	the pattern every time is called, so the faster the IRQ the
 *	faster the pattern will change. On each function call the
 *	next bit of the pattern is used on the LED and 0 means off
 *	and 1 means on.
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef DEV_LED_H_
#define DEV_LED_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

/**
 * LED patterns. These are the default patterns
 * for ON/OFF. You need to create your custom
 * patterns with enum, define of raw data.
 * 0: LED off
 * 1: LED on
 */
typedef enum {
	LED_PATTERN_OFF = 0b00000000,
	LED_PATTERN_ON = 0b11111111,
} en_led_pattern;

#define DECLARE_MODULE_LED(NAME, TICK_MS) \
	struct dev_led_module NAME = { \
		.tick_ms = TICK_MS, \
		.led_pattern_index = 0, \
	}

struct dev_led_module {
	uint8_t		led_pattern_index;
	uint16_t	tick_ms;
	struct list_head led_list;
};

#define DECLARE_DEV_LED(NAME, PORT, PIN, OWNER) \
	struct dev_led NAME = { \
		.owner = OWNER, \
		.port = PORT, \
		.pin = PIN, \
		.pattern = LED_PATTERN_OFF, \
	}

struct dev_led {
	struct dev_led_module * owner;
	GPIO_TypeDef * port;
	uint16_t 	pin;
	uint8_t		pattern;
	struct list_head list;
};


/**
 * @brief Initialize LED
 * @param[in] dev_led A pointer to the LED dev
 */
void dev_led_module_init(struct dev_led_module * dev);

void * dev_led_probe(struct dev_led * led);

void dev_led_remove(struct dev_led * led);

/**
 * @brief Update the LED pattern
 */
void dev_led_update(struct dev_led_module * dev);

/**
 * @brief Set the active pattern
 * @param[in] dev_led A pointer to the LED dev struct
 * @param[in] pattern The pattern to set as active.This is truncated to uint8_t internally
 */
void dev_led_set_pattern(struct dev_led * led, int pattern);

#endif /* DEV_LED_H_ */
