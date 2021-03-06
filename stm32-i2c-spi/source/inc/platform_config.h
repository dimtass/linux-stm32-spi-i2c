/*
 * platform_config.h
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "misc.h"

/**
 * Trace levels for this project.
 * Have in mind that these are bit flags!
 */
enum en_trace_level {
	TRACE_LEVEL_DEFAULT = 	(1 << 0),
	TRACE_LEVEL_SPI = 		(1 << 1),
	TRACE_LEVEL_I2C = 		(1 << 2),
	TRACE_LEVEL_UART = 		(1 << 3),
	TRACE_LEVEL_PWM = 		(1 << 4),
	TRACE_LEVEL_ADC = 		(1 << 5),
};

#define DEBUG_TRACE

#ifdef DEBUG_TRACE
#define TRACE(X) TRACEL(TRACE_LEVEL_DEFAULT, X)
#define TRACEL(TRACE_LEVEL, X) do { if (glb.trace_levels & TRACE_LEVEL) printf X;} while(0)
#else
#define TRACE(X)
#define TRACEL(X,Y)
#endif


#define USE_DEBUG_PIN
#define DEBUG_PIN	GPIO_Pin_14
#define DEBUG_PORT 	GPIOC

/* on reset all of these will be set to 0 */
struct tp_glb {
	uint16_t	tmr_1ms;
	uint8_t		mode;
	enum en_trace_level trace_levels;
};

extern volatile struct tp_glb glb;

static inline void set_trace_level(enum en_trace_level level, uint8_t enable)
{
	if (enable) {
		glb.trace_levels |= (uint32_t) level;
	}
	else {
		glb.trace_levels &= ~((uint32_t) level);
	}
}

#endif /* __PLATFORM_CONFIG_H */
