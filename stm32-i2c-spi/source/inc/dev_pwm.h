/*
 * dev_pwm.h
 * Timer channels and GPIOs
 *  TIM1.1: PA.8
 *  TIM1.2: PA.9
 *  TIM1.3: PA.10
 *  TIM1.4: PA.11
 * 
 *  TIM2.1: PA.0
 *  TIM2.2: PA.1
 *  TIM2.3: PA.2
 *  TIM2.4: PA.3
 * 
 *  TIM3.1: PA.6
 *  TIM3.2: PA.7
 *  TIM3.3: PB.0
 *  TIM3.4: PB.1
 * 
 *  TIM4.1: PB.6
 *  TIM4.2: PB.7
 *  TIM4.3: PB.8
 *  TIM4.4: PB.9
 *
 *  Created on: 14 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef DEV_PWM_H_
#define DEV_PWM_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"

struct dev_pwm {
    TIM_TypeDef *   timer;
    uint16_t        freq;   /* PWM frequency in Hz */
    /* private parameters */
    uint32_t        period;
};

struct dev_pwm_channel {
    uint8_t         num;    /* PWM channel number 1-4 */
    uint8_t         status;
    uint16_t        value;

    /* private */
	GPIO_TypeDef *  port;   /* PWM output port */
    uint16_t        pin;    /* PWM output pin */
};


void pwm_init(struct dev_pwm * pwm);
void pwm_add_channel(struct dev_pwm_channel * channel);
//void pwm_get

#endif // DEV_PWM_H_