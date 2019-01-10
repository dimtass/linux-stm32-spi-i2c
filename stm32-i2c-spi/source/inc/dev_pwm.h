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
#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "platform_config.h"

enum en_pwm_ch_status {
    PWM_CH_STATUS_IDLE = 0,
    PWM_CH_STATUS_RUNNING
};

enum en_pwm_channel_pin {
    PWM1_1 = 0, //PA.8
    PWM1_2,     //PA.9
    PWM1_3,     //PA.10
    PWM1_4,     //PA.11
    PWM2_1,     //PA.0
    PWM2_2,     //PA.1
    PWM2_3,     //PA.2
    PWM2_4,     //PA.3
    PWM3_1,     //PA.6
    PWM3_2,     //PA.7
    PWM3_3,     //PB.0
    PWM3_4,     //PB.1
    PWM4_1,     //PB.6
    PWM4_2,     //PB.7
    PWM4_3,     //PB.8
    PWM4_4,     //PB.9
};

struct tp_pwm_out {
    uint16_t                num;
	GPIO_TypeDef *          port;   /* PWM output port */
    uint16_t                pin;    /* PWM output pin */
};

struct dev_pwm {
    TIM_TypeDef *           timer;
    uint16_t                freq;   /* PWM frequency in Hz */
    /* private parameters */
    uint32_t                period;
    TIM_TimeBaseInitTypeDef config;
};

struct dev_pwm_channel {
    enum en_pwm_channel_pin chan;    /* PWM channel number 1-4 */
    uint16_t                value;
    enum en_pwm_ch_status   status;

    /* private do not fill these */
    TIM_OCInitTypeDef       oc;
	struct tp_pwm_out *     output;
    struct dev_pwm *        parent;
};

void pwm_add_channel(enum en_pwm_channel_pin pwm_channel, uint16_t freq, struct dev_pwm_channel * channel);
void pwm_set_duty_cycle(struct dev_pwm_channel * channel, uint16_t value);
//void pwm_get

#endif // DEV_PWM_H_