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

enum en_pwm_channel_id {
    PWM1_1 = 0, //TIMER 1, PA.8
    PWM1_2,     //TIMER 1, PA.9
    PWM1_3,     //TIMER 1, PA.10
    PWM1_4,     //TIMER 1, PA.11
    PWM2_1,     //TIMER 2, PA.0
    PWM2_2,     //TIMER 2, PA.1
    PWM2_3,     //TIMER 2, PA.2
    PWM2_4,     //TIMER 2, PA.3
    PWM3_1,     //TIMER 3, PA.6
    PWM3_2,     //TIMER 3, PA.7
    PWM3_3,     //TIMER 3, PB.0
    PWM3_4,     //TIMER 3, PB.1
    PWM4_1,     //TIMER 4, PB.6
    PWM4_2,     //TIMER 4, PB.7
    PWM4_3,     //TIMER 4, PB.8
    PWM4_4,     //TIMER 4, PB.9

    PWM_EOL,
};

/**
 * @brief enum pwm_polarity - polarity of a PWM signal
 * @PWM_POLARITY_NORMAL: a high signal for the duration of the duty-
 * cycle, followed by a low signal for the remainder of the pulse
 * period
 * @PWM_POLARITY_INVERSED: a low signal for the duration of the duty-
 * cycle, followed by a high signal for the remainder of the pulse
 * period
 */
enum en_pwm_polarity {
    PWM_POLARITY_NORMAL = 0,
    PWM_POLARITY_INVERSED
};

struct pwm_chip {
    uint8_t                 num;    /* channel number 1-4 */
    TIM_TypeDef *           timer;
	GPIO_TypeDef *          port;   /* PWM output port */
    uint16_t                pin;    /* PWM output pin */
};

struct pwm_config {
    enum en_pwm_channel_id  id;     /* PWM id */
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef       oc;
};

struct pwm_state {
    uint32_t                period;
    uint32_t                freq;
    float                   duty_cycle; /* 0.00 - 100.00 */
    enum en_pwm_polarity    polarity;
    uint8_t                 enabled;
};


struct pwm_device {
    struct pwm_state        state;
    struct pwm_config       config;
	struct pwm_chip *       chip;
};

int pwm_add(enum en_pwm_channel_id pwm_channel, struct pwm_device * pwm, uint32_t freq);
void pwm_config(struct pwm_device * pwm, uint32_t freq);
void pwm_set_polarity(struct pwm_device * pwm, enum en_pwm_polarity polarity);
void pwm_enable(struct pwm_device * pwm);
void pwm_disable(struct pwm_device * pwm);
void pwm_set_duty_cycle(struct pwm_device * pwm, float duty_cycle);


#endif // DEV_PWM_H_