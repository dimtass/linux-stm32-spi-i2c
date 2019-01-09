 /*
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

#include "stm32f10x.h"
#include "dev_pwm.h"

enum en_pwm_channel_pin {
    PWM1_1,
    PWM1_2,
    PWM1_3,
    PWM1_4,
    PWM2_1,
    PWM2_2,
    PWM2_3,
    PWM2_4,
    PWM3_1,
    PWM3_2,
    PWM3_3,
    PWM3_4,
    PWM4_1,
    PWM4_2,
    PWM4_3,
    PWM4_4,
}

struct tp_pwm_pins {
    enum en_pwm_channel_pin pin;
	GPIO_TypeDef *  port;   /* PWM output port */
    uint16_t        pin;    /* PWM output pin */
};

struct tp_pwm_pins pwm_pins = {
    {PWM1_1, GPIOA, GPIO_Pin_8},
    {PWM1_2, GPIOA, GPIO_Pin_9},
    {PWM1_3, GPIOA, GPIO_Pin_10},
    {PWM1_4, GPIOA, GPIO_Pin_11},

    {PWM2_1, GPIOA, GPIO_Pin_0},
    {PWM2_2, GPIOA, GPIO_Pin_1},
    {PWM2_3, GPIOA, GPIO_Pin_2},
    {PWM2_4, GPIOA, GPIO_Pin_3},

    {PWM3_1, GPIOA, GPIO_Pin_6},
    {PWM3_2, GPIOA, GPIO_Pin_7},
    {PWM3_3, GPIOB, GPIO_Pin_0},
    {PWM3_4, GPIOB, GPIO_Pin_1},

    {PWM4_1, GPIOB, GPIO_Pin_6},
    {PWM4_2, GPIOB, GPIO_Pin_7},
    {PWM4_3, GPIOB, GPIO_Pin_8},
    {PWM4_4, GPIOB, GPIO_Pin_9},
}

void pwm_init(struct dev_pwm * pwm)
{
    uint32_t rcc = 0;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    pwm->period = (SystemCoreClock / pwm->freq ) - 1;

    if (pwm->timer == TIM1)
        rcc = RCC_APB1Periph_TIM1;
    else if (pwm->timer == TIM2)
        rcc = RCC_APB1Periph_TIM2;
    else if (pwm->timer == TIM3)
        rcc = RCC_APB1Periph_TIM3;

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = pwm->period;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(pwm->timer, &TIM_TimeBaseStructure);
    /* TIMx counter enable */
    TIM_Cmd(pwm->timer, ENABLE);

    /* TIMx Main Output Enable */
    TIM_CtrlPWMOutputs(pwm->timer, ENABLE);
}

void pwm_apply_channel_settings(struct dev_pwm * pwm, uint8_t channel,  TIM_OCInitTypeDef * TIM_OCInitStructure)
{
    if (channel == 0)
        TIM_OC1Init(pwm->timer, TIM_OCInitStructure);
    else if (channel == 1)
        TIM_OC2Init(pwm->timer, TIM_OCInitStructure);
    else if (channel == 2)
        TIM_OC3Init(pwm->timer, TIM_OCInitStructure);
    else if (channel == 3)
        TIM_OC4Init(pwm->timer, TIM_OCInitStructure);
}

void pwm_init_channel(struct dev_pwm * pwm, uint8_t channel)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    pwm_apply_channel_settings(&TIM_OCInitStructure);
}

void pwm_add_channel(struct dev_pwm_channel * channel)
{
    uint32_t rcc = 0;

    /* GPIO clock enable */
    if (pwm->port == GPIOA)
        rcc = RCC_APB2Periph_GPIOA;
    else if (pwm->port == GPIOB)
        rcc = RCC_APB2Periph_GPIOB;
    else if (pwm->port == GPIOC)
        rcc = RCC_APB2Periph_GPIOC;
    else if (pwm->port == GPIOD)
        rcc = RCC_APB2Periph_GPIOD;

    RCC_APB2PeriphClockCmd(rcc | RCC_APB2Periph_AFIO, ENABLE);

    /* Setup GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pwm->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(pwm->port, &GPIO_InitStructure);
}

void pwm_set_duty_cycle()
{
    /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
    TimerPeriod = (SystemCoreClock / 17570 ) - 1;
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
}