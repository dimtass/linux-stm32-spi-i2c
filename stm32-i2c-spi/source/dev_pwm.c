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
 *      Author: Dimitris Tassopoulos <dimtass@gmail.com>
 */
#include "stm32f10x.h"
#include "dev_pwm.h"

#define DEBUG(X) TRACEL(TRACE_LEVEL_PWM, X)

struct pwm_hw m_pwm_hw[] = {
    /* TIM1 */
    [PWM1_1] = {1, GPIOA, GPIO_Pin_8},
    [PWM1_2] = {2, GPIOA, GPIO_Pin_9},
    [PWM1_3] = {3, GPIOA, GPIO_Pin_10},
    [PWM1_4] = {4, GPIOA, GPIO_Pin_11},
    /* TIM2 */
    [PWM2_1] = {1, GPIOA, GPIO_Pin_0},
    [PWM2_2] = {2, GPIOA, GPIO_Pin_1},
    [PWM2_3] = {3, GPIOA, GPIO_Pin_2},
    [PWM2_4] = {4, GPIOA, GPIO_Pin_3},
    /* TIM3 */
    [PWM3_1] = {1, GPIOA, GPIO_Pin_6},
    [PWM3_2] = {2, GPIOA, GPIO_Pin_7},
    [PWM3_3] = {3, GPIOB, GPIO_Pin_0},
    [PWM3_4] = {4, GPIOB, GPIO_Pin_1},
    /* TIM4 */
    [PWM4_1] = {1, GPIOB, GPIO_Pin_6},
    [PWM4_2] = {2, GPIOB, GPIO_Pin_7},
    [PWM4_3] = {3, GPIOB, GPIO_Pin_8},
    [PWM4_4] = {4, GPIOB, GPIO_Pin_9},
};

static struct dev_pwm m_pwm[4] = {
    {.timer = TIM1,},
    {.timer = TIM2,},
    {.timer = TIM3,},
    {.timer = TIM4,},
};

int pwm_init(struct dev_pwm * pwm, uint16_t freq)
{
    if (!pwm)
        return -1;
    
    pwm->freq = freq;
    pwm->period = (SystemCoreClock / pwm->freq) - 1;

    if (pwm->timer == TIM1) {
        DEBUG(("Using TIM1\n"));
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    }
    else if (pwm->timer == TIM2) {
        DEBUG(("Using TIM2\n"));
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }
    else if (pwm->timer == TIM3) {
        DEBUG(("Using TIM3\n"));
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }
    else if (pwm->timer == TIM4) {
        DEBUG(("Using TIM4\n"));
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }

    /* Time base configuration */
    TIM_TimeBaseStructInit(&pwm->config);
    pwm->config.TIM_Period = pwm->period;
    pwm->config.TIM_CounterMode = TIM_CounterMode_Up;
    pwm->config.TIM_Prescaler = 0;
    pwm->config.TIM_ClockDivision = TIM_CKD_DIV1;
    pwm->config.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(pwm->timer, &pwm->config);

    DEBUG((
        "pwm_init:\n"
        "  timer: %08X\n"
        "  sys: %lu\n"
        "  period: %lu\n"
        "  freq: %d\n",
        (uint16_t)((uint32_t)pwm->timer),
        SystemCoreClock,
        pwm->period,
        pwm->freq
    ));

    return 0;
}

void pwm_update_channel(struct dev_pwm_channel * channel)
{
    if (channel->output->num == 1)
        TIM_OC1Init(channel->parent->timer, &channel->oc);
    else if (channel->output->num == 2)
        TIM_OC2Init(channel->parent->timer, &channel->oc);
    else if (channel->output->num == 3)
        TIM_OC3Init(channel->parent->timer, &channel->oc);
    else if (channel->output->num == 4)
        TIM_OC4Init(channel->parent->timer, &channel->oc);
    
    DEBUG(("Setting "
        "  channel: %d\n"
        "  pin: %d\n"
        "  pulse: %d\n",
        channel->output->num,
        channel->output->pin,
        channel->oc.TIM_Pulse
        ));
}

void pwm_init_channel(struct dev_pwm_channel * channel)
{
    TIM_OCInitTypeDef * TIM_OCInitStructure = &channel->oc;

    TIM_OCStructInit(TIM_OCInitStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure->TIM_Pulse = 0;
    TIM_OCInitStructure->TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure->TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure->TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure->TIM_OCIdleState = TIM_OCIdleState_Set;

    pwm_update_channel(channel);
}

void pwm_add_channel(enum en_pwm_channel_pin pwm_channel, uint16_t freq, struct dev_pwm_channel * channel)
{
    uint32_t rcc = 0;

    DEBUG(("Timer index: %d\n", pwm_channel / 4));
    /* set up PWM */
    channel->parent = &m_pwm[pwm_channel / 4];
    pwm_init(channel->parent, freq);

    /* set up channel */
    channel->value = 0;
    pwm_init_channel(channel);
    channel->output = &m_pwm_out[pwm_channel];
    channel->status = PWM_CH_STATUS_IDLE;

    /* GPIO clock enable */
    if (channel->output->port == GPIOA)
        rcc = RCC_APB2Periph_GPIOA;
    else if (channel->output->port == GPIOB)
        rcc = RCC_APB2Periph_GPIOB;
    // else if (channel->output->port == GPIOC)
    //     rcc = RCC_APB2Periph_GPIOC;
    // else if (channel->output->port == GPIOD)
    //     rcc = RCC_APB2Periph_GPIOD;

    RCC_APB2PeriphClockCmd(rcc | RCC_APB2Periph_AFIO, ENABLE);

    /* Setup GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = channel->output->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(channel->output->port, &GPIO_InitStructure);

    /* TIM1 counter enable */
    TIM_Cmd(channel->parent->timer, ENABLE);

    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(channel->parent->timer, ENABLE);
}

void pwm_set_duty_cycle(struct dev_pwm_channel * channel, uint16_t value)
{
    struct dev_pwm * pwm = channel->parent;
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    channel->value = value;
    channel->oc.TIM_Pulse = (uint16_t) (((uint32_t) channel->value * (pwm->config.TIM_Period - 1)) / 100);
    channel->status = (value) ? PWM_CH_STATUS_RUNNING : PWM_CH_STATUS_IDLE;
    pwm_update_channel(channel);
    DEBUG(("PWM set: %d [%d/%d]\n", value, channel->oc.TIM_Pulse, pwm->config.TIM_Period));
}