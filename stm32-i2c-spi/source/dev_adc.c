/*
 * dev_adc.c
 *
 *  Created on: 10 May 2018
 *      Author: dimtass
 */

#include "stm32f10x.h"
#include "dev_adc.h"

LIST_HEAD(adc_channel_list);

static struct adc_controller m_adc_controller[] = {
    [DEV_ADC1] = {DEV_ADC1, ADC1, RCC_APB2Periph_ADC1, NULL},
    [DEV_ADC2] = {DEV_ADC2, ADC2, RCC_APB2Periph_ADC2, NULL},
};

struct adc_input m_adc_input[] = {
	[ADC_CH0] = {GPIOA, GPIO_Pin_0, RCC_APB2Periph_GPIOA},
	[ADC_CH1] = {GPIOA, GPIO_Pin_1, RCC_APB2Periph_GPIOA},
	[ADC_CH2] = {GPIOA, GPIO_Pin_2, RCC_APB2Periph_GPIOA},
	[ADC_CH3] = {GPIOA, GPIO_Pin_3, RCC_APB2Periph_GPIOA},
	[ADC_CH4] = {GPIOA, GPIO_Pin_4, RCC_APB2Periph_GPIOA},
	[ADC_CH5] = {GPIOA, GPIO_Pin_5, RCC_APB2Periph_GPIOA},
	[ADC_CH6] = {GPIOA, GPIO_Pin_6, RCC_APB2Periph_GPIOA},
	[ADC_CH7] = {GPIOA, GPIO_Pin_7, RCC_APB2Periph_GPIOA},
	[ADC_CH8] = {GPIOB, GPIO_Pin_0, RCC_APB2Periph_GPIOB},
	[ADC_CH9] = {GPIOB, GPIO_Pin_1, RCC_APB2Periph_GPIOB},
};

static uint8_t adc1_curr_channel = 0;
static uint8_t adc2_curr_channel = 0;

#define ADC_EXISTS(ADC, ITTERATOR) (ADC->channel == ITTERATOR)

int adc_init_channel(struct adc_channel * ch, enum en_adc_dev adc_device,
					enum en_adc_channel_num channel_num,
					enum en_adc_mode mode, adc_cb_t callback)
{
	int ret = 0;

	/* Handle errors */
	if (!ch)
		return -1;
	if (adc_device >= DEV_ADC_END)
		return -2;
	if (channel_num >= ADC_CH_END)
		return -3;

	if (SystemCoreClock > 72000000)
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	else
		RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	list_add(&ch->list, &adc_channel_list);

	/* copy parameters */
	ch->ch_num = channel_num;
	ch->mode = mode;
	ch->adc_cb = callback;
	ch->ready = 0;

	ch->controller = &m_adc_controller[adc_device];
	ch->controller->adc_num = adc_device;
	ch->controller->input = &m_adc_input[channel_num];

	/* Enable ADCx and GPIOx clock */
	uint32_t rcc = ch->controller->apb_periph | ch->controller->input->apb_periph;
	RCC_APB2PeriphClockCmd(rcc, ENABLE);

	/* ADC configuration */
	ch->conf.ADC_Mode = ADC_Mode_Independent;
	ch->conf.ADC_ScanConvMode = DISABLE;
	ch->conf.ADC_ContinuousConvMode = ENABLE;
	ch->conf.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ch->conf.ADC_DataAlign = ADC_DataAlign_Right;
	ch->conf.ADC_NbrOfChannel = 1;
	ADC_Init(ch->controller->adc, &ch->conf);

	/* Enable ADCx EOC interrupt */
	ADC_ITConfig(ch->controller->adc, ADC_IT_EOC, ENABLE);

	/* Enable ADCx */
	ADC_Cmd(ch->controller->adc, ENABLE);
	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ch->controller->adc);
	/* Check the end of ADCx reset calibration register */
	while(ADC_GetResetCalibrationStatus(ch->controller->adc));
	/* Start ADCx calibration */
	ADC_StartCalibration(ch->controller->adc);
	/* Check the end of ADCx calibration */
	while(ADC_GetCalibrationStatus(ch->controller->adc));
	/* Default sample time 13.5 cycles */
	ADC_RegularChannelConfig(ch->controller->adc, ch->ch_num, 1, ADC_SampleTime_13Cycles5);

	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // this sets the priority group of the interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	return ret;
}

int adc_start(struct adc_channel * ch)
{
	if (!ch)
		return -1;

	TRACEL(TRACE_LEVEL_ADC, ("ADC%d.%d start\n", ch->controller->adc_num, ch->ch_num));
	/* Set channel */
	ADC_RegularChannelConfig(ch->controller->adc, ch->ch_num, 1, ch->sample_time);
	/* Enable ADCx EOC interrupt */
	ADC_ITConfig(ch->controller->adc, ADC_IT_EOC, ENABLE);
	/* Start ADCx Software Conversion */
	ADC_SoftwareStartConvCmd(ch->controller->adc, ENABLE);

	ch->enable = 1;
	ch->avg.value = 0;
	ch->avg.shift_cntr = 0;

	return 0;
}

void adc_stop(struct adc_channel * ch)
{
	ch->avg.value = 0;
	ch->avg.shift_cntr = 0;
}

void adc_set_sample_speed(struct adc_channel * ch, uint8_t sample_time)
{
	ch->sample_time = sample_time;
}

void adc_enable_channel(struct adc_channel * ch, uint8_t enable)
{
	ch->enable = enable;
}

static inline struct adc_channel * adc_find(ADC_TypeDef * adc, uint8_t channel)
{
	if (!list_empty(&adc_channel_list)) {
		struct adc_channel * ch_it;
		list_for_each_entry(ch_it, &adc_channel_list, list) {
			if ((ch_it->controller->adc == adc) && (ch_it->ch_num == channel)) {
				return ch_it;
			}
		}
	}
	return NULL;
}

static inline struct adc_channel * adc_get_next_channel(ADC_TypeDef * adc)
{
	if (!list_empty(&adc_channel_list)) {
		struct adc_channel * ch_it;
		list_for_each_entry(ch_it, &adc_channel_list, list) {
			if ((ch_it->controller->adc == adc) && ch_it->enable && !ch_it->ready) {
				TRACEL(TRACE_LEVEL_ADC, ("ADC:next->%d\n", ch_it->ch_num));
				return ch_it;
			}
		}
	}
	return NULL;
}

void dev_adc_reset_channel(struct adc_channel * ch)
{
	ch->ready = 0;
	ch->value = 0;
	if (ch->avg.shift) {
		ch->avg.value = 0;
		ch->avg.shift_cntr = 0;
	}
}

void ADC1_2_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
		/* Stop ADCx Software Conversion */
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
		/* Disable ADCx EOC interrupt */
		ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
		/* Retrieve channel */
		struct adc_channel * ch = adc_find(ADC1, adc1_curr_channel);
		if (ch) {
			/* save value */
			if (ch->avg.shift) {

			}
			else {
				ch->value = ADC_GetConversionValue(ADC1);
				ch->ready = 1;
			}
			TRACEL(TRACE_LEVEL_ADC, ("ADC1:value[%d]->%d\n", ch->ch_num, ch->value));
	
			ch = adc_get_next_channel(ADC1);
			if (ch) {
				adc_start(ch);
			}
		}
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
	else if (ADC_GetITStatus(ADC2, ADC_IT_EOC) != RESET) {
		/* Stop ADCx Software Conversion */
		ADC_SoftwareStartConvCmd(ADC2, DISABLE);
		/* Disable ADCx EOC interrupt */
		ADC_ITConfig(ADC2, ADC_IT_EOC, DISABLE);
		/* Retrieve channel */
		struct adc_channel * ch = adc_find(ADC2, adc2_curr_channel);
		if (ch) {
			/* save value */
			if (ch->avg.shift) {

			}
			else {
				ch->value = ADC_GetConversionValue(ADC2);
				ch->ready = 1;
			}
			TRACEL(TRACE_LEVEL_ADC, ("ADC2:value[%d]->%d\n", ch->ch_num, ch->value));
	
			ch = adc_get_next_channel(ADC2);
			if (ch) {
				adc_start(ch);
			}
		}
		ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
	}
}
