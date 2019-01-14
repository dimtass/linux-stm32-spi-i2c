/*
 * dev_adc.c
 *
 *  Created on: 10 May 2018
 *      Author: dimtass
 */

#include "dev_adc.h"

struct adc_input m_adc_input[] = {
	[0] = {GPIOA, GPIO_Pin_0},
	[1] = {GPIOA, GPIO_Pin_1},
	[2] = {GPIOA, GPIO_Pin_2},
	[3] = {GPIOA, GPIO_Pin_3},
	[4] = {GPIOA, GPIO_Pin_4},
	[5] = {GPIOA, GPIO_Pin_5},
	[6] = {GPIOA, GPIO_Pin_6},
	[7] = {GPIOA, GPIO_Pin_7},
	[8] = {GPIOB, GPIO_Pin_0},
	[9] = {GPIOB, GPIO_Pin_1},
};

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC2_DR_Address    ((uint32_t)0x4001284C)
//#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

#define ADC_EXISTS(ADC, ITTERATOR) (ADC->channel == ITTERATOR)

static struct dev_adc_module * dev_adc1 = NULL;
static struct dev_adc_module * dev_adc2 = NULL;

/* Prototypes */
static inline struct dev_adc * dev_adc_get_next_channel(struct dev_adc_module * dev);

void dev_adc_module_init(struct dev_adc_module * dev)
{
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* initialize list for ADC channels */
	INIT_LIST_HEAD(&dev->adc_ch_list);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	/* Enable ADC1 and GPIOA clock */
	if (dev->adc == ADC1) {
		dev_adc1 = dev;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if (dev->adc == ADC2) {
		dev_adc2 = dev;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	}

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(dev->adc, &ADC_InitStructure);

	/* Enable ADC1 EOC interrupt */
	ADC_ITConfig(dev->adc, ADC_IT_EOC, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(dev->adc, ENABLE);
	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(dev->adc);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(dev->adc));
	/* Start ADC1 calibration */
	ADC_StartCalibration(dev->adc);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(dev->adc));

	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // this sets the priority group of the interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff
}


void dev_adc_start(struct dev_adc_module * dev)
{
	if (!dev->curr_channel) dev->curr_channel = dev_adc_get_next_channel(dev);
	TRACEL(TRACE_LEVEL_ADC, ("ADC:start:%d\n", dev->curr_channel->channel));
	if (dev->curr_channel) {
		/* Enable ADC2 EOC interrupt */
		ADC_ITConfig(dev->adc, ADC_IT_EOC, ENABLE);
		/* Start ADC1 Software Conversion */
		ADC_SoftwareStartConvCmd(dev->adc, ENABLE);
	}
}


void dev_adc_stop(struct dev_adc_module * dev)
{
	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(dev->adc, DISABLE);
	/* Enable ADC2 EOC interrupt */
	ADC_ITConfig(dev->adc, ADC_IT_EOC, DISABLE);
}


static inline struct dev_adc * dev_adc_find_channel(struct dev_adc_module * dev, uint8_t ch)
{
	if (!list_empty(&dev->adc_ch_list)) {
		struct dev_adc * ch_it = NULL;
		list_for_each_entry(ch_it, &dev->adc_ch_list, list) {
			if ADC_EXISTS(ch_it, ch) {
				/* found */
				return(ch_it);
			}
		}
	}
	return NULL;
}


void* dev_adc_probe(struct dev_adc * ch)
{
	/* Only add channel if not already found */
	if (!ch->owner || !ch) return NULL;
	struct dev_adc_module * dev = ch->owner;
	if (dev_adc_find_channel(dev, ch->channel)) return NULL;

	struct dev_adc * new_adc = (struct dev_adc *) malloc(sizeof(struct dev_adc));
	memcpy(new_adc, ch, sizeof(struct dev_adc));

	/* init dev head list */
	INIT_LIST_HEAD(&new_adc->list);
	/* Add to led_list */
	list_add(&new_adc->list, &dev->adc_ch_list);
	TRACEL(TRACE_LEVEL_ADC, ("ADC:add->%d\n", new_adc->channel));
	return new_adc;
}


void dev_adc_remove(struct dev_adc * ch)
{
	if (!ch->owner || !ch) return;
	struct dev_adc * found_ch = dev_adc_find_channel(ch->owner, ch->channel);
	if (found_ch) {
		/* remove */
		list_del(&found_ch->list);
		free(found_ch);
		TRACEL(TRACE_LEVEL_ADC, ("ADC:delete->%d\n", ch->channel));
	}
}


uint8_t dev_adc_set_channel(struct dev_adc_module * dev, uint8_t ch)
{
	struct dev_adc * found_ch = dev_adc_find_channel(dev, ch);
	if (found_ch) {
		/* ADC1 regular channel0 configuration */
		ADC_RegularChannelConfig(dev->adc, found_ch->channel, 1, ADC_SampleTime_28Cycles5);

		/* Enable ADC2 EOC interrupt */
		ADC_ITConfig(dev->adc, ADC_IT_EOC, ENABLE);
	}
	return(!!found_ch);
}


uint8_t dev_adc_enable_channel(struct dev_adc_module * dev, uint8_t ch, uint8_t enable)
{
	struct dev_adc * found_ch = dev_adc_find_channel(dev, ch);
	if (found_ch) {
		found_ch->enable = enable;
	}
	return(!!found_ch);
}

static inline struct dev_adc * dev_adc_get_next_channel(struct dev_adc_module * dev)
{
	if (!list_empty(&dev->adc_ch_list)) {
		struct dev_adc * ch;
		list_for_each_entry(ch, &dev->adc_ch_list, list) {
			if (ch->enable && !ch->ready) {
				TRACEL(TRACE_LEVEL_ADC, ("ADC:next->%d\n", ch->channel));
				return ch;
			}
		}
	}
	return NULL;
}


uint8_t dev_adc_reset_channels(struct dev_adc_module * dev)
{
	uint8_t num_of_enabled_channels = 0;
	if (!list_empty(&dev->adc_ch_list)) {
		struct dev_adc * ch;
		list_for_each_entry(ch, &dev->adc_ch_list, list) {
			if (ch->enable) {
				ch->ready = 0;
				num_of_enabled_channels++;
			}
		}
	}
	return(num_of_enabled_channels);
}


void dev_adc_update(struct dev_adc_module * dev)
{
	if (dev->mode == DEV_ADC_MODE_POLLING) {
		if (dev_adc_reset_channels(dev)) {
			dev_adc_start(dev);
		}
	}
}


void ADC1_2_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
		/* save value */
		dev_adc_stop(dev_adc1);
		dev_adc1->curr_channel->value = ADC_GetConversionValue(ADC1);
		TRACEL(TRACE_LEVEL_ADC, ("ADC1:value[%d]->%d\n", dev_adc1->curr_channel->channel, dev_adc1->curr_channel->value));
		dev_adc1->curr_channel->ready = 1;
		dev_adc1->curr_channel = dev_adc_get_next_channel(dev_adc1);
		if (dev_adc1->curr_channel) {
			dev_adc_set_channel(dev_adc1, dev_adc1->curr_channel->channel);
			dev_adc_start(dev_adc1);
		}
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
	else if (ADC_GetITStatus(ADC2, ADC_IT_EOC) != RESET) {
		/* save value */
		dev_adc_stop(dev_adc2);
		dev_adc2->curr_channel->value = ADC_GetConversionValue(ADC2);
		TRACEL(TRACE_LEVEL_ADC, ("ADC2:value[%d]->%d\n", dev_adc2->curr_channel->channel, dev_adc2->curr_channel->value));
		dev_adc2->curr_channel->ready = 1;
		dev_adc2->curr_channel = dev_adc_get_next_channel(dev_adc2);
		if (dev_adc2->curr_channel) {
			dev_adc_set_channel(dev_adc2, dev_adc2->curr_channel->channel);
			dev_adc_start(dev_adc2);
		}
		ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
	}
}
