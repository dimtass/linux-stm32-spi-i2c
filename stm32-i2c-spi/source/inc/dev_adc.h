/*
 * dev_adc.h
 *
 * Copyright 2018 Dimitris Tassopoulos
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * This is a custom library for the ADCs of the stm32f103c8
 * 
 * Created on: 14 May 2018
 * Author: Dimitris Tassopoulos <dimtass@gmail.com>
 */

#ifndef DEV_ADC_H_
#define DEV_ADC_H_

#include <stdint.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

enum en_adc_dev {
    DEV_ADC1,
    DEV_ADC2,

    DEV_ADC_END
};

enum en_adc_channel_num {
	ADC_CH0 = 0,
	ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5,
	ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH_END
};

enum en_adc_mode {
	DEV_ADC_MODE_SINGLE,
	DEV_ADC_MODE_POLLING    // triggers a conversion as soon the previous is done
};

typedef int (*adc_cb_t)(struct adc_channel * adc, uint16_t value);

struct adc_input {
	GPIO_TypeDef    *port;
	uint16_t 		pin;
    uint32_t       	apb_periph;
};

struct adc_controller {
	uint8_t 		adc_num;
	ADC_TypeDef     *adc;
	uint32_t 		apb_periph;
	struct adc_input *input;
};

struct adc_noise_avg {
	uint32_t 		value;
	uint8_t			shift;
	uint8_t			shift_cntr;
};

struct adc_channel {
    uint16_t        ch_num;
    enum en_adc_mode mode;
	uint8_t 		ready;
	uint8_t			enable;
	volatile uint16_t value;
	uint8_t 		sample_time;
	adc_cb_t		adc_cb;	// callback

	ADC_InitTypeDef conf;
	struct adc_noise_avg avg;
    struct adc_controller *controller;
	struct list_head list;
};

int adc_init_channel(struct adc_channel * ch, enum en_adc_dev adc_device,
					enum en_adc_channel_num channel_num,
					enum en_adc_mode mode, adc_cb_t callback);
/**
 * This should only be a power of two, because it shifts the adc value
 */
int adc_avg_conf(uint8_t avg_shift);

/**
 * @param[in] sample_time	see ADC_SampleTime_* in stm32f10x_adc.h
 */
void adc_set_sample_speed(struct adc_channel * ch, uint8_t sample_time);

uint8_t adc_set_channel(struct adc_channel * dev, uint8_t ch);
uint8_t adc_enable_channel(struct adc_channel * dev, uint8_t enable);
uint8_t adc_disable_channel(struct adc_channel * dev);
void adc_reset_channel(struct adc_channel * ch);
// void adc_update(struct adc_channel * dev);


#endif /* DEV_ADC_H_ */
