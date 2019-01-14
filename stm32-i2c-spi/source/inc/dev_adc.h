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

#include <stdint.h>
#include "stm32f10x.h"
#include "platform_config.h"

enum en_adc_dev {
    DEV_ADC1,
    DEV_ADC2,

    DEV_ADC_END
};

enum en_adc_mode {
	DEV_ADC_MODE_SINGLE,
	DEV_ADC_MODE_POLLING    // creates IRQ for the host
};

typedef int (adc_cb_t)(struct adc_device * adc, uint16_t value);

static struct adc_controller m_adc_controller[] = {
    [DEV_ADC1] = {ADC1, 0x4001244C, RCC_APB2Periph_ADC1, },
    [DEV_ADC2] = {},
};

struct adc_controller {
	ADC_TypeDef     *adc;
    uint32_t        address;
    uint32_t        apb_periph;
	GPIO_TypeDef    *port;
};

struct adc_dma {
    DMA_Channel_TypeDef     *channel;
    uint32_t                *mem_address;
    DMA_InitTypeDef         conf;
};

struct adc_device {
    uint16_t        ch;
    uint8_t         enable;
    enum en_adc_mode mode;
    uint8_t         dma_enable;
	ADC_InitTypeDef conf;
    struct adc_dma  *dma;
    struct adc_controller *controller;
};



#ifndef DEV_ADC_H_
#define DEV_ADC_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

enum en_adc_mode {
	DEV_ADC_MODE_SINGLE,
	DEV_ADC_MODE_POLLING
};

#define DECLARE_ADC_CH(OWNER, CHANNEL, ENABLE) { OWNER, CHANNEL, ENABLE, 0, 0 }

#define DECLARE_DEV_ADC(OWNER, NAME, CHANNEL, ENABLE) \
	struct dev_adc NAME = { \
		.owner = OWNER, \
		.channel = CHANNEL, \
		.enable = ENABLE, \
		.ready = 0, \
		.value = 0, \
	}

struct dev_adc {
	struct dev_adc_module * owner;
	uint8_t	channel;
	uint8_t enable;
	uint8_t ready;
	volatile uint16_t value;
	struct list_head list;
};


#define DECLARE_MODULE_ADC(NAME, ADC, MODE, TICK_MS) \
	struct dev_adc_module NAME = { \
		.adc = ADC, \
		.mode = MODE, \
		.tick_ms = TICK_MS, \
		.curr_channel = NULL, \
	}

struct dev_adc_module {
	ADC_TypeDef * adc;
	enum en_adc_mode	mode;
	uint16_t	tick_ms;
	struct dev_adc * curr_channel;
	struct list_head adc_ch_list;
};

void dev_adc_module_init(struct dev_adc_module * adc);
void dev_adc_start(struct dev_adc_module * dev);
void* dev_adc_probe(struct dev_adc * adc_dev_arr);
void dev_adc_remove(struct dev_adc * adc_dev_arr);
uint8_t dev_adc_set_channel(struct dev_adc_module * dev, uint8_t ch);
uint8_t dev_adc_enable_channel(struct dev_adc_module * dev, uint8_t ch, uint8_t enable);
uint8_t dev_adc_reset_channels(struct dev_adc_module * dev);
void dev_adc_update(struct dev_adc_module * dev);


#endif /* DEV_ADC_H_ */
