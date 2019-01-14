/**
 * dev_spi.h
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
 * Created on: 14 May 2018
 * Author: Dimitris Tassopoulos <dimtass@gmail.com>
 */

#ifndef DEV_SPI_H_
#define DEV_SPI_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"

enum en_spi_port {
	DEV_SPI1_GPIOA,
	DEV_SPI1_GPIOB,
	DEV_SPI2
};

#define DECLARE_SPI_CHANNEL(NAME, OWNER, CHANNEL) \
	struct spi_device NAME = { \
		.channel = CHANNEL, \
	}


struct dma_channel {
	DMA_Channel_TypeDef * tx_ch;
	uint32_t		tx_flags;
	uint32_t		tx_iqrn;
	DMA_Channel_TypeDef * rx_ch;
	uint32_t		rx_flags;
	uint32_t		rx_iqrn;
	DMA_InitTypeDef config;
};


struct spi_controller {
	SPI_TypeDef * 	spi;
	GPIO_TypeDef *  port;
	uint16_t		miso;
	uint16_t		mosi;
	uint16_t		sck;
	uint16_t		nss;
	SPI_InitTypeDef config;
	struct dma_channel * dma;
};


struct spi_device {
	struct spi_controller * master;
	struct spi_controller * controller;
	uint16_t	chip_select;
	uint8_t 	bits_per_word;	// Use only 8 or 16 (default: 8)
	uint16_t 	speed;			// Actually this is the prescaller (2, 4, 8, 16, 32, 64, 128, 256)
	uint8_t		mode;			// See below:
#define SPI_MODE_CPHA        0x01
#define SPI_MODE_CPOL        0x02
#define SPI_MODE_0      (0|0)
#define SPI_MODE_1      (0|SPI_MODE_CPHA)
#define SPI_MODE_2      (SPI_MODE_CPOL|0)
#define SPI_MODE_3      (SPI_MODE_CPOL|SPI_MODE_CPHA)

	void (*receive_irq)(uint8_t * buffer, uint16_t buffer_length);
};


/* Init functions */
void* spi_init_master(enum en_spi_port port, struct spi_device * spi);
void spi_remove(struct spi_device * spi);

/* Control functions */
void spi_start(struct spi_device * spi);
void spi_stop(struct spi_device * spi);
void spi_wait(struct spi_device * spi);
void spi_set8(struct spi_device * spi);
void spi_set16(struct spi_device * spi);

/* 8-bit send/receive functions */
void spi_send8(struct spi_device * spi, uint8_t * data, size_t data_len);
void spi_sendCircular8(struct spi_device * spi, uint8_t * data, size_t data_len);
void spi_recv8(struct spi_device * spi, uint8_t * data, size_t data_len);
void spi_recvCircular8(struct spi_device * spi, uint8_t * data, size_t data_len);

/* 16-bit functions */
void spi_send16(struct spi_device * spi, uint16_t *data, size_t data_len);
void spi_sendCircular16(struct spi_device * spi, uint16_t *data, size_t data_len);
void spi_recv16(struct spi_device * spi, uint16_t * data, size_t data_len);
void spi_recvCircular16(struct spi_device * spi, uint8_t * data, size_t data_len);

void spi_get_available_freq(uint8_t * num_of_freq, uint32_t ** freq_array);
#endif /* DEV_SPI_H_ */
