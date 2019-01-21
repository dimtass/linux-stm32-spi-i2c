/**
 * dev_spi.h
 *
 * Copyright 2018 Dimitris Tassopoulos <dimtass@gmail.com>
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
 */


#ifndef DEV_SPI_SLAVE_H_
#define DEV_SPI_SLAVE_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "dev_spi_common.h"

#define DECLARE_SPI_SLAVE_CHANNEL(NAME, OWNER, CHANNEL) \
	struct spi_slave_device NAME = { \
		.channel = CHANNEL, \
	}

#define DECLARE_SPI_BUFFER(NAME,TYPE,BUFFER_SIZE) \
    TYPE NAME##_rx_buffer[BUFFER_SIZE]; \
    TYPE NAME##_tx_buffer[BUFFER_SIZE]; \
    struct spi_buffers NAME = { \
        .rx_buffer = NAME##_rx_buffer, \
        .rx_buffer_len = BUFFER_SIZE, \
        .tx_buffer = NAME##_tx_buffer, \
        .tx_buffer_len = BUFFER_SIZE, \
    }



void spi_init_slave(enum en_spi_port port,
                    struct spi_buffers * buffers,
                    receive_irq_t callback,
                    struct spi_device * spi);

#endif //DEV_SPI_SLAVE_H_