/*
 * dev_i2c_slave.h
 *
 *  Created on: 10 Jan 2019
 *      Author: Dimitris Tassopoulos <dimtass@gmail.com>
 */

#ifndef DEV_I2C_SLAVE_H_
#define DEV_I2C_SLAVE_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

enum en_i2c_dev {
    DEV_I2C1 = 0,
    DEV_I2C2,

    DEV_EOL
};

enum i2c_slave_event {
    I2C_SLAVE_ADDRESSED,
	I2C_SLAVE_READ_REQUESTED,
	I2C_SLAVE_WRITE_REQUESTED,
	I2C_SLAVE_READ_PROCESSED,
	I2C_SLAVE_WRITE_RECEIVED,
	I2C_SLAVE_STOP,
};

struct i2c_client;

typedef uint8_t (*i2c_slave_cb_t)(struct i2c_client *, enum i2c_slave_event, uint8_t *);

struct i2c_adapter {
    I2C_TypeDef *       i2c;
    GPIO_TypeDef *      port;       /* SPI output port */
    uint16_t            pin_sda;    /* SPI SDA pin */
    uint16_t            pin_scl;    /* SPI SCL pin */
};

struct i2c_info {
    enum en_i2c_dev     dev;
    uint32_t            speed_hz;
    uint8_t             enabled;
    I2C_InitTypeDef     config;
    // uint8_t             * tx_buffer;
    // uint16_t            tx_buffer_len;
    // uint8_t             * rx_buffer;
    // uint16_t            rx_buffer_len;
};

struct i2c_client {
    uint16_t                address;
    struct i2c_info         info;
    struct i2c_adapter *    adapter;
    i2c_slave_cb_t          slave_cb;
	struct list_head list;
    // int                     irq;
};

int i2c_slave_init(enum en_i2c_dev dev, uint16_t address, uint32_t speed_hz,
                    i2c_slave_cb_t callback, struct i2c_client * i2c);
void i2c_enable(struct i2c_client * i2c);
void i2c_disable(struct i2c_client * i2c);
void i2c_send(struct i2c_client * i2c, const uint8_t * buffer, uint16_t buffer_size);
void i2c_receive(struct i2c_client * i2c, uint8_t * buffer, uint16_t recv_len);

#endif //DEV_I2C_SLAVE_H_