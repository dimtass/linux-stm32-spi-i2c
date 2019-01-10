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

enum en_i2c_dev {
    DEV_I2C1 = 0,
    DEV_I2C2
};

struct dev_i2c_out {
    I2C_TypeDef *       i2c;
    GPIO_TypeDef *      port;       /* SPI output port */
    uint16_t            pin_sda;    /* SPI SDA pin */
    uint16_t            pin_scl;    /* SPI SCL pin */
};

struct dev_i2c_slave {
    enum en_i2c_dev dev;
    I2C_InitTypeDef config;
    void (*receive_irq)(uint8_t * buffer, uint16_t len);
    /* private */
    struct dev_i2c_out *    periph;
};


void dev_i2c_slave_init(enum en_i2c_dev dev, uint8_t address, uint32_t speed, struct dev_i2c_slave * slave);

#endif //DEV_I2C_SLAVE_H_