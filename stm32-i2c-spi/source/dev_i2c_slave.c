
#include "stm32f10x.h"
#include "dev_i2c_slave.h"

static struct dev_i2c_out i2c_out[] = {
    [DEV_I2C1] = {I2C1, GPIOB, GPIO_Pin_7, GPIO_Pin_6},
    [DEV_I2C2] = {I2C2, GPIOB, GPIO_Pin_11, GPIO_Pin_10},
};


void dev_i2c_slave_init(enum en_i2c_dev dev, uint8_t address, uint32_t speed, struct dev_i2c_slave * slave)
{
    /* set up GPIOs */
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    slave->dev = dev;
    slave->periph = &i2c_out[slave->dev];
    GPIO_InitStructure.GPIO_Pin = slave->periph->pin_sda | slave->periph->pin_scl;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(slave->periph->port, &GPIO_InitStructure);

    /* set up interrupts */
    if (slave->dev == DEV_I2C1)
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    else if (slave->dev == DEV_I2C2)
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* error interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* I2C configuration */
    slave->config.I2C_Mode = I2C_Mode_I2C;
    slave->config.I2C_DutyCycle = I2C_DutyCycle_2;
    slave->config.I2C_OwnAddress1 = address;
    slave->config.I2C_Ack = I2C_Ack_Enable;
    slave->config.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    slave->config.I2C_ClockSpeed = speed;

    /* I2C Peripheral Enable */
    I2C_Cmd(slave->periph->i2c, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C1, &slave->config);

    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE); //Part of the STM32 I2C driver
    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE); //Part of the STM32 I2C driver
}

void I2C1_ClearFlag(void) {
    // ADDR-Flag clear
    while((I2C1->SR1 & I2C_SR1_ADDR) == I2C_SR1_ADDR) {
        I2C1->SR1;
        I2C1->SR2;
    }

    // STOPF Flag clear
    while((I2C1->SR1&I2C_SR1_STOPF) == I2C_SR1_STOPF) {
        I2C1->SR1;
        I2C1->CR1 |= 0x1;
    }
}
/*******************************************************************/

/*******************************************************************/
void I2C1_EV_IRQHandler(void) {
    uint32_t event;
    uint8_t wert;

    // Reading last event
    event=I2C_GetLastEvent(I2C1);

    if(event==I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) {
        // Master has sent the slave address to send data to the slave
        i2c1_mode=I2C1_MODE_SLAVE_ADR_WR;
    }
    else if(event==I2C_EVENT_SLAVE_BYTE_RECEIVED) {
        // Master has sent a byte to the slave
        wert=I2C_ReceiveData(I2C1);
        // Check address
        if(i2c1_mode==I2C1_MODE_SLAVE_ADR_WR) {
            i2c1_mode=I2C1_MODE_ADR_BYTE;
            // Set current ram address
            i2c1_ram_adr=wert;
        }
        else {
            i2c1_mode=I2C1_MODE_DATA_BYTE_WR;
            // Store data in RAM
            set_i2c1_ram(i2c1_ram_adr, wert);
            // Next ram adress
            i2c1_ram_adr++;
        }
    }
    else if(event==I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) {
        // Master has sent the slave address to read data from the slave
        i2c1_mode=I2C1_MODE_SLAVE_ADR_RD;
        // Read data from RAM
        wert=get_i2c1_ram(i2c1_ram_adr);
        // Send data to the master
        I2C_SendData(I2C1, wert);
        // Next ram adress
        i2c1_ram_adr++;
    }
    else if(event==I2C_EVENT_SLAVE_BYTE_TRANSMITTED) {
        // Master wants to read another byte of data from the slave
        i2c1_mode=I2C1_MODE_DATA_BYTE_RD;
        // Read data from RAM
        wert=get_i2c1_ram(i2c1_ram_adr);
        // Send data to the master
        I2C_SendData(I2C1, wert);
        // Next ram adress
        i2c1_ram_adr++;
    }
    else if(event==I2C_EVENT_SLAVE_STOP_DETECTED) {
        // Master has STOP sent
        I2C1_ClearFlag();
        i2c1_mode=I2C1_MODE_WAITING;
    }
}

    /*******************************************************************/
void I2C1_ER_IRQHandler(void) {
    if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
    }
}
