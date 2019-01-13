
#include "stm32f10x.h"
#include "dev_i2c_slave.h"

#define DEBUG(X) TRACEL(TRACE_LEVEL_PWM, X)

LIST_HEAD(i2c_list);

static struct i2c_adapter m_i2c_adapter[] = {
    [DEV_I2C1] = {I2C1, GPIOB, GPIO_Pin_7, GPIO_Pin_6},
    [DEV_I2C2] = {I2C2, GPIOB, GPIO_Pin_11, GPIO_Pin_10},
};


static inline struct i2c_client * i2c_find(I2C_TypeDef * spi_ch)
{
	if (!list_empty(&i2c_list)) {
		struct i2c_client * i2c_it = NULL;
		list_for_each_entry(i2c_it, &i2c_list, list) {
			if (i2c_it->adapter->i2c == spi_ch) {
				/* found */
				return(i2c_it);
			}
		}
	}
	return NULL;
}

int i2c_slave_init(enum en_i2c_dev dev, uint16_t address, uint32_t speed_hz,
                    i2c_slave_cb_t callback, struct i2c_client * i2c)
{

    // test();
    // return 0;

    if (dev >= DEV_EOL) {
        return -1;
    }
	RCC_PCLK1Config(RCC_HCLK_Div4);    /* Clock APB1 */
    
	list_add(&i2c->list, &i2c_list);

    i2c->adapter = &m_i2c_adapter[dev];
    i2c->address = address;
    i2c->info.dev = dev;
    i2c->info.speed_hz = speed_hz;
    i2c->slave_cb = callback;

    I2C_SoftwareResetCmd(i2c->adapter->i2c, ENABLE);
	I2C_SoftwareResetCmd(i2c->adapter->i2c, DISABLE);

    /* Set clock power */
    if (i2c->adapter->i2c == I2C1) {
        I2C_Cmd(I2C1, DISABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    }
    else if (i2c->adapter->i2c == I2C1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    }
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* set up GPIOs */
    GPIO_InitTypeDef  gpio;
    gpio.GPIO_Pin = i2c->adapter->pin_sda | i2c->adapter->pin_scl;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(i2c->adapter->port, &gpio);

    /* set up interrupts */
    NVIC_InitTypeDef nvic;
    if (i2c->adapter->i2c == I2C1)
        nvic.NVIC_IRQChannel = I2C1_EV_IRQn;
    else if (i2c->adapter->i2c == I2C2)
        nvic.NVIC_IRQChannel = I2C2_EV_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* error interrupt */
    if (i2c->adapter->i2c == I2C1)
        nvic.NVIC_IRQChannel = I2C1_ER_IRQn;
    else if (i2c->adapter->i2c == I2C2)
        nvic.NVIC_IRQChannel = I2C2_ER_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);

    /* I2C configuration */
    I2C_InitTypeDef * conf = &i2c->info.config;
    I2C_StructInit(conf);
    conf->I2C_Mode = I2C_Mode_I2C;
    conf->I2C_DutyCycle = I2C_DutyCycle_2;
    conf->I2C_OwnAddress1 = (i2c->address << 1);
    conf->I2C_Ack = I2C_Ack_Enable;
    conf->I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    conf->I2C_ClockSpeed = i2c->info.speed_hz;
    I2C_Init(i2c->adapter->i2c, conf);

    I2C_ITConfig(i2c->adapter->i2c, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE); //Part of the STM32 I2C driver
    // I2C_ITConfig(i2c->adapter->i2c, I2C_IT_BUF, ENABLE);
    /* I2C Peripheral Enable */
    I2C_Cmd(i2c->adapter->i2c, ENABLE);
    i2c->info.enabled = 1;

    DEBUG(("Setting up I2C with address: 0x%02X and speed: %lu\n", i2c->address, i2c->info.speed_hz));

    return 0;
}

void i2c_enable(struct i2c_client * i2c)
{
    I2C_Cmd(i2c->adapter->i2c, ENABLE);
    i2c->info.enabled = 1;
}

void i2c_disable(struct i2c_client * i2c)
{
    I2C_Cmd(i2c->adapter->i2c, DISABLE);
    i2c->info.enabled = 0;
}

void I2Cx_ClearFlag(struct i2c_client * i2c) {
    // ADDR-Flag clear
    I2C_TypeDef * dev = i2c->adapter->i2c;
    while((dev->SR1 & I2C_SR1_ADDR) == I2C_SR1_ADDR) {
        dev->SR1;
        dev->SR2;
    }

    // STOPF Flag clear
    while((dev->SR1 & I2C_SR1_STOPF) == I2C_SR1_STOPF) {
        dev->SR1;
        dev->CR1 |= 0x1;
    }
}

void I2C1_EV_IRQHandler(void) {
    uint32_t event;
    uint8_t byte;

    struct i2c_client * i2c = i2c_find(I2C1);
    I2C_TypeDef * dev = i2c->adapter->i2c;

    if (!i2c) {
        return;
    }

    // if (i2c->adapter->i2c == I2C1)
    //     PORT_STATUS_LED->ODR ^= PIN_STATUS_LED;

    // Reading last event
    event = I2C_GetLastEvent(dev);

    if (event == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) {
        // Master has sent the slave address to send data to the slave
        i2c->slave_cb(i2c, I2C_SLAVE_ADDRESSED, NULL);
    }
    else if (event == I2C_EVENT_SLAVE_BYTE_RECEIVED) {
        // Master has sent a byte to the slave
        byte = I2C_ReceiveData(dev);
        i2c->slave_cb(i2c, I2C_SLAVE_READ_REQUESTED, &byte);
    }
    /* Master reads data from slave */
    else if (event == I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) {
        byte = 0xAA;
        byte = i2c->slave_cb(i2c, I2C_SLAVE_READ_REQUESTED, &byte);
        I2C_SendData(dev, byte);
        // // Master has sent the slave address to read data from the slave
        // i2c1_mode = I2C1_MODE_SLAVE_ADR_RD;
        // // Read data from RAM
        // byte=get_i2c1_ram(i2c1_ram_adr);
        // // Send data to the master
        // I2C_SendData(I2C1, byte);
        // // Next ram adress
        // i2c1_ram_adr++;
    }
    else if (event == I2C_EVENT_SLAVE_BYTE_TRANSMITTED) {
        byte = 0xBB;
        byte = i2c->slave_cb(i2c, I2C_SLAVE_READ_REQUESTED, &byte);
        I2C_SendData(dev, byte);
        // // Master wants to read another byte of data from the slave
        // i2c1_mode=I2C1_MODE_DATA_BYTE_RD;
        // // Read data from RAM
        // wert=get_i2c1_ram(i2c1_ram_adr);
        // // Send data to the master
        // I2C_SendData(I2C1, wert);
        // // Next ram adress
        // i2c1_ram_adr++;
    }
    else if (event == I2C_EVENT_SLAVE_STOP_DETECTED) {
        // Master has STOP sent
        I2Cx_ClearFlag(i2c);
    }
}

/*******************************************************************/
void I2C1_ER_IRQHandler(void) {
    if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
    }
}

void I2C2_ER_IRQHandler(void) {
    if (I2C_GetITStatus(I2C2, I2C_IT_AF)) {
        I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
    }
}