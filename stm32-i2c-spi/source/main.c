/**
 * This is a demo for the ILI9341 spi display.
 * 
 * st-flash --reset write build-stm32/stm32f103-ili9341-dma.bin 0x8000000
 * 
 *  Created on: Jul 5, 2018
 *      Author: Dimitris Tassopoulos
*/

#include <stdio.h>
#include "cortexm_delay.h"
#include "platform_config.h"


/* This function overclocks stm32 to 128MHz */
extern uint32_t overclock_stm32f103(void);

/* Callbacks */
void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender);
uint8_t i2c_interrupt(struct i2c_client *, enum i2c_slave_event, uint8_t * byte);

/* Declare glb struct and initialize buffers */
volatile struct tp_glb glb;

DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);

struct pwm_device pwm_chan;
struct i2c_client i2c;

int counter = 0;
int fps = 0;

void main_loop(void)
{
	/* 10 ms timer */
	if (glb.tmr_10ms) {
		glb.tmr_10ms = 0;
		dev_uart_update(&dbg_uart);
	}
}

int main(void)
{
	/* overclock. Comment out for default clocks */
	SystemCoreClock = overclock_stm32f103();

	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}
	/* Initialize the delay timer */
	delay_init(SystemCoreClock);

	// RCC_PCLK1Config(RCC_HCLK_Div2);    /* Clock APB1 */
	// RCC_PCLK2Config(RCC_HCLK_Div2);    /* Clock APB1 */

	/* enable/disable traces */
	set_trace_level(
			0
			| TRACE_LEVEL_DEFAULT
			| TRACE_LEVEL_SPI
			| TRACE_LEVEL_I2C
			| TRACE_LEVEL_UART
			| TRACE_LEVEL_PWM
			,1);
	/* debug led */
	GPIO_InitTypeDef led;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	led.GPIO_Pin = PIN_STATUS_LED;
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_STATUS_LED, &led);

	PORT_STATUS_LED->ODR |= PIN_STATUS_LED;

	/* setup uart port */
	dev_uart_add(&dbg_uart);
	/* set callback for uart rx */
	dbg_uart.fp_dev_uart_cb = dbg_uart_parser;

	/* set up the PWM on TIM1 with a 32KHz freq */
	// pwm_add(PWM1_1, &pwm_chan, 32000);
	// pwm_set_polarity(&pwm_chan, PWM_POLARITY_NORMAL);
	// pwm_set_duty_cycle(&pwm_chan, 53.5);
	// pwm_enable(&pwm_chan);

	// pwm_disable(&pwm_chan);

	/* I2C set up */
	i2c_slave_init(DEV_I2C1, 0x08, 100000, &i2c_interrupt, &i2c);
	i2c_enable(&i2c);

	TRACE(("SystemCoreClock: %lu\n", SystemCoreClock));
	TRACE(("stm32f103 & SPI & TRACE_LEVEL_I2C...\n"));
	// main2();
	while(1) {
		main_loop();
	}
}

volatile uint8_t i2c_counter = 0;

uint8_t i2c_interrupt(struct i2c_client * i2c, enum i2c_slave_event event, uint8_t * byte)
{
	uint8_t resp = 0;
	PORT_STATUS_LED->ODR ^= PIN_STATUS_LED;
	// TRACE(("%d: 0x%02X", event, *byte));
	switch (event) {
	case I2C_SLAVE_ADDRESSED:
		TRACE(("1 - %d: 0x%02X\n", event, *byte));
		break;
	case I2C_SLAVE_READ_REQUESTED:
		resp = 0xAA; //i2c_counter++;
		TRACE(("2 - %d: 0x%02X\n", event, *byte));
		break;
	case I2C_SLAVE_WRITE_REQUESTED:
		TRACE(("3 - %d: 0x%02X\n", event, *byte));
		resp = 0xBB;
		break;
	case I2C_SLAVE_READ_PROCESSED:
		TRACE(("4 - %d: 0x%02X\n", event, *byte));
		break;
	case I2C_SLAVE_WRITE_RECEIVED:
		TRACE(("5 - %d: 0x%02X\n", event, *byte));
		resp = 0xB1;
		break;
	case I2C_SLAVE_STOP:
		TRACE(("6 - %d: 0x%02X\n", event, *byte));
		break;
	};
	return resp;
}

/* Supported commands :
 * 	MODE=<mode>
 * 	where <mode>,
 * 		BENCH	: benchmark mode
 * 		CALIB	: touch calibrations mode
 *
 * 	FPS=<en>
 * 	where <en>:
 * 		0	: disable both trace and show FPS
 * 		1	: trace FPS on uart but disable on LCD
 * 		3	: trace and show FPS
 *
 */
void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender)
{
	buffer[bufferlen] = 0;
	TRACEL(TRACE_LEVEL_UART, ("dbg_uart_parser: %s\n", buffer));
	if (!strncmp((char*) buffer, "PWMD=", 5)) {
		float value = atof((char*) &buffer[5]);
		pwm_set_duty_cycle(&pwm_chan, value);
		TRACE(("Setting PWM duty-cycle to: %f\n", value));
	}
	else if (!strncmp((char*) buffer, "PWMF=", 5)) {
		uint32_t value = atoi((char*) &buffer[5]);
		pwm_config(&pwm_chan, value);
		TRACE(("Setting PWM freq to: %lu\n", value));
	}
	else if (!strncmp((char*) buffer, "PWME=", 5)) {
		uint8_t enable = atoi((char*) &buffer[5]);
		if (enable)
			pwm_enable(&pwm_chan);
		else
			pwm_disable(&pwm_chan);
		TRACE(("Enable PWM: %d\n", enable));
	}
}
