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
#include "dev_adc.h"
#include "dev_uart.h"
#include "dev_spi.h"
#include "dev_pwm.h"
#include "dev_i2c_slave.h"
#include "dev_timer.h"
#include "dev_led.h"
#include "dev_spi_slave.h"
#include "state_machine.h"

/* This function overclocks stm32 to 128MHz */
extern uint32_t overclock_stm32f103(void);

/* Callbacks */
void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender);
uint8_t i2c_interrupt(struct i2c_client *, enum i2c_slave_event, uint8_t * byte);
void spi_callback(struct spi_device * dev);
void adc_temp_cbk(struct adc_channel * adc, uint16_t value);
void adc_light_sensor_cbk(struct adc_channel * adc, uint16_t value);

/* Declare glb struct and initialize buffers */
volatile struct tp_glb glb;

DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);
DECLARE_MODULE_LED(led_module, 250);
DECLARE_DEV_LED(led_status, GPIOC, GPIO_Pin_13, &led_module);
DECLARE_ADC_CH(adc_temp, ADC_Channel_TempSensor, NULL, 0);
DECLARE_ADC_CH(adc_light_sensor, ADC_Channel_0, GPIOA, GPIO_Pin_0);
DECLARE_SPI_BUFFER(spi_buffer,uint16_t,1);

static LIST_HEAD(dev_timer_list);

struct pwm_device pwm_chan;
struct i2c_client i2c;
struct adc_channel temp_sensor;
struct spi_device spi_slave;
struct dev_timer * adc_timer; 

/* I2C machine states */
enum en_i2c_commands {
	I2C_CMD_READ_PWM_VALUE = 0xA0,
	I2C_CMD_WRITE_PWM_VALUE = 0xD0,
};
enum en_i2c_states {
	SM_IDLE,
	SM_I2C_READ_PWM,
	SM_I2C_WRITE_PWM,
};
struct state_item_t sys_states[] = {
	[SM_IDLE] = DECLARE_STATE_ITEM(SM_IDLE, NULL, NULL, NULL, NULL),
    [SM_I2C_READ_PWM] = DECLARE_STATE_ITEM(SM_I2C_READ_PWM, NULL, NULL, NULL, NULL),
    [SM_I2C_WRITE_PWM] = DECLARE_STATE_ITEM(SM_I2C_WRITE_PWM, NULL, NULL, NULL, NULL),
};
DECLARE_STATE_MACHINE(sm, sys_states);
uint8_t i2c_cmd;	// last i2c command


void main_loop(void)
{
	/* 1 ms timer */
	if (glb.tmr_1ms) {
		glb.tmr_1ms = 0;
		dev_timer_polling(&dev_timer_list);
	}
	state_handler(&sm);
}

void test_tmr_irq(void * tmp)
{
	{
		int adc_value;
		int temperature;

		const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
		const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V

		adc_value = adc_get_value(adc_temp.index);
		temperature = (uint16_t)((V25-adc_value)/Avg_Slope+25);
		TRACE(("TEMP=%d, T=%d C\n", adc_value, temperature));
	}
	
	TRACE(("LIGHT=%d\n", adc_get_value(adc_light_sensor.index)));

	TRACE(("ADC: "));
	for (int i=1; i<=adc_get_num_of_channels(); i++) {
		TRACE(("%d:%d , ", i, adc_get_value(i)));
	}
	TRACE(("\n"));
}

int main(void)
{
	/* overclock. Comment out for default clocks */
	// SystemCoreClock = overclock_stm32f103();

	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}
	/* Initialize the delay timer */
	delay_init(SystemCoreClock);

	/* enable/disable traces */
	set_trace_level(
			0
			| TRACE_LEVEL_DEFAULT
			| TRACE_LEVEL_SPI
			| TRACE_LEVEL_I2C
			| TRACE_LEVEL_UART
			// | TRACE_LEVEL_PWM
			| TRACE_LEVEL_ADC
			,1);

	/* debug pin */
#ifdef USE_DEBUG_PIN
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef debug_pin;
	debug_pin.GPIO_Pin = DEBUG_PIN;
	debug_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	debug_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_PORT, &debug_pin);
	DEBUG_PORT->ODR &= ~DEBUG_PIN;
#endif


	/* setup uart port */
	dev_uart_add(&dbg_uart);
	/* set callback for uart rx */
	dbg_uart.fp_dev_uart_cb = dbg_uart_parser;
	dev_timer_add((void*) &dbg_uart, 5, (void*) &dev_uart_update, &dev_timer_list);

	/* Initialize led module */
	dev_led_module_init(&led_module);
	/* Attach led module to a timer */
	dev_timer_add((void*) &led_module, led_module.tick_ms, (void*) &dev_led_update, &dev_timer_list);
	/* Add a new led to the led module */
	dev_led_probe(&led_status);
	dev_led_set_pattern(&led_status, 0b00001111);

	/* set up the PWM on TIM1 with a 32KHz freq */
	pwm_add(PWM3_4, &pwm_chan, 32000);
	pwm_set_polarity(&pwm_chan, PWM_POLARITY_NORMAL);
	pwm_set_duty_cycle(&pwm_chan, 10.5);
	pwm_enable(&pwm_chan);

	// pwm_disable(&pwm_chan);

	/* Initialize ADCs */
	adc_module_init();
	adc_add_channel(&adc_light_sensor);
	ADC_TempSensorVrefintCmd(ENABLE);
	adc_add_channel(&adc_temp);
	adc_start();

	/* I2C set up */
	i2c_slave_init(DEV_I2C1, 0x08, 100000, &i2c_interrupt, &i2c);
	i2c_enable(&i2c);

	/* SPI setup */
	spi_set_options(&spi_slave, 16, SPI_MODE_0);
	spi_init_slave(DEV_SPI2, &spi_buffer, &spi_callback, &spi_slave);

	/* State machine */
	init_state_machine(&sm, SM_IDLE);
	// dev_timer_add((void*)&sm, 1, state_handler, &dev_timer_list); // Run machine state every 1ms

	TRACE(("SystemCoreClock: %lu\n", SystemCoreClock));
	// TRACE(("stm32f103 & SPI & TRACE_LEVEL_I2C...\n"));

	while(1) {
		main_loop();
	}
}

void adc_light_sensor_cbk(struct adc_channel * adc, uint16_t value)
{
	TRACE(("LIGHT: %d\n", value));
}


void adc_temp_cbk(struct adc_channel * adc, uint16_t value)
{
	const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
	const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
	int temperature = (uint16_t)((V25-value)/Avg_Slope+25);
	TRACE(("ADC=%d, T=%d C\r\n", value, temperature));
}

union un_spi_cmd {
	uint16_t 	value;
	struct {
		uint8_t channel;
		uint8_t cmd;
	};
};

void spi_callback(struct spi_device * dev)
{
	struct spi_buffers * buffers = (struct spi_buffers *) dev->data;
	// uint16_t * rx_buffer = (uint16_t*) buffers->rx_buffer;
	uint16_t * tx_buffer = (uint16_t*) buffers->tx_buffer;

	// TRACE(("SPI cbk: "));
	// for (int i=0; i<buffers->rx_buffer_len; i++) {
	// 	TRACE(("%04X,", rx_buffer[i]));
	// }
	// TRACE(("\n"));

	
	tx_buffer[0] = adc_get_value(adc_light_sensor.index);

	// union un_spi_cmd cmd;
	// cmd.value = rx_buffer[0];
	// // TRACE(("cmd:%02X, ch:%02X\n", cmd.cmd, cmd.channel));
	// switch (cmd.cmd) {
	// case 0xCD:
	// 	// TRACE(("1\n"));
	// 	if (cmd.channel == 0xA0) {
	// 		// TRACE(("2\n"));
	// 		tx_buffer[1] = adc_get_value(adc_light_sensor.index);
	// 	}
	// 	else if (cmd.channel == 0xA1) {
	// 		// TRACE(("3\n"));
	// 		tx_buffer[1] = adc_get_value(adc_temp.index);
	// 	}
	// 	break;
	// }
#ifdef USE_DEBUG_PIN
	DEBUG_PORT->ODR ^= DEBUG_PIN;
#endif
	// TRACE(("adc: %d\n", tx_buffer[1]));
}

uint8_t i2c_interrupt(struct i2c_client * i2c, enum i2c_slave_event event, uint8_t * byte)
{
	uint8_t resp = 0;
	
	TRACE(("%d: 0x%02X", event, *byte));
	switch (event) {
	case I2C_SLAVE_ADDRESSED:
		TRACE(("1 - %d: 0x%02X\n", event, *byte));
		state_change(&sm, SM_IDLE);
		i2c_cmd = 0;
		break;
	case I2C_SLAVE_READ_REQUESTED:
		TRACE(("2 - %d: 0x%02X\n", event, *byte));
		if (sm.state_curr->state != SM_IDLE) {
			switch(i2c_cmd) {
			case I2C_CMD_READ_PWM_VALUE:
				TRACE(("Read PWM value: %d\n", *byte));
				resp = (uint8_t)pwm_chan.state.duty_cycle;
				break;
			}
			i2c_cmd = 0;
			break;
		}
		break;
	case I2C_SLAVE_WRITE_REQUESTED:
		TRACE(("3 - %d: 0x%02X\n", event, *byte));
		if (sm.state_curr->state != SM_IDLE) {
			switch(i2c_cmd) {
			case I2C_CMD_WRITE_PWM_VALUE:
				TRACEL(TRACE_LEVEL_PWM, ("Set PWM value: %d\n", *byte));
				pwm_set_duty_cycle(&pwm_chan, *byte);
				break;
			}
			i2c_cmd = 0;
			break;
		}
		switch(*byte) {
		case I2C_CMD_READ_PWM_VALUE:
			i2c_cmd = *byte;
			state_change(&sm, SM_I2C_READ_PWM);
			break;
		case I2C_CMD_WRITE_PWM_VALUE:
			i2c_cmd = *byte;
			state_change(&sm, SM_I2C_WRITE_PWM);
			break;
		default:
			i2c_cmd = 0;
			state_change(&sm, SM_IDLE);
			break;
		}
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
	else if (!strncmp((char*) buffer, "ADCTMR=", 7)) {
		uint8_t enable = atoi((char*) &buffer[7]);
		if (enable) {
			if (!adc_timer)
				adc_timer = dev_timer_add(NULL, 2000, (void*) &test_tmr_irq, &dev_timer_list);
			TRACE(("Enable ADC timer\n"));
		}
		else {
			dev_timer_del(adc_timer, &dev_timer_list);
			adc_timer = NULL;
			TRACE(("Disable ADC timer\n"));
		}
	}
	
}
