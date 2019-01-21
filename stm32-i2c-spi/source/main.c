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

/* This function overclocks stm32 to 128MHz */
extern uint32_t overclock_stm32f103(void);

/* Callbacks */
void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender);
uint8_t i2c_interrupt(struct i2c_client *, enum i2c_slave_event, uint8_t * byte);
void adc_temp_cbk(struct adc_channel * adc, uint16_t value);
void adc_light_sensor_cbk(struct adc_channel * adc, uint16_t value);

/* Declare glb struct and initialize buffers */
volatile struct tp_glb glb;

DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);
DECLARE_MODULE_LED(led_module, 250);
DECLARE_DEV_LED(led_status, GPIOC, GPIO_Pin_13, &led_module);
DECLARE_ADC_CH(adc_temp, ADC_Channel_TempSensor, NULL, 0);
DECLARE_ADC_CH(adc_light_sensor, ADC_Channel_0, GPIOA, GPIO_Pin_0);
DECLARE_SPI_BUFFER(spi_buffer,uint16_t,3);

static LIST_HEAD(dev_timer_list);

struct pwm_device pwm_chan;
struct i2c_client i2c;
struct adc_channel temp_sensor;
struct spi_device spi_slave;

int counter = 0;
int fps = 0;


void main_loop(void)
{
	/* 10 ms timer */
	if (glb.tmr_1ms) {
		glb.tmr_1ms = 0;
		dev_timer_polling(&dev_timer_list);
	}
}

void spi_callback(struct spi_device * dev)
{
	struct spi_buffers * buffers = (struct spi_buffers *) dev->data;
	uint16_t * rx_buffer = (uint16_t*) buffers->rx_buffer;
	uint16_t * tx_buffer = (uint16_t*) buffers->tx_buffer;

	TRACE(("SPI cbk: "));
	for (int i=0; i<buffers->rx_buffer_len; i++) {
		TRACE(("%04X,", rx_buffer[i]));
	}
	TRACE(("\n"));

	for (int i=1; i<buffers->tx_buffer_len; i++) {
		tx_buffer[i] += 1;
		TRACE(("%04X,", tx_buffer[i]));
	}
	TRACE(("\n"));

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
	SystemCoreClock = overclock_stm32f103();

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
			| TRACE_LEVEL_PWM
			| TRACE_LEVEL_ADC
			,1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* debug led */
#ifdef DEBUG_PORT
	GPIO_InitTypeDef debug_pin;
	debug_pin.GPIO_Pin = DEBUG_PIN;
	debug_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	debug_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_PORT, &led);
	DEBUG_PORT->ODR |= DEBUG_PIN;
#endif


	/* setup uart port */
	dev_uart_add(&dbg_uart);
	/* set callback for uart rx */
	dbg_uart.fp_dev_uart_cb = dbg_uart_parser;

	/* Initialize led module */
	dev_led_module_init(&led_module);
	/* Attach led module to a timer */
	dev_timer_add((void*) &led_module, led_module.tick_ms, (void*) &dev_led_update, &dev_timer_list);
	/* Add a new led to the led module */
	dev_led_probe(&led_status);
	dev_led_set_pattern(&led_status, 0b00001111);


	/* set up the PWM on TIM1 with a 32KHz freq */
	// pwm_add(PWM1_1, &pwm_chan, 32000);
	// pwm_set_polarity(&pwm_chan, PWM_POLARITY_NORMAL);
	// pwm_set_duty_cycle(&pwm_chan, 53.5);
	// pwm_enable(&pwm_chan);

	// pwm_disable(&pwm_chan);

	/* Initialize ADCs */
	adc_module_init();
	adc_add_channel(&adc_light_sensor);
	ADC_TempSensorVrefintCmd(ENABLE);
	adc_add_channel(&adc_temp);
	adc_start();
	
	// dev_timer_add(NULL, 2000, (void*) &test_tmr_irq, &dev_timer_list);

	/* I2C set up */
	i2c_slave_init(DEV_I2C1, 0x08, 100000, &i2c_interrupt, &i2c);
	i2c_enable(&i2c);

	spi_set_options(&spi_slave, 16, SPI_MODE_0);
	spi_init_slave(DEV_SPI2, &spi_buffer, &spi_callback, &spi_slave);

	TRACE(("SystemCoreClock: %lu\n", SystemCoreClock));
	TRACE(("stm32f103 & SPI & TRACE_LEVEL_I2C...\n"));
	// main2();
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

volatile uint8_t i2c_counter = 0;

uint8_t i2c_interrupt(struct i2c_client * i2c, enum i2c_slave_event event, uint8_t * byte)
{
	uint8_t resp = 0;
	
#ifdef DEBUG_PORT
	DEBUG_PORT->ODR ^= DEBUG_PIN;
#endif
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
