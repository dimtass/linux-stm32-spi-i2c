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

void dbg_uart_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender);

/* Declare glb struct and initialize buffers */
volatile struct tp_glb glb;

DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);

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

	/* enable/disable traces */
	set_trace_level(
			0
			| TRACE_LEVEL_DEFAULT
			| TRACE_LEVEL_SPI
			| TRACE_LEVEL_I2C
			| TRACE_LEVEL_UART
			,1);
	/* setup uart port */
	dev_uart_add(&dbg_uart);
	/* set callback for uart rx */
	dbg_uart.fp_dev_uart_cb = dbg_uart_parser;

	TRACE(("stm32f103 & SPI & TRACE_LEVEL_I2C...\n"));

	while(1) {
		main_loop();
	}
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
	// if (!strncmp((char*) buffer, "MODE=", 5)) {
	// 	if (!strncmp((char*) &buffer[5], "BENCH", 5)) {
	// 	}
	// 	else if (!strncmp((char*) &buffer[5], "CALIB", 5)) {
	// 	}
	// }
}
