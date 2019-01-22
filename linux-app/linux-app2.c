/*
 * Copyright (C) MIT.
 *
 * Description: This is an example of using the I2C and SPI interfaces
 * 	to read the ADC value of an SPI photoresistor and then send the value
 * 	on a I2C PWM LED. You can find more info here:
 * 	http://www.stupid-projects.com/linux-and-the-i2c-and-spi-interface/
 *
 * Author: Dimitris Tassopoulos <dimtass@gmail.com>
 *
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define POLLING_MS	50
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define STR_MAX_SIZE 32

#define DEBUG
#ifdef DEBUG
#define TRACE(X) do { printf X ;} while(0)
#else
#define TRACE(x)
#endif

/** SPI **/
struct spi_dev {
	char		device[STR_MAX_SIZE];
	int			fd;
	uint8_t		mode;
	uint8_t		bits;
	uint32_t	speed;
};

int spi_init(struct spi_dev* dev)
{
	dev->fd = open(dev->device, O_RDWR);
	if (dev->fd < 0){
		perror("can't open device");
		exit(1);
	}

	/*
	 * spi mode
	 */
	if (ioctl(dev->fd, SPI_IOC_WR_MODE, &dev->mode)) {
		perror("can't set spi mode");
		exit(1);
	}

	/*
	 * bits per word
	 */
	if (ioctl(dev->fd, SPI_IOC_WR_BITS_PER_WORD, &dev->bits) == -1){
		perror("can't set bits per word");
		exit(1);
	}

	/*
	 * max speed hz
	 */
	if (ioctl(dev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->speed) == -1){
		perror("can't set max speed hz");
		exit(1);
	}

	printf("SPI mode: %d\n", dev->mode);
	printf("SPI bits per word: %d\n", dev->bits);
	printf("SPI max speed: %d Hz (%d KHz)\n", dev->speed, dev->speed/1000);

	return dev->fd;
}

static void spi_recv(struct spi_dev * dev)
{
	int ret;
	uint8_t tx[4] = {0xCD, 0xA0,};
	uint8_t rx[4] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = 4,
	};

	ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		perror("can't send spi message");
		exit(1);
	}

	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
}

int spi_send16(struct spi_dev * dev, uint16_t data)
{
	uint8_t tx[2] = {0};
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = 0,
		.speed_hz = dev->speed,
		.bits_per_word = dev->bits,
	};
	tx[0] = (data >> 8) & 0xFF;
	tx[1] = data & 0xFF;

	if (ioctl(dev->fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
		perror("can't send spi message");
		exit(1);
	}
	return tr.len;
}

void print_usage(int argc, char** argv)
{
	printf(
		"Usage:\n"
		"%s [SPI DEV]\n"
		"	[SPI DEV]: the SPI device/bus that the PWM LED is connected (e.g. /dev/spidev0.0)\n\n",
		argv[0]
	);
}


int main(int argc, char** argv) {

	print_usage(argc, argv);

	struct spi_dev spi = {
		.fd = -1,
		.mode = 0,
		.bits = 8,
		.speed = 30000000,
	};
	strncpy(spi.device, argv[1], STR_MAX_SIZE);
	printf("Application started\n"); /* prints Hello World */

	/* init spidev */
	spi_init(&spi);

	/* loop forever */
	// while (1) {
		/* send the PWM LED value via SPI */
	// spi_send16(&spi, adc_value);
	// spi_send16(&spi, 0x1234);
	// spi_send16(&spi, 0xFFFF);
	spi_recv(&spi);
		// TRACE((":  %d\n", adc_value));
		/* sleep */
		// usleep(POLLING_MS * 1000);
	// }

	/* clean up */
	close(spi.fd);

    return 0;
}


