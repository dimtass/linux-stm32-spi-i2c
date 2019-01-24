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
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <argp.h>

#define POLLING_MS	50
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define STR_MAX_SIZE 32

#define DEBUG
#ifdef DEBUG
#define TRACE(X) do { printf X ;} while(0)
#else
#define TRACE(x)
#endif

/** I2C **/
#define I2C_ADDR 0x08

struct i2c_dev {
	char 	device[STR_MAX_SIZE];
	int		fd;
	int		addr;
};

int i2c_init(char * dev)
{
	int fd;
	if ((fd = open(dev, O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    perror("Failed to open the i2c bus");
	    exit(1);
	}
	return fd;
}

uint8_t i2c_read_pwm(int i2c_addr, int i2c_fd)
{
	uint8_t byte = 0;

	if (ioctl(i2c_fd, I2C_SLAVE, i2c_addr) < 0) {
		perror("Failed to acquire bus access and/or talk to slave.\n");
		exit(1);
		/* ERROR HANDLING; you can check errno to see what went wrong */
	}
	/* Send command */
	byte = 0xA0;
	if (write(i2c_fd, &byte, 1) != 1) {
		printf("Failed to write to bus (%d).\n", i2c_fd);
		return 0;
	}
	/* Read value */
	if (read(i2c_fd, &byte, 1) != 1) {
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to read from the i2c bus (%d).\n", i2c_fd);
	}
	return byte;
}

void i2c_write_pwm(int i2c_addr, int i2c_fd, uint8_t value)
{
	uint8_t spi_buf[2];

	if (ioctl(i2c_fd, I2C_SLAVE, i2c_addr) < 0) {
		perror("Failed to acquire bus access and/or talk to slave.\n");
		exit(1);
		/* ERROR HANDLING; you can check errno to see what went wrong */
	}
	/* Send command */
	spi_buf[0] = 0xD0;
	spi_buf[1] = value;
	if (write(i2c_fd, &spi_buf, 2) != 2) {
		printf("Failed to write to bus (%d).\n", i2c_fd);
		return;
	}
}

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
	if (dev->fd) {
		close(dev->fd);
	}
	// fd is either closed or not accessible 
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

	return dev->fd;
}

enum spi_read_cmd {
	SPI_READ_LIGHT = 0xA0,
	SPI_READ_TEMP = 0xA1,
};

static uint16_t spi_read_light(struct spi_dev * dev, enum spi_read_cmd cmd, uint8_t enable_printf)
{
	#define BUF_SIZE 2
	int ret;
	uint8_t tx[BUF_SIZE] = {0xCD, cmd, };
	uint8_t rx[BUF_SIZE] = {0xFF, 0xFF};
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = BUF_SIZE,
		// .delay_usecs = 10,
	};

	ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		perror("can't send spi message");
		exit(1);
	}
	if (enable_printf) {
		for (int i=0; i<BUF_SIZE; i++)
			printf("%02X ", rx[i]);
		printf("\n");
	}
	return (rx[0] << 8) | rx[1];
}

void print_usage(int argc, char** argv)
{
	printf(
		"Usage:\n"
		"%s [I2C DEV] [SPI DEV]\n"
		"	[I2C DEV]: the I2C device/bus that the photoresistor is connected (e.g. /dev/i2c-0)\n"
		"	[SPI DEV]: the SPI device/bus that the PWM LED is connected (e.g. /dev/spidev0.0)\n\n",
		argv[0]
	);
}

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}


static void usage(const char *argv0)
{
    fprintf(stderr, "Usage:\n"
		"%s [-i I2C_DEV] [-s SPI DEV] [-m MODE] [-s SAMPLES]\n"
		"\t-i   : the I2C device/bus that the photoresistor is connected (e.g. /dev/i2c-0)\n"
		"\t-s   : the SPI device/bus that the PWM LED is connected (e.g. /dev/spidev0.0)\n"
		"\t-b   : SPI baudrate (default:1000000)\n"
		"\t-r   : Number of runs/iterations for the SPI/I2C read/write (default:-1, run forever)\n"
		"\t-m   : mode. 0: Normal (default), 1: Benchmark [1,2,5,10,20,30MHz]\n"
		"\t-p   : enable printf. This creates a dummy load by printing the SPI values (default:0)\n"
		,
		argv0);
    exit(EXIT_FAILURE);
}

enum en_mode {
	MODE_NORMAL = 0,
	MODE_BENCHMARK
};

enum en_benchmark_speed {
	SPI_1MHz,
	SPI_2MHz,
	SPI_5MHz,
	SPI_10MHz,
	SPI_20MHz,
	SPI_30MHz,
	SPI_SPEED_END
};
#define DEFAULT_ITERATIONS 10

int main(int argc, char** argv)
{
	int opt;
	int mode = MODE_NORMAL;
	int spi_baud = 1000000;
	int iterations = -1;
	int spi_speed_index = 0;
	uint8_t en_printf = 0;

	/* Create the I2C data */
	struct i2c_dev i2c = {
		.fd = -1,
		.addr = I2C_ADDR
	};

	struct spi_dev spi = {
		.fd = -1,
		.mode = 0,
		.bits = 8,
		.speed = 1000000,
	};
	int spi_benchmark_speeds[SPI_SPEED_END] = {
		[SPI_1MHz] = 1000000,
		[SPI_2MHz] = 2000000,
		[SPI_5MHz] = 5000000,
		[SPI_10MHz] = 10000000,
		[SPI_20MHz] = 20000000,
		[SPI_30MHz] = 30000000,
	};

    while ((opt = getopt(argc, argv, "i:s:b:m:r:p:h:")) != -1)
    {
        switch (opt)
        {
		case 'i':
			strncpy(i2c.device, optarg, STR_MAX_SIZE);
			break;
        case 's':
			strncpy(spi.device, optarg, STR_MAX_SIZE);
            break;
        case 'm':
            mode = atoi(optarg);
            break;
        case 'b':
            spi.speed = atoi(optarg);
            break;
		case 'r':
			iterations = atoi(optarg);
			break;
		case 'p':
			en_printf = atoi(optarg);
			break;
		case 'h':
        default:
            usage(argv[0]);
			exit(0);
        }
    }

    // if (argc - optind < 2)
    // {
    //     fprintf(stderr, "%s: too few arguments. Needs at least -i and -s. Got %d\n", argv[0], argc);
    //     usage(argv[0]);
    // }

	printf("== Settings:\n\tMode: ");
	switch (mode) {
	case MODE_NORMAL:
		printf("Fast\n");
		break;
	case MODE_BENCHMARK:
		printf("Benchmark\n");
		break;
	};
	printf("\tNumber of runs: %d\n", iterations);
	printf("\tSPI dev: %s\n", spi.device);
	printf("\tI2C dev: %s\n", i2c.device);
	printf("\n"); /* prints Hello World */

	/* init i2c */
	i2c.fd = i2c_init(i2c.device);

	/* init spidev */
	spi.fd = spi_init(&spi);

	uint32_t counter = 0;
	long long start = current_timestamp();
	long long stop = 0;
	int msec = 0;
	int *benchmark_results;

	printf("Start:\n\n");

	if (mode == MODE_BENCHMARK) {
		spi.speed = spi_benchmark_speeds[0];
		printf("== Benchmark mode\n\tSPI speed: %d Hz (%d KHz)\n",
				spi.speed, spi.speed/1000);
		if (iterations < 1) iterations = DEFAULT_ITERATIONS;
		benchmark_results = (int*)malloc(iterations * SPI_SPEED_END * sizeof(int));
	};
	int iter_cntr = iterations;
	int bench_res_cntr = 0;

	/* don't try to figure out how this works. It just does. */
	while(iter_cntr) {
		uint16_t adc = spi_read_light(&spi, SPI_READ_LIGHT, en_printf);
		i2c_write_pwm(i2c.addr, i2c.fd, (100*adc)/0x4095);
		counter++;

		stop = current_timestamp();
		if (stop - start >= 1000) {
			start = current_timestamp();
			
			if (mode == MODE_BENCHMARK)
				benchmark_results[bench_res_cntr++] = counter;
			printf("%d\n", counter);
			counter = 0;
			if (iter_cntr) iter_cntr--;

			/* When in benchmark mode repeat for all spi speeds */
			if (!iter_cntr && (mode == MODE_BENCHMARK)) {

				if ((++spi_speed_index) >= SPI_SPEED_END) break;
				spi.speed = spi_benchmark_speeds[spi_speed_index];
				printf("\tSPI speed: %d Hz (%d KHz)\n",
							spi.speed, spi.speed/1000);
				iter_cntr = iterations;
				/* update spi */
				if (ioctl(spi.fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi.speed) == -1){
					perror("can't set max speed hz");
					exit(1);
				}
			}
		}
	}

	/* print results? */
	if (mode == MODE_BENCHMARK) {
		printf("\nResults: %d\n", bench_res_cntr);
		int i, k = 0;
		for (i=0; i<SPI_SPEED_END; i++) {
			printf("\tSPI speed: %d Hz (%d KHz)\n",
						spi_benchmark_speeds[i], spi_benchmark_speeds[i]/1000);
			for (k=0; k<iterations; k++) {
				printf("%d\n", *(benchmark_results + i*iterations + k));
			}
		}
	}

	/* clean up */
	close(i2c.fd);
	close(spi.fd);

    return 0;
}


