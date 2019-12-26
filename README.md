Linux, the I2C and SPI interface and the SMP and PREEMPT-RT kernel
----
This source code is part of this stupi-project here:
https://www.stupid-projects.com/linux-and-the-i2c-and-spi-interfaces-part-2/

The first part, which uses an Arduino nano is here:
https://www.stupid-projects.com/linux-and-the-i2c-and-spi-interfaces/

## Hardware
For testing I've used the [stm32f103 - bluepill](https://wiki.stm32duino.com/index.php?title=Blue_Pill)
to emulate two devices, one SPI photoresistor sensor and one I2C PWM LED.
These devices, for example, could be two different attiny85; but as the
stm32f103 is power enough for the task, we can use the same chip to emulate
both. Also this mcu runs at 72MHz and has a DMA for most of its peripherals
and if it's overcolcked to 128MHz can achieve ~62MHz SPI, see
[here](https://www.stupid-projects.com/driving-an-ili9341-lcd-with-an-overclocked-stm32f103/)

Also, I'm using the [nanopi-neo](http://wiki.friendlyarm.com/wiki/index.php/NanoPi_NEO)
for the Linux hardware as it's a low spec board with an allwinner H3,
it's really cheap and common to find and it has both an I2C and SPI interface.

Also a random-whatevah photoresistor and LED are used but not harmed during the experiment.

## Connection
These are the connections between the `Nanopi-neo` and the `stm32f103`.

Signal | STM32F103 | Nanopi-neo
-|-|-
/SS | B12 | 24 (SPI0_CS)
MOSI | B15 | 19 (SPI0_MISO)
MISO | B14 | 21 (SPI0_MOSI)
SCK | B13 | 23 (SPI0_CLK)
SDA | PB7 | 3 (I2C0_SDA)
SCL | PB6 | 5 (I2C0_SCL)

> For the `Nanopi-neo` pinout header you can have a look [here](http://wiki.friendlyarm.com/wiki/index.php/NanoPi_NEO)

The photoresistor is connected on the PA0 pin and the PWM output is set to PB10.

For the SPI slave DMA is used and for the I2C only interrupts, though DMA is available, too.

## Yocto build
```sh
mkdir -p yocto/sources
cd yocto/sources
git clone --depth 1 -b zeus git://git.yoctoproject.org/poky
git clone --depth 1 -b zeus git@github.com:openembedded/meta-openembedded.git
git clone --depth 1 git@bitbucket.org:dimtass/meta-allwinner-hx.git
mv ../../meta-allwinner-i2c-spi-arduino .
cp meta-allwinner-hx/scripts/setup-environment.sh .
cp meta-allwinner-i2c-spi-arduino/flash_sd.sh .
cd ..
```

Now you should be in the `yocto/` folder.
```sh
MACHINE=nanopi-neo source ./setup-environment.sh build
bitbake arduino-test-image
bitbake -c populate_sdk arduino-test-image
```

This might take a lot of time depending your build system. In the end you get an SDK
to build the kernel modules and the SD image. To copy the image to the SD card you
can either run the `flash_sd.sh` like that:

```sh
MACHINE=nanopi-neo ./flash_sd.sh /dev/sdX
```

or use the bmap-tool. You need to have the bmap-tool install to flash the image.
To install it just run this:
```sh
sudo apt install bmap-tools
```

Or get the last version from [here](https://github.com/intel/bmap-tools).

## Preparing the build environment
You'll need to build some modules and code for this project, therefore you
need first to build the Yocto SDK as explained before and the install it in
its default location. Therefore, after running the `populate_sdk` task with
bitbake, you'll find the sdk in the `build/tmp/deploy/sdk` folder. To install
it then run this command to your workstation:
```sh
./build/tmp/deploy/sdk/poky-glibc-x86_64-arduino-test-image-armv7vehf-neon-toolchain-2.5.1.sh
```

This will install the SDK in the `/opt/poky/2.5.1/` folder.

From now on to build any code you need to open the terminal and run these
commands:
```sh
source /opt/poky/2.5.1/environment-setup-armv7vehf-neon-poky-linux-gnueabi
```

## User space app
From the user space you can have access to the devices without the need of
special drivers, by just using the Linux API that exposes the I2C interface.
In case of the SPI a driver is needed though, but that comes with the OS
and you don't have to write your own and it's called `spidev`. In the Yocto
image the proper kernel flags are enable and also the `spidev` is registered
in the OS by loading the `sun8i-h3-spi-spidev` device-tree overlay in
`/boot/allwinnerEnv.txt`.

To build the application, just cd in the `linux-app` folder and build it with
the SDK.

```sh
cd linux-app
source /opt/poky/2.5.1/environment-setup-armv7vehf-neon-poky-linux-gnueabi
$CC linux-app.c -o linux-app
```

The `$CC` variable is the path of the gcc toolchain of the SDK, so be aware
to use that and not just `gcc`.

And then `scp` the executable to the target and run it.
```sh
scp linux-app root@192.168.0.33:/home/root
```

And on the target:
```sh
cd /home/root
./linux-app
```

Then you'll see that the app will sample the SPI light sensor
updates the I2C PWM LED in a while loop.

```sh
./linux-app -i /dev/i2c-0 -s /dev/spidev0.0 -m 0 -b 5000000 -p 0
== Settings:
        Mode: Fast
        Number of runs: -1
        SPI dev: /dev/spidev0.0
        I2C dev: /dev/i2c-0

Start:

2935
2935
2937
2938
...
```

For more info how to use the `linux-app` tool run this:
```sh
./linux-app -h
```

Just for convenience this is the output of the above command:
```sh
Usage:
./linux-app [-i I2C_DEV] [-s SPI DEV] [-m MODE] [-s SAMPLES]
        -i   : the I2C device/bus that the photoresistor is connected (e.g. /dev/i2c-0)
        -s   : the SPI device/bus that the PWM LED is connected (e.g. /dev/spidev0.0)
        -b   : SPI baudrate (default 1000000)
        -r   : Number of runs/iterations for the SPI/I2C read/write (default -1, run forever)
        -m   : mode
                0: Fast mode (default).
                1: Load mode. Adds some printfs to create dummy load.
                2: Benchmark mode. Tries these SPI speeds: 1Mz, 2MHz, 5MHz, 10MHz, 20MHz, 30MHz
```

## Kernels used
The kernels used for the benchmarks are the following:

```sh
Linux nanopi-neo 4.14.87-allwinner #1 SMP Wed Dec 26 15:26:48 UTC 2018 armv7l GNU/Linux
```

```sh
Linux nanopi-neo 4.14.78-rt47-allwinner #1 SMP PREEMPT RT Mon Jan 21 20:12:29 UTC 2019 armv7l GNU/Linux
```

