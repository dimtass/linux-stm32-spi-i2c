SUMMARY = "Allwinner console image with I2C and SPI tools"
LICENSE = "MIT"
AUTHOR = "Dimitris Tassopoulos <dimtass@gmail.com>"

inherit allwinner-base-image

# Add our custom tools
IMAGE_INSTALL += " \
    i2c-tools \
    spitools \
"