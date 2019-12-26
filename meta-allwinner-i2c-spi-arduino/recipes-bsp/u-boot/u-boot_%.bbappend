SUMMERY = "Override default boot env and load spidev and i2c0 overlays"
AUTHOR = "Dimitris Tassopoulos <dimtass@gmail.com>"

FILESEXTRAPATHS_prepend := "${THISDIR}/linux_${LINUX_VERSION}:${THISDIR}/files:"
SRC_URI += "file://allwinnerEnv.txt"