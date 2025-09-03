#!/bin/bash
dtc -I dts -O dtb -o w5500-driver.dtbo wizchip-driver.dts
#sudo cp w5500-driver.dtbo /boot/firmware/overlays/
#dmesg
