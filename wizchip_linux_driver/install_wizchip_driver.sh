#!/bin/bash

KERNEL_VER=$(uname -r)
DRIVER_DST_DIR="/lib/modules/$KERNEL_VER/kernel/drivers/net/ethernet/wiznet"
echo "======================================================"
echo "Building W5500 driver for kernel version: $KERNEL_VER"
echo "Driver destination directory: $DRIVER_DST_DIR"
echo "======================================================"
echo "script started "

# 드라이버 디렉토리가 없다면 생성
sudo mkdir -p "$DRIVER_DST_DIR"


make || { echo "Make failed"; exit 1; }

# make 

modinfo ./output/*ko

sudo depmod -a

#sudo cp ./*ko /lib/modules/6.6.51+rpt-rpi-v8/kernel/drivers/net/ethernet/wiznet/
sudo cp ./output/*.ko "$DRIVER_DST_DIR" || { echo "Copy failed"; exit 1; }

sudo depmod -a

sudo modprobe -r wizchip_spi wizchip
sudo modprobe wizchip
sudo modprobe wizchip_spi

lsmod |grep wizchip

#dmesg
