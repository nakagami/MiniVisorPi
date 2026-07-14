#!/bin/bash

. tools-pi4/environment

cp $1 $DISK_IMG_DIR$BINARY_NAME

tools-pi4/create_boot_scr.sh
cp $U_BOOT_DIR/arch/arm/dts/bcm2711-rpi-4-b.dtb $DISK_IMG_DIR/DTB
tools-pi4/create_disk.sh

echo "Disk image ready at $DISK_IMG"
echo "Write it to the physical SD card with, e.g.:"
echo "  sudo dd if=$DISK_IMG of=/dev/sdX bs=4M status=progress conv=fsync"
