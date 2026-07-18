#!/bin/bash

. tools-pi4/environment

# Loads mini.elf from the SD card's FAT boot partition (mmc 1:1 -- the
# "emmc2" controller that drives the physical SD card slot on RPi4) instead
# of the virtio-blk device used by the QEMU environment's scripts/boot.txt.
$U_BOOT_DIR/tools/mkimage -A arm64 -T script -C none -d scripts/boot-pi4.txt $DISK_IMG_DIR/boot.scr
