#!/bin/bash

. tools-pi4/environment

# Loads mini.elf from the FAT boot partition of either the SD card (mmc --
# the "emmc2" controller that drives the physical SD card slot on RPi4) or a
# USB mass-storage device (behind the VL805 xHCI controller), instead of the
# virtio-blk device used by the QEMU environment's scripts/boot.txt.
$U_BOOT_DIR/tools/mkimage -A arm64 -T script -C none -d scripts/boot-pi4.txt $DISK_IMG_DIR/boot.scr
