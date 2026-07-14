#!/bin/bash

. tools-pi4/environment

cp $1 $DISK_IMG_DIR$BINARY_NAME
echo "mini.elf copied to $DISK_IMG_DIR$BINARY_NAME"
echo "Run tools-pi4/create_sdcard.sh $1 to build a full bootable SD card image."
