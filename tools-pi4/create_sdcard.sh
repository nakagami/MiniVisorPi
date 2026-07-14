#!/bin/bash

. tools-pi4/environment

cp $1 $DISK_IMG_DIR$BINARY_NAME

# u-boot.bin is loaded as the "kernel" by the Raspberry Pi GPU firmware
# (see config.txt's kernel= entry below), and bcm2711-rpi-4-b.dtb is loaded
# by that same firmware and passed to u-boot via $fdt_addr -- unlike the
# QEMU environment, where -bios/-dumpdtb supply these directly.
cp $BIN_DIR/u-boot.bin $DISK_IMG_DIR/u-boot.bin
cp $U_BOOT_DIR/arch/arm/dts/bcm2711-rpi-4-b.dtb $DISK_IMG_DIR/bcm2711-rpi-4-b.dtb

tools-pi4/create_boot_scr.sh

cat > $DISK_IMG_DIR/config.txt <<'EOF'
# Raspberry Pi 4 boot configuration for MiniVisor
arm_64bit=1
enable_uart=1
kernel=u-boot.bin
device_tree=bcm2711-rpi-4-b.dtb
EOF

tools-pi4/create_disk.sh

echo "Disk image ready at $DISK_IMG"
echo
echo "NOTE: MiniVisorPi does not redistribute Raspberry Pi's official GPU"
echo "firmware. Before the board will boot, also copy start4.elf and"
echo "fixup4.dat (from https://github.com/raspberrypi/firmware/tree/master/boot,"
echo "or /boot/firmware on a Raspberry Pi OS install) into $DISK_IMG_DIR"
echo "before running this script, or directly onto the SD card's boot"
echo "partition afterwards."
echo
echo "Write the image to the physical SD card with, e.g.:"
echo "  sudo dd if=$DISK_IMG of=/dev/sdX bs=4M status=progress conv=fsync"
