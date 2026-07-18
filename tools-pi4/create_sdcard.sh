#!/bin/bash

. tools-pi4/environment

if [ ! -f $DISK_IMG_DIR$BINARY_NAME ]; then
    echo "$DISK_IMG_DIR$BINARY_NAME not found. Run 'cargo build-pi4' first." >&2
    exit 1
fi

# u-boot.bin is loaded as the "kernel" by the Raspberry Pi GPU firmware
# (see config.txt's kernel= entry below). bcm2711-rpi-4-b.dtb is loaded by
# that same firmware and passed to u-boot via $fdt_addr -- unlike the QEMU
# environment, where -bios/-dumpdtb supply these directly. This must be
# Raspberry Pi's official firmware devicetree (placed into $DISK_IMG_DIR by
# the user, see the NOTE below), NOT u-boot's own bundled copy under
# $U_BOOT_DIR/arch/arm/dts: u-boot's copy lacks the "bt"/"uart0_pins"/
# "bt_pins" __symbols__ labels that dtoverlay=disable-bt below needs to
# resolve its fixups, so the overlay silently fails to apply against it.
cp $BIN_DIR/u-boot.bin $DISK_IMG_DIR/u-boot.bin
if [ ! -f $DISK_IMG_DIR/bcm2711-rpi-4-b.dtb ]; then
    echo "${DISK_IMG_DIR}bcm2711-rpi-4-b.dtb not found. Fetch Raspberry Pi's" >&2
    echo "official firmware devicetree first, e.g.:" >&2
    echo "  curl -L -o ${DISK_IMG_DIR}bcm2711-rpi-4-b.dtb https://github.com/raspberrypi/firmware/raw/master/boot/bcm2711-rpi-4-b.dtb" >&2
    exit 1
fi

tools-pi4/create_boot_scr.sh

cat > $DISK_IMG_DIR/config.txt <<'EOF'
# Raspberry Pi 4 boot configuration for MiniVisor
arm_64bit=1
enable_uart=1
kernel=u-boot.bin
device_tree=bcm2711-rpi-4-b.dtb
# Work around a known Raspberry Pi 4 EMMC2 (SD card) controller issue where
# UHS voltage switching can fail on some boards, causing the SD card to be
# undetected ("Card did not respond to voltage select! : -110") in u-boot.
disable_emmc2_lowvoltage=1
# By default, the RPi4's only PL011 UART (uart0) is wired to the onboard
# Bluetooth module, while the GPIO14/15 header pins (where a USB-serial
# console is normally attached) carry the mini/AUX UART (uart1) instead.
# MiniVisor's serial driver only supports PL011, so without this overlay
# it silently prints to the (unconnected) Bluetooth UART. disable-bt remaps
# PL011 onto GPIO14/15 and disables Bluetooth, matching what MiniVisor and
# u-boot (whose console follows the devicetree's /chosen stdout-path) expect.
dtoverlay=disable-bt
EOF

tools-pi4/create_disk.sh

echo "Disk image ready at $DISK_IMG"
echo
echo "NOTE: MiniVisorPi does not redistribute Raspberry Pi's official GPU"
echo "firmware or devicetree. Before running this script, fetch and place:"
echo "  - start4.elf, fixup4.dat, and bcm2711-rpi-4-b.dtb into $DISK_IMG_DIR"
echo "  - overlays/disable-bt.dtbo into \${DISK_IMG_DIR}overlays/"
echo "from https://github.com/raspberrypi/firmware/tree/master/boot (or"
echo "/boot/firmware on a Raspberry Pi OS install), e.g.:"
echo "  curl -L -o ${DISK_IMG_DIR}start4.elf https://github.com/raspberrypi/firmware/raw/master/boot/start4.elf"
echo "  curl -L -o ${DISK_IMG_DIR}fixup4.dat https://github.com/raspberrypi/firmware/raw/master/boot/fixup4.dat"
echo "  curl -L -o ${DISK_IMG_DIR}bcm2711-rpi-4-b.dtb https://github.com/raspberrypi/firmware/raw/master/boot/bcm2711-rpi-4-b.dtb"
echo "  mkdir -p ${DISK_IMG_DIR}overlays"
echo "  curl -L -o ${DISK_IMG_DIR}overlays/disable-bt.dtbo https://github.com/raspberrypi/firmware/raw/master/boot/overlays/disable-bt.dtbo"
echo
echo "Write the image to the physical SD card with, e.g.:"
echo "  sudo dd if=$DISK_IMG of=/dev/sdX bs=4M status=progress conv=fsync"
