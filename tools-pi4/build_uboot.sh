#!/bin/bash

. tools-pi4/environment

if [ -z $CROSS_COMPILE ]; then
    export CROSS_COMPILE=aarch64-linux-gnu-
fi

rm -rf $U_BOOT_DIR
git clone --depth=1 -b v2024.04 https://github.com/u-boot/u-boot.git $U_BOOT_DIR

pushd $U_BOOT_DIR
# rpi_arm64_defconfig targets real Raspberry Pi 3/4 hardware in AArch64 mode
# (CONFIG_DEFAULT_DEVICE_TREE=bcm2711-rpi-4-b), unlike qemu_arm64_defconfig
# used for the QEMU environment in tools/build_uboot.sh.
make rpi_arm64_defconfig
make -j$(nproc)
cp u-boot.bin $BIN_DIR/u-boot.bin
popd
