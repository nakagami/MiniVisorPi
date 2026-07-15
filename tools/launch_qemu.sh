#!/bin/bash

. tools/environment

$QEMU   -M virt,gic-version=2,secure=off,virtualization=on \
        -smp 4 -bios $BIN_DIR/u-boot.bin -cpu cortex-a53 -m 2G \
        -nographic -device virtio-blk-device,drive=disk \
        -drive file=$DISK_IMG,format=raw,if=none,media=disk,id=disk \
        -netdev user,id=net0 \
        -device virtio-net-device,netdev=net0
