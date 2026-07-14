#!/bin/bash

. tools/environment

$QEMU \
  -M virt,gic-version=2,secure=off,virtualization=on,dumpdtb=$BIN_DIR/qemu.dtb \
  -smp 4 -bios $BIN_DIR/u-boot.bin -cpu cortex-a53 -m 2G \
  -nographic -device virtio-blk-device,drive=disk \
  -drive file=fat:rw:$DISK_IMG_DIR,format=raw,if=none,media=disk,id=disk \
  -netdev socket,id=net0,udp=127.0.0.1:10000,localaddr=127.0.0.1:10000 \
  -device virtio-net-device,netdev=net0

dtc -I dtb -O dts -o $BIN_DIR/qemu.dts $BIN_DIR/qemu.dtb
