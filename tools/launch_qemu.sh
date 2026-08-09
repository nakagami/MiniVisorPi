#!/bin/bash

. tools/environment

# Use KVM when available on an aarch64 host: full TCG emulation is
# ~10-50x slower, which makes guest disk I/O (served synchronously by
# the hypervisor per request) take minutes for e.g. `pip install`'s
# Python startup imports. Note that the hypervisor itself runs at EL2,
# so the host kernel also needs aarch64 nested-virtualization support
# for this to work; if KVM is unusable, QEMU falls back to TCG here.
if [ -e /dev/kvm ] && [ "$(uname -m)" = "aarch64" ]; then
    ACCEL="-accel kvm"
    CPU=host
else
    ACCEL=""
    CPU=cortex-a53
fi

$QEMU   -M virt,gic-version=2,secure=off,virtualization=on \
        -smp 4 -bios $BIN_DIR/u-boot.bin $ACCEL -cpu $CPU -m 4G \
        -nographic -device virtio-blk-device,drive=disk \
        -drive file=$DISK_IMG,format=raw,if=none,media=disk,id=disk \
        -netdev user,id=net0 \
        -device virtio-net-device,netdev=net0
