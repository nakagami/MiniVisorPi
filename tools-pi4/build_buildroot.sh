#!/bin/bash

. tools-pi4/environment

VERSION="2026.05.1"

rm -rf $BUILDROOT_DIR
mkdir -p $BUILDROOT_DIR

pushd $BUILDROOT_DIR
curl https://buildroot.org/downloads/buildroot-$VERSION.tar.xz | tar xvJf -
pushd buildroot-$VERSION

export FORCE_UNSAFE_CONFIGURE=1 # For docker
if [ "`echo $PATH | grep ' '`" ]; then
    export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" # For WSL
fi

# raspberrypi4_64_defconfig targets real Raspberry Pi 4 hardware, unlike
# qemu_aarch64_virt_defconfig used for the QEMU environment in
# tools/build_buildroot.sh.
make raspberrypi4_64_defconfig

# raspberrypi4_64_defconfig's kernel config has no virtio support (real Pi4
# hardware has no virtio bus), but this kernel actually runs as MiniVisorPi's
# guest, which always exposes storage/network via virtio-mmio (see
# scripts/virt.dts). Merge in the virtio options so the guest kernel can
# find /dev/vda; without this it panics at root-mount time.
./utils/config --set-str BR2_LINUX_KERNEL_CONFIG_FRAGMENT_FILES "$BASE_DIR/tools-pi4/linux-virtio.fragment"
make olddefconfig

make -j$(nproc) || exit $?

cp output/images/Image $DISK_IMG_DIR/Image
cp output/images/rootfs.ext2 $DISK_IMG_DIR/DISK0

popd
popd
rm -rf $BUILDROOT_DIR
