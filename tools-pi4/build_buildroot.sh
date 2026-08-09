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

# Real Pi4 hardware may boot with no Ethernet cable plugged in. The default
# S40network script runs `ifup -a` (and thus `udhcpc`) synchronously, which
# retries DHCPDISCOVER forever and blocks the rest of boot (including the
# login prompt) when there is no link. Override it to background `ifup -a`
# so boot always proceeds regardless of cable/link state.
./utils/config --set-str BR2_ROOTFS_OVERLAY "$BASE_DIR/tools-pi4/rootfs-overlay"

# Python 3 + pip for the guest rootfs. The default 64M ext2 image
# is too small once Python is included, so enlarge the rootfs to 256M.
./utils/config --enable BR2_PACKAGE_PYTHON3
./utils/config --enable BR2_PACKAGE_PYTHON_PIP
./utils/config --set-str BR2_TARGET_ROOTFS_EXT2_SIZE "256M"
make olddefconfig

make -j$(nproc) || exit $?

cp output/images/Image $DISK_IMG_DIR/Image
cp output/images/rootfs.ext2 $DISK_IMG_DIR/DISK0

popd
popd
rm -rf $BUILDROOT_DIR
