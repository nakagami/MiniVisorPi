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

# Rootfs overlays. tools/rootfs-overlay carries /etc/init.d/S45ntptime,
# which sets the clock via NTP (busybox ntpd) in the background -- the
# guest has no RTC and would otherwise sit at the epoch, breaking TLS
# (pip install fails with "certificate is not yet valid"). Real Pi4
# hardware may boot with no Ethernet cable plugged in, so the sync must
# never block boot.
./utils/config --set-str BR2_ROOTFS_OVERLAY "$BASE_DIR/tools-pi4/rootfs-overlay $BASE_DIR/tools/rootfs-overlay"
./utils/config --set-str BR2_PACKAGE_BUSYBOX_CONFIG_FRAGMENT_FILES "$BASE_DIR/tools/busybox-ntp.fragment"

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
