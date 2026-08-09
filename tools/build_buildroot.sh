#!/bin/bash

. tools/environment

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

make qemu_aarch64_virt_defconfig
sed -i -e 's/BR2_PACKAGE_HOST_QEMU=y/BR2_PACKAGE_HOST_QEMU=n/' .config

# Python 3 + pip for the guest rootfs. The default 64M ext2 image
# is too small once Python is included, so enlarge the rootfs to 256M.
./utils/config --enable BR2_PACKAGE_PYTHON3
./utils/config --enable BR2_PACKAGE_PYTHON_PIP

# NTP clock sync at boot (busybox ntpd via a config fragment + the
# rootfs overlay's /etc/init.d/S45ntptime). The guest has no RTC and
# would otherwise sit at the epoch, breaking TLS (pip install fails
# with "certificate is not yet valid").
./utils/config --set-str BR2_PACKAGE_BUSYBOX_CONFIG_FRAGMENT_FILES "$BASE_DIR/tools/busybox-ntp.fragment"
./utils/config --set-str BR2_ROOTFS_OVERLAY "$BASE_DIR/tools/rootfs-overlay"
./utils/config --set-str BR2_TARGET_ROOTFS_EXT2_SIZE "256M"
make olddefconfig

make -j$(nproc) || exit $?

cp output/images/Image $DISK_IMG_DIR/Image
cp output/images/rootfs.ext2 $DISK_IMG_DIR/DISK0

popd
popd
rm -rf $BUILDROOT_DIR
