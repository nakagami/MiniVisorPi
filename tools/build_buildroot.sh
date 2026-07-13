#!/bin/bash

. tools/environment

VERSION="2025.05"

rm -rf $BUILDROOT_DIR
mkdir -p $BUILDROOT_DIR

pushd $BUILDROOT_DIR
curl https://buildroot.org/downloads/buildroot-$VERSION.tar.xz | tar xvJf -
pushd buildroot-$VERSION

# Bump host-m4 to 1.4.21 to fix build failure on hosts with glibc 2.43,
# where bsearch/memchr are defined as _Generic macros that conflict with
# the gnulib headers bundled in m4 1.4.20.
sed -i 's/^M4_VERSION = 1.4.20$/M4_VERSION = 1.4.21/' package/m4/m4.mk
sed -i \
  -e 's/^sha256  e236ea3a1ccf5f6c270b1c4bb60726f371fa49459a8eaaebc90b216b328daf2b  m4-1.4.20.tar.xz$/sha256  f25c6ab51548a73a75558742fb031e0625d6485fe5f9155949d6486a2408ab66  m4-1.4.21.tar.xz/' \
  package/m4/m4.hash

export FORCE_UNSAFE_CONFIGURE=1 # For docker
if [ "`echo $PATH | grep ' '`" ]; then
    export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" # For WSL
fi

make qemu_aarch64_virt_defconfig
sed -i -e 's/BR2_PACKAGE_HOST_QEMU=y/BR2_PACKAGE_HOST_QEMU=n/' .config
make -j$(nproc) || exit $?

cp output/images/Image $DISK_IMG_DIR/Image
cp output/images/rootfs.ext2 $DISK_IMG_DIR/DISK0

popd
popd
rm -rf $BUILDROOT_DIR
