#!/bin/bash

set -xe

# set config
cp -a ea3588s_defconfig ./arch/arm64/configs/ea3588s_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- ea3588s_defconfig

# build dtb
dtc -I dts -O dtb ea3588s.dts -o ea3588s.dtb

# build kernel
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j`nproc`

# build modules
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules -j`nproc`

# install modules
mkdir -p ../rockdev/modules
find . -name "*.ko" |xargs -i cp {} ../rockdev/modules/

KVER=`make kernelrelease`

dd if=/dev/zero of=../rockdev/boot.img bs=1M count=60
mkfs.ext2 -U 7A3F0000-0000-446A-8000-702F00006273 -L kdevboot ../rockdev/boot.img
mount ../rockdev/boot.img /mnt
mkdir -p /mnt/dtb

cp -f ea3588s.dtb /mnt/dtb
cp -f arch/arm64/boot/Image /mnt/vmlinuz-${KVER}
cp -f .config /mnt/config-${KVER}
cp -f System.map /mnt/System.map-${KVER}
touch /mnt/initrd.img-${KVER}

mkdir -p /mnt/extlinux/
cp -f extlinux.conf /mnt/extlinux/
cp -f extlinux.conf /mnt/
find /mnt
sync
umount /mnt

echo "All done!"

