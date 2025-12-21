#!/bin/bash

set -xe

cp -a ea3588s_defconfig ./arch/arm64/configs/ea3588s_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- ea3588s_defconfig

dtc -I dts -O dtb ea3588s.dts -o ea3588s.dtb
KVER=`make kernelrelease`
mkdir -p ../rockdev/dtb
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=`pwd`/../rockdev modules_install
cp -f ea3588s.dtb ../rockdev/dtb
cp -f arch/arm64/boot/Image ../rockdev/vmlinuz-${KVER}
cp -f .config ../rockdev/config-${KVER}
cp -f System.map ../rockdev/System.map-${KVER}

dd if=/dev/zero of=../rockdev/boot.img bs=1M count=60
mkfs.ext2 -U 7A3F0000-0000-446A-8000-702F00006273 -L kdevboot ../rockdev/boot.img
mount ../rockdev/boot.img /mnt
mkdir -p /mnt/dtb
cp -f ea3588s.dtb /mnt/dtb/
cp -f ../rockdev/vmlinuz-${KVER} /mnt/
cp -f ../rockdev/config-${KVER} /mnt/
cp -f ../rockdev/System.map-${KVER} /mnt/
touch /mnt/initrd.img-${KVER}
mkdir -p /mnt/extlinux/
cp -f extlinux.conf /mnt/extlinux/
cp -f extlinux.conf /mnt/
find /mnt
sync
umount /mnt

echo "All done!"

