#!/bin/bash

set -xe


cp -a ea3588s_defconfig ./arch/arm64/configs/ea3588s_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- ea3588s_defconfig

dtc -I dts -O dtb ea3588s.dts -o ea3588s.dtb
KVER=`make kernelrelease`
mkdir -p output/dtb
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=`pwd`/output modules_install
cp -f ea3588s.dtb output/dtb
cp -f arch/arm64/boot/Image output/vmlinuz-${KVER}
cp -f .config output/config-${KVER}
cp -f System.map output/System.map-${KVER}

dd if=/dev/zero of=output/boot.img bs=1M count=60
mkfs.ext2 -U 7A3F0000-0000-446A-8000-702F00006273 -L kdevboot output/boot.img
mount output/boot.img /mnt
mkdir -p /mnt/dtb
cp -f ea3588s.dtb /mnt/dtb/
cp -f output/vmlinuz-${KVER} /mnt/
cp -f output/config-${KVER} /mnt/
cp -f output/System.map-${KVER} /mnt/
touch /mnt/initrd.img-${KVER}
mkdir -p /mnt/extlinux/
cp -f extlinux.conf /mnt/extlinux/
cp -f extlinux.conf /mnt/
find /mnt
sync
umount /mnt

echo "All done!"

