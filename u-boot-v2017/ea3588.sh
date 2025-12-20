#!/bin/bash

set -xe

# CROSS_COMPILE_ARM64=~/toolchains/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

GCC=`realpath ../gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu`
CROSS_COMPILE_ARM64=aarch64-linux-gnu-
echo "using gcc: [${CROSS_COMPILE_ARM64}]"

# Clean the old files.
rm -rf spl/u-boot-spl* #tpl/

# Start building (U-Boot, SPL, etc.)
make CROSS_COMPILE=${CROSS_COMPILE_ARM64} rk3588_defconfig
make CROSS_COMPILE=${CROSS_COMPILE_ARM64} -j`nproc`

# Copy files.
cp spl/u-boot-spl.bin ../rkbin/bin/rk35/rk3588_spl_v3.bin

# Call the official build.
./make.sh rk3588

ls -alh fit/uboot.itb

mkdir -p ../rockdev/
cp -a fit/uboot.itb ../rockdev/uboot.img
ls -alh ../rockdev/uboot.img
md5sum ../rockdev/uboot.img
echo "All done! [$?]"

