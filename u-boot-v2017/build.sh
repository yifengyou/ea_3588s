#!/bin/bash

CROSS_COMPILE_ARM64=~/toolchains/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

# Clean the old files.
rm -rf spl/u-boot-spl* #tpl/

# Start building (U-Boot, SPL, etc.)
make CROSS_COMPILE=${CROSS_COMPILE_ARM64} rk3588_defconfig
make CROSS_COMPILE=${CROSS_COMPILE_ARM64}

# Copy files.
cp spl/u-boot-spl.bin ../rkbin/bin/rk35/rk3588_spl_v3.bin

# Call the official build.
./make.sh rk3588
