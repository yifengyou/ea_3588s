#!/bin/bash

set -x

export GCC="/usr/"
export CROSS_COMPILE_ARM64="${GCC}/bin/aarch64-linux-gnu-"
echo "using gcc: [${CROSS_COMPILE_ARM64}]"

rm -rf spl/u-boot-spl*

make CROSS_COMPILE=${CROSS_COMPILE_ARM64} radxa5b_defconfig
#make CROSS_COMPILE=${CROSS_COMPILE_ARM64} rk3588_defconfig
make CROSS_COMPILE=${CROSS_COMPILE_ARM64} -j`nproc`

cp spl/u-boot-spl.bin ../rkbin/bin/rk35/rk3588_spl_v3.bin
./make.sh rk3588

cp -a fit/uboot.itb uboot.img
md5sum uboot.img
mkdir -p ../rockdev/
mv uboot.img ../rockdev/

echo "All done! [$?]"

