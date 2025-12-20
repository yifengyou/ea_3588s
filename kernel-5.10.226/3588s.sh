#!/bin/bash

set -x
JOB=`sed -n "N;/processor/p" /proc/cpuinfo|wc -l`

ARCH=`uname -m`
export KERNEL_TARGET=ea3588s

if [ X"${ARCH}" == X"aarch64" ] ; then
	GCC=""
	CROSS_COMPILE_ARM64=""
elif [ X"${ARCH}" == X"x86_64" ] ; then
	CROSS_COMPILE_ARM64=aarch64-linux-gnu-
	echo "using gcc: [${CROSS_COMPILE_ARM64}]"
else
	echo "${ARCH} is not supported now!"
	exit 1
fi


# clean
# make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 mrproper

# kernel
if [ -f .config ] ; then
	cp -a .config .config-bak
fi
make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 ${KERNEL_TARGET}_defconfig
if [ $? -ne 0 ] ; then
	echo "config failed!"
	exit 1
fi

if [ -f .config-bak ] ; then
	diff .config .config-bak
	if [ $? -eq 0 ] ; then
		cp -a .config-bak .config
	fi
fi

set -e
make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 -j$JOB
# make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 dtbs -j$JOB

mkdir -p ../rockdev/modules
cp arch/arm64/boot/Image ../rockdev/
find . -name "*.ko" |xargs -i /bin/cp -a {} ../rockdev/modules/

ls -alh ../rockdev/modules/
md5sum ../rockdev/modules/*.ko

echo "All done! [$?]"

