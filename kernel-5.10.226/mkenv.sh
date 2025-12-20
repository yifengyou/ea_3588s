#!/bin/bash
# 
# Copyright (c) 2022 Shenzhen Lztek Co., Ltd
#
# SPDX-License-Identifier: GPL-2.0
#

TOP_DIR=`realpath ..`
GCC_DIR=$TOP_DIR/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

alias mkcc='make CROSS_COMPILE=$GCC_DIR'
