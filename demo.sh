#!/bin/bash

export ARCH=mips
export CROSS_COMPILE=mipsel-linux-

make evb_gsc329x_defconfig
make -j6 -s
