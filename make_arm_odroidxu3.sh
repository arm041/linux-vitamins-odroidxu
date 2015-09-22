#!/bin/bash
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- odroidxu3_defconfig
make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabi-

