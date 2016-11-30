#!/bin/bash

adb wait-for-device
echo "reboot fastboot ..."
adb shell reboot fastboot
echo "flash zImage-dtb ..."
fastboot flash kernel arch/arm/boot/zImage-dtb
echo "reboot ..."
fastboot reboot

