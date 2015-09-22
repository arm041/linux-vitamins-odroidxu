#!/bin/bash

sudo adb wait-for-device
echo "reboot fastboot ..."
sudo adb shell reboot fastboot
echo "flash zImage-dtb ..."
sudo fastboot flash kernel arch/arm/boot/zImage-dtb
echo "reboot ..."
sudo fastboot reboot

