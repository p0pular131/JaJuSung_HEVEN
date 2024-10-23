#!/bin/bash
sudo modprobe pcan
echo "sudo modprobe pcan"
sleep 1

sudo modprobe peak_usb
echo "sudo modprobe peak_usb"
lsmod | grep peak_usb
sleep 1

sudo modprobe can_raw
echo "sudo modprobe can_raw"
lsmod | grep can_raw
sleep 1

dmesg | grep -i pcan
sleep 1

echo "CAN setting Done"
