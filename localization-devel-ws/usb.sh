#!/bin/bash

# IMU port

# List all USB devices and find the IMU device
IMU_DEVICE=$(lsusb | grep -i "06c2:008d")  # Phidgets Inc. (formerly GLAB) USB2.1 Hub

if [ -z "$IMU_DEVICE" ]; then
  echo "IMU device not found."
else
  # Extract the bus and device numbers
  BUS=$(echo $IMU_DEVICE | awk '{print $2}')
  DEVICE=$(echo $IMU_DEVICE | awk '{print $4}' | sed 's/://')

  # Construct the device path
  DEVICE_PATH="/dev/bus/usb/$BUS/$DEVICE"

  # Check the current permissions of the device
  echo "Current permissions for $DEVICE_PATH:"
  ls -l $DEVICE_PATH

  # Change the permissions to read and write for all users
  sudo chmod 666 $DEVICE_PATH

  # Verify the permissions have been changed
  echo "Updated permissions for $DEVICE_PATH:"
  ls -l $DEVICE_PATH

  # IMU udev rules
  PACKAGES_PATH=/home/user/localization-ws-ros1/src
  sudo /lib/systemd/systemd-udevd --daemon
  cd $PACKAGES_PATH/phidgets_drivers/phidgets_api
  sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
fi

# RPLiDAR port

# Get the device name from dmesg
DEVICE_NAME=$(sudo dmesg | grep -oP 'cp210x converter now attached to \KttyUSB[0-9]+' | tail -1)

# Validate the DEVICE_NAME to ensure it matches the expected pattern
if [[ "$DEVICE_NAME" =~ ^ttyUSB[0-9]+$ ]]; then
  DEVICE_PATH="/dev/$DEVICE_NAME"

  # Change the permissions to read and write for all users
  sudo chmod 666 $DEVICE_PATH

  # Verify the permissions have been changed
  echo "Updated permissions for $DEVICE_PATH:"
  ls -l $DEVICE_PATH
else
  echo "Invalid device name: $DEVICE_NAME"
fi