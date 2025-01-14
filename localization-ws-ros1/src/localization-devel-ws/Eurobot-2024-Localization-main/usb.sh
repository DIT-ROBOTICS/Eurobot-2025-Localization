# USB driver setup

PACKAGES_PATH=/home/localization/localization_ws/src/Eurobot-2024-Localization

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

sudo /lib/systemd/systemd-udevd --daemon
cd $PACKAGES_PATH
cd $PACKAGES_PATH/local_filter/imu/phidgets_drivers/phidgets_api
sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules

cd $PACKAGES_PATH/Vive/libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger