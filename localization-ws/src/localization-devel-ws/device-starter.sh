# source /opt/ros/humble/setup.bash
# colcon build
PACKAGES_PATH=/home/localization_ws/src
sudo /lib/systemd/systemd-udevd --daemon
cd $PACKAGES_PATH
cd $PACKAGES_PATH/local/imu/phidgets_drivers/phidgets_api
sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
# source /install/setup.bash
# ros2 launch phidgets_spatial spatial-launch.py 
sudo chmod 777 /dev/ttyUSB0