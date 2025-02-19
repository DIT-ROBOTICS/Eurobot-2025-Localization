export ROS_HOSTNAME="192.168.50.11"
export ROS_MASTER_URI="http://192.168.50.11:11311"
source /opt/ros/noetic/setup.bash
source /home/localization/localization_ws/devel/setup.bash

sudo chown localization:localization ~/.gazebo