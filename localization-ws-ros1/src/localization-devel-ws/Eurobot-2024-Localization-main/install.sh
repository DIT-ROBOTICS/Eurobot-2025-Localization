# Install basic package
sudo apt update -y
sudo apt install ros-noetic-costmap-converter \
                 ros-noetic-robot-localization \
                 ros-noetic-imu-tools \
                 libusb-1.0-0 libusb-1.0-0-dev -y

PACKAGES_PATH=/home/localization/localization_ws/src/Eurobot-2024-Localization

# Build LiDAR
rm -rf $PACKAGES_PATH/.YDLidar-SDK/build
mv $PACKAGES_PATH/.YDLidar-SDK $PACKAGES_PATH/YDLidar-SDK
mkdir $PACKAGES_PATH/YDLidar-SDK/build
cd $PACKAGES_PATH/YDLidar-SDK/build
cmake ..
make
sudo make install
mv $PACKAGES_PATH/YDLidar-SDK $PACKAGES_PATH/.YDLidar-SDK

# Get obstacle_detector
cd $PACKAGES_PATH
cd ..
git clone https://github.com/tysik/obstacle_detector.git

# Build libsurvive
cd $PACKAGES_PATH/Vive/libsurvive
sudo apt-get install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake -y
make

# Build localization worksapce
source /opt/ros/noetic/setup.bash
cd $PACKAGES_PATH/../..
catkin_make
source /home/localization/localization_ws/devel/setup.bash

