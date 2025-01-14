# Eurobot-2024-Localization

> Eurobot localization workspace for 2024

- elaborate README allow a team to grow!
- clear commit messages are benefitial for all!

## Install

> with docker

```bash
# Move to your ws
mkdir devel
mkdir build
mkdir src
cd src
git clone git@github.com:DIT-ROBOTICS/Eurobot-2024-Localization.git
git clone git@github.com:DIT-ROBOTICS/Eurobot-2024-Localization-envs.git
git clone git@github.com:DIT-ROBOTICS/eurobot_msgs.git
```

### Build

```bash
sudo docker compose -f /home/localization/Eurobot-2024/src/Eurobot-2024-Localization-envs/compose-build.yaml up --build
```

### Run

```bash
sudo docker compose -f /home/localization/Eurobot-2024/src/Eurobot-2024-Localization-envs/compose-run.yaml up
```

#### send ready signal to start

> Publishing a ready signal is necessary for determining Blue/Yellow side.
> Without that, localization program will not start to work.

Blue
```bash
rostopic pub /robot/startup/ready_signal geometry_msgs/PointStamped "{header: {frame_id: '0'}, point: {x: 2.7, y: 1.0, z: 180.0}}" -1
```
Yellow
```bash
rostopic pub /robot/startup/ready_signal geometry_msgs/PointStamped "{header: {frame_id: '1'}, point: {x: 0.3, y: 1.0, z: 0.0}}" -1
```

> Modify the value in `point` if it's necessary.
> z: in degree

### Develop

```bash
cd Eurobot-2024-Localization-envs
docker compose -f docker-compose.yaml up -d
docker exec -it localization bash
# in the container
source Eurobot-2024-Localization/rosenv.sh
```

## Structure

```
.
└──  Your Workspace
     └── build
     └── devel
     └── src
         ├── Eurobot-2024-Localization
         │   ├── .YDLidar-SDK
         │   ├── eurobot_localization
         │   ├── rival_localization
         │   ├── lidar
         │   │   ├── lidar_localization
         │   │   └── ydlidar_ros_driver
         │   ├── local_filter
         │   │   ├── imu
         │   │   │   ├── imu_drive
         │   │   │   └── phidgets_drivers
         │   │   ├── local_filter
         │   │   └── odometry
         │   │       ├── odometry
         │   │       ├── rosserial_msgs
         │   │       └── rosserial_server
         │   ├── simulation
         │   └── vive
         └── Eurobot-2024-Localization-envs

```


## Architecture
> Local filter ( IMU + Odometry ) + global filter ( LiDAR )

### Local filter (unupdated)

- Place all of the required component in local filter
- Run with rosserial and imu firmmware
```bash=1
roslaunch local_filter local_filter.launch
```
- Run without rosserial but imu firmware
```bash=1
roslaunch local_filter local_filter_no_comm.launch # no such file now!
```
- Run without rosserial and imu firmware
```bash=1
roslaunch local_filter local_filter_no_firmware.launch # no such file now!
```

### Global filter (unupdated)

- Place LiDAR driver and triangle localization in lidar
- Place global filter in eurobot_localization
- Run with only triangle localization
```bash=1
roslaunch lidar_localization lidar_localization_2023.launch # no such file now!
```
- Run with only lidar driver and triangle localization
```bash=1
roslaunch lidar_localization lidar_with_driver.launch
```
- Run global filter with lidar driver
```bash=1
roslaunch eurobot_localization global_ekf.launch
```
- Run global filter without lidar driver
```bash=1
roslaunch eurobot_localization global_ekf_without_lidar.launch # no such file now!
```

### Together

- Run all of the localization component
```bash=1
roslaunch eurobot_localization eurobot_localization.launch
```

### Local machine setup configure

- Port name
- Static TF for laser frame and imu frame
