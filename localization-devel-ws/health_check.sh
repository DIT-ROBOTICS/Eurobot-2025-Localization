#!/bin/bash

# Function to handle termination signals
cleanup() {
    echo "Terminating script..."
    exit 0
}

# Trap termination signals
trap cleanup SIGINT SIGTERM

while true; do
    # local_filer is the most important topic
    topic="/local_filter"
    output=$(timeout 2s ros2 topic hz "$topic" 2>&1)
    if [[ "$output" == *"WARNING: topic [$topic] does not appear to be published yet"* ]]; then
        echo "WARNING: topic [$topic] does not appear to be published yet. Checking sensors..."
        # check wheel sensor
        topic="/driving_duaiduaiduai"
        output=$(timeout 2s ros2 topic hz "$topic" 2>&1)
        if [[ "$output" == *"WARNING: topic [$topic] does not appear to be published yet"* ]]; then
            echo "WARNING: topic [$topic] does not appear to be published yet. Please re-plug the wheel sensor."
        fi
        # check imu
        topic="/imu/data_raw"
        output=$(timeout 2s ros2 topic hz "$topic" 2>&1)
        if [[ "$output" == *"WARNING: topic [$topic] does not appear to be published yet"* ]]; then
            echo "WARNING: topic [$topic] does not appear to be published yet. Please re-plug the imu sensor, or check with 'lsusb'"
        fi
    fi

    # check /lidar_pose, if it is not published for 5 seconds, prompt the user to check whether the initial pose is right, or check LiDAR connection
    # use topic echo, if any topic comes in, it's fine, if there's no topic, give warning
    topic="/lidar_pose"
    output=$(timeout 5s ros2 topic echo "$topic" 2>&1)
    if [[ "$output" == *"data"* ]]; then
        echo "LiDAR is working fine."
        else echo "WARNING: topic [$topic] has no new msg. Please check the initial pose or LiDAR connection."
    fi
    sleep 5
done