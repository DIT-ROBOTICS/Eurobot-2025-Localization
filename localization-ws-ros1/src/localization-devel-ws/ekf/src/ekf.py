#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from math import sin, cos, atan2
import tf

class EKFFootprintBroadcaster:
    def __init__(self):
        rospy.init_node('ekf_footprint_broadcaster')
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/robot/lidar_bonbonbon', PoseWithCovarianceStamped, self.gps_callback)
        rospy.Subscriber('/camera', Odometry, self.camera_callback)
        rospy.Subscriber('/robot/local_filter', Odometry, self.odom_callback)

        self.X = np.array([0.0, 0.0, 0.0])  # Initial state

        # Covariance matrix
        self.P = np.eye(3) * 1e-3
        # Noise covariance
        self.Q = np.eye(3) * 1e-3  # Process noise
        self.R_gps = np.eye(3) * 1e-2  # GPS measurement noise
        self.R_camera = np.eye(3) * 1e-2  # Camera measurement noise

        self.last_odom = rospy.Time.now().to_sec()

        self.ekf_pose_topic = rospy.get_param('~ekf_pose_topic', 'ekf_pose')
        self.parent_frame_id = rospy.get_param('~robot_parent_frame_id', 'robot/map')
        self.child_frame_id = rospy.get_param('~robot_frame_id', 'robot/base_footprint')
        self.rate = rospy.Rate(rospy.get_param('~update_rate', 100))  # based on the update rate of the odometry

    def gps_callback(self, msg):
        gps_time = msg.header.stamp.to_sec()
        current_time = rospy.Time.now().to_sec()
        if abs(current_time - gps_time) >1.5:  # GPS data too old
            rospy.logwarn(f"Current time: {current_time}, GPS time: {gps_time}")
            rospy.logwarn("GPS data too old")
            return

        if np.isnan(msg.pose.pose.position.x) or np.isnan(msg.pose.pose.position.y):
            rospy.logwarn("Invalid GPS data received")
            return

        gps_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0])
        self.ekf_update(gps_measurement, self.R_gps)

    def camera_callback(self, msg):
        if np.isnan(msg.pose.pose.position.x) or np.isnan(msg.pose.pose.position.y):
            rospy.logwarn("Invalid Camera data received")
            return
        camera_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0])
        self.ekf_update(camera_measurement, self.R_camera)

    def odom_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        dt = current_time - self.last_odom
        self.last_odom = current_time

        delta_x = msg.twist.twist.linear.x * dt
        delta_y = msg.twist.twist.linear.y * dt
        delta_theta = msg.twist.twist.angular.z * dt
        self.ekf_predict(delta_x, delta_y, delta_theta)

    def ekf_predict(self, delta_x, delta_y, delta_theta):
        theta = self.X[2]
        F = np.array([
            [1, 0, -delta_x * sin(theta) - delta_y * cos(theta)],
            [0, 1, delta_x * cos(theta) - delta_y * sin(theta)],
            [0, 0, 1]
        ])

        self.X[0] += delta_x * cos(theta) - delta_y * sin(theta)
        self.X[1] += delta_x * sin(theta) + delta_y * cos(theta)
        self.X[2] += delta_theta
        self.X[2] = self.normalize_angle(self.X[2])

        self.P = F @ self.P @ F.T + self.Q

    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))  # Ensure angle is in [-pi, pi]

    def ekf_update(self, z, R):
        if np.any(np.isnan(z)):  # Check if the measurement is valid
            rospy.logwarn("Invalid measurement data received.")
            return
        
        H = np.eye(3)  # Observation matrix
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X += K @ (z - H @ self.X)
        self.X[2] = self.normalize_angle(self.X[2])
        self.P = (np.eye(3) - K @ H) @ self.P
        self.broadcast_footprint()

    def broadcast_footprint(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.X[2])
        self.tf_broadcaster.sendTransform(
            (self.X[0], self.X[1], 0.0),
            q,
            rospy.Time.now(),
            self.child_frame_id,
            self.parent_frame_id
        )

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    ekf_broadcaster = EKFFootprintBroadcaster()
    ekf_broadcaster.run()
