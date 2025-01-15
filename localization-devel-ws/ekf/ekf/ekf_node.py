#!/usr/bin/env python3
import math
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def euler_from_quaternion(self, x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z # in radians

def normalize_angle(self, angle):
    return atan2(sin(angle), cos(angle))

def is_invalid_data(self, x, y):
    return np.isnan(x) or np.isnan(y)

class EKFFootprintBroadcaster(Node):
    def __init__(self):
        super().__init__('ekf')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(PoseWithCovarianceStamped, 'robot/lidar_bonbonbon', self.gps_callback, 10)
        self.create_subscription(Odometry, 'robot/camera', self.camera_callback, 10)
        self.create_subscription(Odometry, 'robot/local_filter', self.odom_callback, 10)
        self.ekf_pose_publisher = self.create_publisher(Odometry, 'robot/final_pose', 10)

        self.declare_parameter('robot_parent_frame_id', 'robot/map')
        self.declare_parameter('robot_frame_id', 'robot/base_footprint')
        self.declare_parameter('update_rate', 100)

        self.parent_frame_id = self.get_parameter('robot_parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        self.rate = self.get_parameter('update_rate').get_parameter_value().integer_value

        self.X = np.array([0, 0, 0.0])
        self.P = np.eye(3) * 1e-3
        self.Q = np.eye(3) * 1e-3
        self.R_gps = np.eye(3) * 1e-2
        self.R_camera = np.eye(3) * 1e-2

        self.last_odom = self.get_clock().now().nanoseconds / 1e9
        self.footprint_publish()

    def gps_callback(self, msg):
        gps_time = msg.header.stamp.to_sec()
        current_time = self.get_clock().now().nanoseconds / 1e9
        if abs(current_time - gps_time) >1.5:  # GPS data too old
            self.get_logger().info(f"Current time: {current_time}, GPS time: {gps_time}")
            return

        if self.is_invalid_data(msg.pose.pose.position.x, msg.pose.pose.position.y):
            self.get_logger().info("Invalid GPS data received")
            return
        theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])    
        gps_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, theta])
        R_gps[0] = msg.covariance[0]
        R_gps[4] = msg.covariance[7]
        R_gps[7] = msg.covariance[35]
        self.ekf_update(gps_measurement, self.R_gps)

    def camera_callback(self, msg):
        if is_invalid_data(msg.pose.pose.position.x, msg.pose.pose.position.y):
            logger = rclpy.logging.get_logger('logger')
            logger.info('Invalid Camera data')
            return
        theta_Cam = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]    
        camera_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, theta_Cam])
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
        self.footprint_publish()

    def ekf_update(self, z, R):
        if np.any(np.isnan(z)):  # Check if the measurement is valid
            self.get_logger().warn("Invalid measurement data received.")
            return
        
        H = np.eye(3)  # Observation matrix
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X += K @ (z - H @ self.X)
        self.X[2] = self.normalize_angle(self.X[2])
        self.P = (np.eye(3) - K @ H) @ self.P

    def footprint_publish(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('robot_parent_frame_id').value
        t.child_frame_id = self.get_parameter('robot_frame_id').value

        t.transform.translation.x = self.X[0]
        t.transform.translation.y = self.X[1]
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.X[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_static_broadcaster.sendTransform(t)

        final_pose = Odometry()
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.header.frame_id = self.parent_frame_id
        final_pose.pose.pose.position.x = self.X[0]
        final_pose.pose.pose.position.y = self.X[1]
        final_pose.pose.pose.position.z = 0.0
        final_pose.pose.pose.orientation.x = quat[0]
        final_pose.pose.pose.orientation.y = quat[1]
        final_pose.pose.pose.orientation.z = quat[2]
        final_pose.pose.pose.orientation.w = quat[3]
        self.ekf_pose_publisher.publish(final_pose)

def main(args=None):
    rclpy.init(args=args)
    ekf = EKFFootprintBroadcaster()
    rclpy.spin(ekf)  # Keep the node running
    rclpy.shutdown()  # Shut down the ROS 2 client

if __name__ == '__main__':
    main()

