#!/usr/bin/env python3
import math
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def quaternion_from_euler(roll, pitch, yaw):
    roll, pitch, yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]

def euler_from_quaternion(x, y, z, w):
    t0, t1 = +2.0 * (w * x + y * z), +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3, t4 = +2.0 * (w * z + x * y), +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def is_invalid_data(x, y):
    return np.isnan(x) or np.isnan(y)

class EKFFootprintBroadcaster(Node):
    def __init__(self):
        super().__init__('ekf')
        self.claim_parameters()

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.final_pose = PoseWithCovarianceStamped()
        self.final_pose.header.frame_id = self.parent_frame_id

        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # State vector: x, y, theta, vx, vy, w
        self.P = np.eye(6) * 9 * 1e-4
        self.P[2, 2] = 0.003 # theta
        self.P[3, 3] = 1e-6 # vx
        self.P[4, 4] = 1e-6 # vy
        self.P[5, 5] = 1e-6 # w

        self.init_subscribers()
        self.footprint_publish()

        
    def claim_parameters(self):
        self.declare_parameter('robot_parent_frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_footprint')

        self.parent_frame_id = self.get_parameter('robot_parent_frame_id').value
        self.child_frame_id = self.get_parameter('robot_frame_id').value
 

    def init_subscribers(self):
        self.create_subscription(PoseWithCovarianceStamped, 'lidar_pose', self.gps_callback, 10)
    
    def gps_callback(self, msg):
        self.X[0] = msg.pose.pose.position.x
        self.X[1] = msg.pose.pose.position.y
        self.X[2] = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.P[0, 0] = msg.pose.covariance[0]
        self.P[1, 1] = msg.pose.covariance[7]
        self.P[2, 2] = msg.pose.covariance[35]

        self.footprint_publish()
      
    def footprint_publish(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = self.X[0]
        t.transform.translation.y = self.X[1]
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.X[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_static_broadcaster.sendTransform(t)

        self.final_pose.header.stamp = self.get_clock().now().to_msg()
        self.final_pose.pose.pose.position.x = self.X[0]
        self.final_pose.pose.pose.position.y = self.X[1]
        self.final_pose.pose.pose.position.z = 0.0
        self.final_pose.pose.pose.orientation.x = quat[0]
        self.final_pose.pose.pose.orientation.y = quat[1]
        self.final_pose.pose.pose.orientation.z = quat[2]
        self.final_pose.pose.pose.orientation.w = quat[3]
        self.final_pose.pose.covariance[0] = self.P[0, 0]
        self.final_pose.pose.covariance[7] = self.P[1, 1]
        self.final_pose.pose.covariance[35] = self.P[2, 2]
        self.ekf_pose_publisher.publish(self.final_pose)


def main(args=None):
    rclpy.init(args=args)
    ekf = EKFFootprintBroadcaster()
    rclpy.spin(ekf)  # Keep the node running
    rclpy.shutdown()  # Shut down the ROS 2 client

if __name__ == '__main__':
    main()

