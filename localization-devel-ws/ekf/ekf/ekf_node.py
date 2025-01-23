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

        self.X = np.array([0.0, 0.0, 0.0])  # State vector: x, y, theta
        self.P_predict = np.eye(3) * 0.16
        self.P_update = np.eye(3) * 1e-2
        self.Q = np.eye(3) * 1e-3
        self.R_gps = np.eye(3) * 1e-2
        self.R_camera = np.eye(3) * 1e-2
        self.R_camera[2, 2] = 0.25

        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9
        self.cnt = 0
        self.init_subscribers()
        self.ekf_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'final_pose', 10)
        # self.create_timer(1.0 / self.rate, self.footprint_publish)
        self.create_timer(1, self.camera_callback)
        self.footprint_publish()

        
    def claim_parameters(self):
        self.declare_parameter('robot_parent_frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('camera_frame_id', 'marker_6')
        self.declare_parameter('camera_parent_id', 'map')
        self.declare_parameter('update_rate', 50)

        self.parent_frame_id = self.get_parameter('robot_parent_frame_id').value
        self.child_frame_id = self.get_parameter('robot_frame_id').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.camera_parent_id = self.get_parameter('camera_parent_id').value
        self.rate = self.get_parameter('update_rate').value

    def init_subscribers(self):
        self.create_subscription(PoseWithCovarianceStamped, 'lidar_pose', self.gps_callback, 10)
        self.create_subscription(Twist, 'odoo_googoogoo', self.odom_callback, 10)
        self.create_subscription(PoseWithCovariance, 'initial_pose', self.init_callback,10)
    
    def init_callback(self, msg):

        theta = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        if self.cnt == 0:
            self.X[2] = theta
            if msg.covariance[0] > 0:
                if msg.covariance[0] < 1:
                    self.P_predict[0, 0] = msg.covariance[0]
                    self.P_predict[1, 1] = msg.covariance[7]
                    self.P_predict[2, 2] = msg.covariance[35]
            self.X[0] = msg.pose.position.x
            self.X[1] = msg.pose.position.y
            self.cnt += 1


    def gps_callback(self, msg):
        gps_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_time = self.get_clock().now().nanoseconds / 1e9
        if abs(current_time - gps_time) > 1.5:  # GPS data too old
            # self.get_logger().info(f"Current time: {current_time}, GPS time: {gps_time}")
            return

        if is_invalid_data(msg.pose.pose.position.x, msg.pose.pose.position.y):
            # self.get_logger().info("Invalid GPS data received")
            return

        theta = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        gps_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, theta])

        self.R_gps[0, 0] = msg.pose.covariance[0]
        self.R_gps[1, 1] = msg.pose.covariance[7]
        self.R_gps[2, 2] = msg.pose.covariance[35]
        self.ekf_update(gps_measurement, self.R_gps)


    def camera_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # self.get_logger().info(f"Camera callback triggered at: {now}")
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_parent_id,
                self.camera_frame_id, 
                rclpy.time.Time()      
            )
            trans = t.transform.translation
            rot = t.transform.rotation
            theta = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)

            camera_measurement = np.array([trans.x, trans.y, theta])
            # self.get_logger().info(f"Camera transform: x={trans.x}, y={trans.y}, theta={theta}")
            self.ekf_update(camera_measurement, self.R_camera)
        except TransformException as ex:
            self.get_logger().warn(f"TransformException in camera_callback: {ex}")

    def odom_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        delta_x = msg.linear.x * dt /1e3
        delta_y = msg.linear.y * dt /1e3
        delta_theta = msg.angular.z * dt /1e3 
        # self.get_logger().info(f"dTime:{dt}, d_x:{delta_x}")
        self.ekf_predict(delta_x, delta_y, delta_theta) 

    def ekf_predict(self, delta_x, delta_y, delta_theta):
        theta = self.X[2]
        F = np.eye(3)
        F[0, 2] = -delta_x * math.sin(theta) - delta_y * math.cos(theta)
        F[1, 2] = delta_x * math.cos(theta) - delta_y * math.sin(theta)

        self.X[0] += delta_x * math.cos(theta) - delta_y * math.sin(theta)
        self.X[1] += delta_x * math.sin(theta) + delta_y * math.cos(theta)
        self.X[2] += delta_theta
        self.X[2] = normalize_angle(self.X[2])
        # self.get_logger().info(f"predict-x{self.X[0]}")
        self.P_predict = F @ self.P_update @ F.T + self.Q
        if (self.P_predict[0, 0] > 1) | (self.P_predict[1, 1] > 1 ) | (self.P_predict[2, 2] > 1) :
            self.get_logger().warn(f"large Cov_pred:{self.P_predict[0, 0]},{self.P_predict[1, 1]},{self.P_predict[2, 2]}")
            self.P_predict = np.eye(3) * 1e-2
        self.footprint_publish()

    def ekf_update(self, z, R):
        if np.any(np.isnan(z)):  # Check if the measurement is valid
            self.get_logger().warn("Invalid measurement data received.")
            return
        
        H = np.eye(3)  # Observation matrix
        S = H @ self.P_predict @ H.T + R
        K = self.P_predict @ H.T @ np.linalg.inv(S)
        self.X += K @ (z - H @ self.X)
        self.X[2] = normalize_angle(self.X[2])
        self.P_update = (np.eye(3) - K @ H) @ self.P_predict
        if (self.P_update[0, 0] > 1) | (self.P_update[1, 1] > 1 ) | (self.P_update[2, 2] > 1) :
            self.get_logger().warn(f"large Cov_update:{self.P_update[0, 0]},{self.P_update[1, 1]},{self.P_update[2, 2]}")
            self.P_predict = np.eye(3) * 1e-2
            
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

        final_pose = PoseWithCovarianceStamped()
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.header.frame_id = self.parent_frame_id
        final_pose.pose.pose.position.x = self.X[0]
        final_pose.pose.pose.position.y = self.X[1]
        final_pose.pose.pose.position.z = 0.0
        final_pose.pose.pose.orientation.x = quat[0]
        final_pose.pose.pose.orientation.y = quat[1]
        final_pose.pose.pose.orientation.z = quat[2]
        final_pose.pose.pose.orientation.w = quat[3]
        final_pose.pose.covariance[0] = self.P_predict[0, 0]
        final_pose.pose.covariance[7] = self.P_predict[1, 1]
        final_pose.pose.covariance[35] = self.P_predict[2, 2]
        self.ekf_pose_publisher.publish(final_pose)


def main(args=None):
    rclpy.init(args=args)
    ekf = EKFFootprintBroadcaster()
    rclpy.spin(ekf)  # Keep the node running
    rclpy.shutdown()  # Shut down the ROS 2 client

if __name__ == '__main__':
    main()

