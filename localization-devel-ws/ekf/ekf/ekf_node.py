#!/usr/bin/env python3
import math
import numpy as np
import time
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import Imu
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return [
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]

def euler_from_quaternion(x, y, z, w):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

class EKFFootprintBroadcaster(Node):
    def __init__(self):
        super().__init__('ekf')
        self.X = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3) * 9e-2
        self.P[2, 2] = 3
        self.Q = np.eye(3)
        self.R_gps = np.eye(3) * 1e-2
        self.R_gps[2, 2] = 0.09
        self.R_camera = np.eye(3) * 1e-2
        self.v_x = 0.0
        self.v_y = 0.0
        self.w = 0.0
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_odom_time = now
        self.gps_time = now
        self.cam_time = now
        self.last_pose = np.array([-1.0, 0.0, 0.0])
        self.claim_parameters()
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.t = TransformStamped()
        self.t.header.frame_id = self.parent_frame_id
        self.t.child_frame_id = self.child_frame_id
        self.final_pose = PoseWithCovarianceStamped()
        self.final_pose.header.frame_id = self.parent_frame_id
        self.cam_measurement = [-100, -100, -100]
        self.init_topics()
        self.footprint_publish()
        if self.use_cam:
            self.create_timer(1.0 / self.rate, self.camera_update)

    def claim_parameters(self):
        self.declare_parameter('use_cam', 0)
        self.declare_parameter('robot_parent_frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('update_rate', 1)
        self.declare_parameter('q_linear', 1e-3)
        self.declare_parameter('q_angular', 1e-2)
        self.declare_parameter('r_camera_linear', 1e-2)
        self.declare_parameter('r_camera_angular', 0.15)
        self.declare_parameter('r_threshold_xy', 1e-3)
        self.declare_parameter('r_threshold_theta', 1e-2)
        self.parent_frame_id = self.get_parameter('robot_parent_frame_id').value
        self.child_frame_id = self.get_parameter('robot_frame_id').value
        self.rate = self.get_parameter('update_rate').value
        self.Q[0, 0] = self.get_parameter('q_linear').value
        self.Q[1, 1] = self.get_parameter('q_linear').value
        self.Q[2, 2] = self.get_parameter('q_angular').value
        self.R_camera[0, 0] = self.get_parameter('r_camera_linear').value
        self.R_camera[1, 1] = self.get_parameter('r_camera_linear').value
        self.R_camera[2, 2] = self.get_parameter('r_camera_angular').value
        self.use_cam = self.get_parameter('use_cam').value
        self.r_threshold_xy = self.get_parameter('r_threshold_xy').value
        self.r_threshold_theta = self.get_parameter('r_threshold_theta').value

    def init_topics(self):
        self.create_subscription(PoseWithCovarianceStamped, 'lidar_pose', self.gps_callback, 1)
        self.create_subscription(PoseWithCovarianceStamped, 'initial_pose', self.init_callback, 1)
        self.create_subscription(Twist, 'odoo_googoogoo', self.odomcallback, 1)
        self.create_subscription(Imu, '/imu/data_cov', self.imu_callback, 1)
        self.create_subscription(PoseStamped, '/ceiling_robot/pose', self.camera_callback, 1)
        self.ekf_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'final_pose', 1)

    def init_callback(self, msg):
        self.X[0] = msg.pose.pose.position.x
        self.X[1] = msg.pose.pose.position.y
        self.X[2] = euler_from_quaternion(msg.pose.pose.orientation.x,
                                          msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z,
                                          msg.pose.pose.orientation.w)
        if msg.pose.covariance[0] > 0 and msg.pose.covariance[7] > 0 and msg.pose.covariance[35] > 0:
            self.P[0, 0] = msg.pose.covariance[0]
            self.P[1, 1] = msg.pose.covariance[7]
            self.P[2, 2] = msg.pose.covariance[35]

    def gps_callback(self, msg):
        self.gps_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(self.get_clock().now().nanoseconds / 1e9 - self.gps_time) > 1.5:
            return
        theta = euler_from_quaternion(msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w)
        gps_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, theta])
        self.R_gps[0, 0] = min(msg.pose.covariance[0], self.r_threshold_xy)
        self.R_gps[1, 1] = min(msg.pose.covariance[7], self.r_threshold_xy)
        self.R_gps[2, 2] = min(msg.pose.covariance[35], self.r_threshold_theta)
        self.ekf_update(gps_measurement, self.R_gps)

    def camera_callback(self, msg):
        self.cam_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        theta = euler_from_quaternion(msg.pose.orientation.x,
                                      msg.pose.orientation.y,
                                      msg.pose.orientation.z,
                                      msg.pose.orientation.w)
        self.cam_measurement = np.array([msg.pose.position.x, msg.pose.position.y, theta])

    def camera_update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if abs(now - self.cam_time) > 1.5 or self.cam_measurement[0] == -100:
            return
        if abs(now - self.gps_time) > 0.2:
            self.R_camera[2, 2] = 1e-10
        self.ekf_update(self.cam_measurement, self.R_camera)
        self.R_camera[0, 0] = self.R_camera[1, 1] = 1e-2
        self.R_camera[2, 2] = 0.15

    def imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_odom_time
        self.last_odom_time = now
        self.w = msg.angular_velocity.z
        self.ekf_predict(self.v_x, self.v_y, self.w, dt)

    def odomcallback(self, msg):
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y

    def ekf_predict(self, v_x, v_y, w, dt):
        theta = self.X[2]
        ct, st = np.cos(theta), np.sin(theta)
        cd, sd = np.cos(w * dt), np.sin(w * dt)

        if abs(w) > 1e-3:
            self.X[0] += (ct * sd - st * (cd - 1)) * v_x / w - (st * sd - ct * (cd - 1)) * v_y / w
            self.X[1] += (st * sd - ct * (cd - 1)) * v_x / w + (ct * sd - st * (cd - 1)) * v_y / w
        else:
            dx = v_x * dt
            dy = v_y * dt
            self.X[0] += dx * np.cos(theta + w*dt) - dy * np.sin(theta + w*dt)
            self.X[1] += dx * np.sin(theta + w*dt) + dy * np.cos(theta + w*dt)
        self.X[2] += w * dt


        self.footprint_publish()
        self.P = self.P + self.Q

    def ekf_update(self, z, R):
        if np.any(np.isnan(z)):
            self.get_logger().warn("Invalid measurement data received.")
            return

        residual = z - self.X
        residual[2] = normalize_angle(residual[2])

        if np.all(np.abs(residual) < np.array([1e-4, 1e-4, 1e-3])):
            return

        # start_time = time.perf_counter()
        K = self.P @ np.linalg.inv(self.P + R)
        self.X += K @ residual
        self.X[2] = normalize_angle(self.X[2])
        self.P = (np.eye(3) - K) @ self.P
        # end_time = time.perf_counter()
        # self.get_logger().info(f"EKF update took: {end_time - start_time:.6f} s")

    def footprint_publish(self):
        now = self.get_clock().now().to_msg()
        self.final_pose.header.stamp = now
        self.t.header.stamp = now
        quat = quaternion_from_euler(0, 0, self.X[2])
        
        self.final_pose.pose.pose.position.x = self.X[0]
        self.final_pose.pose.pose.position.y = self.X[1]
        self.final_pose.pose.pose.orientation.z = quat[0]
        self.final_pose.pose.pose.orientation.w = quat[1]
        self.final_pose.pose.covariance[0] = self.P[0, 0]
        self.final_pose.pose.covariance[7] = self.P[1, 1]
        self.final_pose.pose.covariance[35] = self.P[2, 2]
        self.ekf_pose_publisher.publish(self.final_pose)
        
        if np.allclose(self.X, self.last_pose, atol=1e-4):
            return
        self.last_pose = self.X.copy()

        self.t.transform.translation.x = self.X[0]
        self.t.transform.translation.y = self.X[1]
        self.t.transform.translation.z = 0.0
        
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = quat[0]
        self.t.transform.rotation.w = quat[1]
        self.tf_static_broadcaster.sendTransform(self.t)

def main(args=None):
    rclpy.init(args=args)
    ekf = EKFFootprintBroadcaster()
    rclpy.spin(ekf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
