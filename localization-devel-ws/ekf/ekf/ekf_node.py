#!/usr/bin/env python3
import math
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance
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
        
        self.X = np.array([0.0, 0.0, 0.0])  # State vector: x, y, theta
        self.P = np.eye(3) * 9 * 1e-4
        self.P[2, 2] = 0.003 
        self.Q = np.eye(3) 
        self.R_gps = np.eye(3) * 1e-2
        self.R_gps[2, 2] = 0.09
        self.R_camera = np.eye(3) * 1e-2

        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9
        self.gps_time = self.get_clock().now().nanoseconds / 1e9
        self.claim_parameters()
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.t = TransformStamped()
        self.t.header.frame_id = self.parent_frame_id
        self.t.child_frame_id = self.child_frame_id
        self.final_pose = PoseWithCovarianceStamped()
        self.final_pose.header.frame_id = self.parent_frame_id
        self.cam_measurement = [-100, -100, -100]
        self.cam_time = 0
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
        self.create_subscription(PoseWithCovariance, 'initial_pose', self.init_callback,1)
        self.create_subscription(Odometry, 'local_filter', self.local_callback, 1)
        self.create_subscription(PoseStamped, '/ceiling_robot/pose', self.camera_callback, 1)
        self.ekf_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'final_pose', 1)

    
    def init_callback(self, msg):
        
        self.X[0] = msg.pose.position.x
        self.X[1] = msg.pose.position.y

        theta = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        self.X[2] = theta
        if msg.covariance[0] > 0 and msg.covariance[7] > 0 and msg.covariance[35] > 0:
            if msg.covariance[0] < 1 and msg.covariance[7] < 1 and msg.covariance[35] < 1:
                self.P[0, 0] = msg.covariance[0]
                self.P[1, 1] = msg.covariance[7]
                self.P[2, 2] = msg.covariance[35]

    def gps_callback(self, msg):
        self.gps_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_time = self.get_clock().now().nanoseconds / 1e9
        if abs(current_time - self.gps_time) > 1.5:
            return

        if is_invalid_data(msg.pose.pose.position.x, msg.pose.pose.position.y):
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
        for i in range(2):
            if self.R_gps[i, i] > self.r_threshold_xy :
                self.R_gps[i, i] = self.r_threshold_xy 
        if self.R_gps[2, 2] > self.r_threshold_theta:
            self.R_gps[2, 2] = self.r_threshold_theta
        self.ekf_update(gps_measurement, self.R_gps)

    def camera_callback(self, msg):
        self.cam_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if is_invalid_data(msg.pose.position.x, msg.pose.position.y):
            return

        theta = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        self.cam_measurement = np.array([msg.pose.position.x, msg.pose.position.y, theta])

    def camera_update(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if abs(current_time - self.cam_time) > 1.5:  
            self.cam_measurement = [-100, -100, -100]
            return
        if self.cam_measurement[0]==-100:  # Check if the measurement is valid
            self.get_logger().warn("Invalid cam measurement data received.")
            return
        null_time = abs(current_time - self.gps_time)
        if null_time > 0.2:
            self.R_camera[2,2] = 1e-10
           
        self.ekf_update(self.cam_measurement, self.R_camera)
        self.R_camera[0,0] = 1e-2
        self.R_camera[1,1] = 1e-2
        self.R_camera[2,2] = 0.15

    def local_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        w = msg.twist.twist.angular.z
        # self.get_logger().info(f"dTime:{dt}, d_x:{delta_x}")
        self.ekf_predict(v_x, v_y, w, dt) 

    def ekf_predict(self, v_x, v_y, w, dt):
        theta = self.X[2]
        c_theta = math.cos(theta)
        s_theta = math.sin(theta)
        c_delta = math.cos(w * dt)
        s_delta = math.sin(w * dt)
        if abs(w) > 1e-3:
            self.X[0] += (c_theta*s_delta - s_theta*(c_delta-1))*v_x / w - (s_theta*s_delta - c_theta*(c_delta-1))*v_y / w
            self.X[1] += (s_theta*s_delta - c_theta*(c_delta-1))*v_x / w + (c_theta*s_delta - s_theta*(c_delta-1))*v_y / w
        else:
            self.X[0] += v_x * dt * math.cos(theta + w * dt) - v_y * dt * math.sin(theta + w * dt)
            self.X[1] += v_x * dt *math.sin(theta + w * dt) + v_y * dt * math.cos(theta + w * dt)

        self.X[2] += w * dt
        self.footprint_publish()
        self.P = self.P + self.Q
        # if (self.P[0, 0] > 1e-2) | (self.P[1, 1] > 1e-2 ) | (self.P[2, 2] > 0.003) :
        #     self.get_logger().warn(f"large Cov_update:{self.P[0, 0]},{self.P[1, 1]},{self.P[2, 2]}")
        #     self.P = np.eye(3) * 1e-2
        #     self.P[2, 2] = 0.003

    def ekf_update(self, z, R):
        if np.any(np.isnan(z)):  # Check if the measurement is valid
            self.get_logger().warn("Invalid measurement data received.")
            return
        
        K = self.P @ np.linalg.inv(self.P + R)
        self.P = (np.eye(3) - K) @ self.P
        # self.X = self.X + K @ (z - self.X) # here we should make sure angle subtraction, not just z - self.X
        residual = z - self.X
        if abs(residual[2]) > math.pi:
            residual[2] = normalize_angle(residual[2])
        self.X = self.X + K @ residual

        # if (self.P[0, 0] > 1e-2) | (self.P[1, 1] > 1e-2 ) | (self.P[2, 2] > 0.003) : # TODO: position and theta should be checked seperately
        #     self.get_logger().warn(f"large Cov_update:{self.P[0, 0]},{self.P[1, 1]},{self.P[2, 2]}")
        #     self.P = np.eye(3) * 1e-2
        #     self.P[2, 2] = 0.003
            
    def footprint_publish(self):
        self.final_pose.header.stamp = self.get_clock().now().to_msg()
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.transform.translation.x = self.X[0]
        self.t.transform.translation.y = self.X[1]
        self.t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.X[2])
        self.t.transform.rotation.x = quat[0]
        self.t.transform.rotation.y = quat[1]
        self.t.transform.rotation.z = quat[2]
        self.t.transform.rotation.w = quat[3]
        self.tf_static_broadcaster.sendTransform(self.t)

        
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

