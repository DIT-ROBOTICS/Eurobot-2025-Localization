#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped
import geometry_msgs.msg
import numpy as np
from math import sin, cos, atan2
import tf
import tf2_ros

class EKFFootprintBroadcaster:
    def __init__(self):
        rospy.init_node('ekf_footprint_broadcaster')
        self.listener = tf.TransformListener()
        rospy.Subscriber('/robot/lidar_bonbonbon', PoseWithCovarianceStamped, self.gps_callback)
        rospy.Subscriber('/robot/camera', Odometry, self.camera_callback)
        rospy.Subscriber('/robot/local_filter', Odometry, self.odom_callback)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.ekf_pose_publisher = rospy.Publisher('/robot/final_pose', Odometry, queue_size=10)

        self.X = np.array([2.259588, 1.709261426,  47.599])  # Initial state

        self.P = np.eye(3) * 1e-3
        self.Q = np.eye(3) * 1e-3  # Process noise
        self.R_gps = np.eye(3) * 1e-2  # GPS measurement noise
        self.R_camera = np.eye(3) * 1e-2  # Camera measurement noise

        self.last_odom = rospy.Time.now().to_sec()
        self.last_broadcast_time = rospy.Time.now()

        self.parent_frame_id = rospy.get_param('~robot_parent_frame_id', 'robot/map')
        self.child_frame_id = rospy.get_param('~robot_frame_id', 'robot/base_footprint')
        self.predict_frame_id = rospy.get_param('~robot_predict_frame_id', 'robot/predict')
        self.rate = rospy.Rate(rospy.get_param('~update_rate', 100))  # based on the update rate of the odometry
        current_time = rospy.Time.now()
        self.broadcast_footprint()
        rospy.loginfo("EKF Footprint Broadcaster initialized.")

    def gps_callback(self, msg):
        gps_time = msg.header.stamp.to_sec()
        current_time = rospy.Time.now().to_sec()
        if abs(current_time - gps_time) >1.5:  # GPS data too old
            rospy.logwarn(f"Current time: {current_time}, GPS time: {gps_time}")
            return

        if self.is_invalid_data(msg.pose.pose.position.x, msg.pose.pose.position.y):
            rospy.logwarn("Invalid GPS data received")
            return
        theta = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]    
        gps_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, theta])
        self.ekf_update(gps_measurement, self.R_gps)

    def camera_callback(self, msg):
        if self.is_invalid_data(msg.pose.pose.position.x, msg.pose.pose.position.y):
            rospy.logwarn("Invalid Camera data received")
            return
        theta_Cam = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]    
        camera_measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, theta_Cam])
        # try:
        #     (trans, rot) = self.listener.lookupTransform('/robot/map', '/robot/predict', rospy.Time.now())
        #     theta_Cam = tf.transformations.euler_from_quaternion(rot)[2]
        #     camera_measurement = np.array([trans[0], trans[1], theta_Cam])
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.logwarn("TF lookup from /robot/map to /robot/predict failed.")
        #     return
        # self.ekf_update(camera_measurement, self.R_camera)

    def odom_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        dt = current_time - self.last_odom
        self.last_odom = current_time

        delta_x = msg.twist.twist.linear.x * dt
        delta_y = msg.twist.twist.linear.y * dt
        delta_theta = msg.twist.twist.angular.z * dt

        # delta_x = 0
        # delta_y = 0
        # delta_theta = 0
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
        self.broadcast_footprint()

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
        

    def broadcast_footprint(self):
        current_time = rospy.Time.now()
        # if (current_time - self.last_broadcast_time).to_sec() < 0.001:  
        #     return
        
        q = tf.transformations.quaternion_from_euler(0, 0, self.X[2])
        _static_transform_stamped = geometry_msgs.msg.TransformStamped()
        _static_transform_stamped.header.stamp = current_time
        _static_transform_stamped.header.frame_id = self.parent_frame_id    
        _static_transform_stamped.child_frame_id = self.child_frame_id
        _static_transform_stamped.transform.translation.x = self.X[0]
        _static_transform_stamped.transform.translation.y = self.X[1]
        _static_transform_stamped.transform.translation.z = 0
        _static_transform_stamped.transform.rotation.x = q[0]
        _static_transform_stamped.transform.rotation.y = q[1]
        _static_transform_stamped.transform.rotation.z = q[2]
        _static_transform_stamped.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(_static_transform_stamped)
        self.last_broadcast_time = current_time

        self.ekf_pose = Odometry()
        self.ekf_pose.header.stamp = current_time
        self.ekf_pose.header.frame_id = self.parent_frame_id
        self.ekf_pose.pose.pose.position.x = self.X[0]
        self.ekf_pose.pose.pose.position.y = self.X[1]
        self.ekf_pose.pose.pose.position.z = 0.0
        self.ekf_pose.pose.pose.orientation.x = q[0]
        self.ekf_pose.pose.pose.orientation.y = q[1]
        self.ekf_pose.pose.pose.orientation.z = q[2]
        self.ekf_pose.pose.pose.orientation.w = q[3]
        self.ekf_pose_publisher.publish(self.ekf_pose)
        
        # try:
        #     # Listen to the transform from map to base_footprint
        #     current_time = rospy.Time.now()
        #     (trans, rot) = self.listener.lookupTransform(self.parent_frame_id, self.predict_frame_id, rospy.Time.now())
        #     theta = tf.transformations.euler_from_quaternion(rot)[2]
        #     rospy.loginfo(f"Current TF from {self.parent_frame_id} to {self.predict_frame_id}: "
        #                   f"x={trans[0]}, y={trans[1]}, theta={theta}")
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.logwarn(f"TF lookup from {self.parent_frame_id} to {self.predict_frame_id} failed.")
        #     return
        
    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))

    def is_invalid_data(self, x, y):
        return np.isnan(x) or np.isnan(y)

    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    ekf_broadcaster = EKFFootprintBroadcaster()
    ekf_broadcaster.run()