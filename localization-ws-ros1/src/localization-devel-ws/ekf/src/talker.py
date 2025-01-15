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

class Talker:
    def __init__(self):
        rospy.init_node('talker')

        self.ekf_pose_publisher = rospy.Publisher('/robot/final_pose', Odometry, queue_size=10)

        self.X = np.array([2.259588, 1.709261426, 170.9])  # Initial state

        self.parent_frame_id = rospy.get_param('~robot_parent_frame_id', 'robot/map')
        self.rate = rospy.Rate(10)  # 10 Hz
        self.last_broadcast_time = rospy.Time.now()

    def broadcast_footprint(self):
        current_time = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(0, 0, self.X[2])  # Convert Euler to quaternion

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

    def start(self):
        while not rospy.is_shutdown():
            self.broadcast_footprint()
            self.rate.sleep()

if __name__ == '__main__':
    talker = Talker()
    talker.start()
