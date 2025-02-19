#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def publish_pose():
    rospy.init_node('pose_publisher')
    
    # 从参数服务器读取配置
    position = rospy.get_param('~position', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    orientation = rospy.get_param('~orientation', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
    
    # 设置Publisher
    pub = rospy.Publisher('/robot/final_pose', Odometry, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz 发布频率

    while not rospy.is_shutdown():
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = position['x']
        odom_msg.pose.pose.position.y = position['y']
        odom_msg.pose.pose.position.z = position['z']
        odom_msg.pose.pose.orientation.x = orientation['x']
        odom_msg.pose.pose.orientation.y = orientation['y']
        odom_msg.pose.pose.orientation.z = orientation['z']
        odom_msg.pose.pose.orientation.w = orientation['w']
        pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
