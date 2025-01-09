#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

def publish_camera_data():
    rospy.init_node('camera_publisher', anonymous=True)
    camera_pub = rospy.Publisher('robot/camera', Odometry, queue_size=10)
    
    clock_msg = rospy.wait_for_message('/clock', Clock)
    start_time = clock_msg.clock.to_sec()
    
    # 數據列表
    data_list = [
        {"time": 50.0, "position": (0.4806712233883248,1.0932451234202643, 0), "orientation": (0, 0, 0.9999016629254438,-0.014023711310927924)},
        {"time": 50.5, "position": (0.4806712233883248,1.0932451234202643, 0), "orientation": (0, 0, 0.9999016629254438,-0.014023711310927924)},
        {"time": 51, "position": (0.4806712233883248,1.0932451234202643, 0), "orientation": (0, 0, 0.9999016629254438,-0.014023711310927924)},
        # {"time": 32.45, "position": (2.634, 1.384, 0), "orientation": (0, 0, -0.7039, 0.7103)},
        # {"time": 35.39, "position": (3, 2, 0), "orientation": (0, 0, 0, -1)},
        # {"time": 40.35, "position": (2.386, 1.244, 0), "orientation": (0, 0, 0.089, -0.996)}
    ]
    
    for data in data_list:
        # 計算目標時間
        target_time = start_time + data["time"]
        rospy.loginfo(f"start time: {start_time}, target time: {target_time}")

        # 獲取當前時間
        current_time = rospy.Time.now().to_sec()  # 使用模擬時間
        delay = target_time - current_time

        # 如果還沒到目標時間，則等待
        if delay > 0:
            rospy.sleep(delay)

        # 構建 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.from_sec(target_time)  # 使用目標時間
        odom_msg.header.frame_id = "robot/map"
        
        # 設置位置和方向
        odom_msg.pose.pose.position.x = data["position"][0]
        odom_msg.pose.pose.position.y = data["position"][1]
        odom_msg.pose.pose.position.z = data["position"][2]
        odom_msg.pose.pose.orientation.x = data["orientation"][0]
        odom_msg.pose.pose.orientation.y = data["orientation"][1]
        odom_msg.pose.pose.orientation.z = data["orientation"][2]
        odom_msg.pose.pose.orientation.w = data["orientation"][3]
        
        # 發布消息
        camera_pub.publish(odom_msg)
        rospy.loginfo(f"Published camera data at {current_time}")

    rospy.loginfo("Finished publishing camera data")

if __name__ == '__main__':
    try:
        publish_camera_data()
    except rospy.ROSInterruptException:
        pass
