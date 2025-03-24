import rclpy
from rclpy.node import Node 
from obstacle_detector.msg import Obstacles
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class LidarCalibrator(Node):
    def __init__(self):
        super().__init__('lidar_calibrator')

        # 訂閱障礙物資料
        self.obstacle_sub = self.create_subscription(
            Obstacles, 
            'raw_obstacles', 
            self.obstacle_callback, 
            10)
        
        # 發布計算出的距離
        self.distance_pub = self.create_publisher(
            Point,
            'distance',
            10)
        
        # 發布 Marker
        self.marker_pub = self.create_publisher(
            Marker,
            'nearest_obstacle',
            10)

        self.dis = Point()
        self.create_timer(1, self.distance_publisher)

    def obstacle_callback(self, msg):
        self.get_logger().info('Received obstacles')

        if msg.circles:  # 確保有障礙物
            # 找到最近的障礙物
            nearest_obs = min(msg.circles, key=lambda obs: np.sqrt(obs.center.x**2 + obs.center.y**2))
            distance = np.sqrt(nearest_obs.center.x**2 + nearest_obs.center.y**2)

            self.get_logger().info(
                f'Nearest obstacle at x: {nearest_obs.center.x:.2f}, y: {nearest_obs.center.y:.2f}, distance: {distance:.2f}'
            )

            # 更新 Point 訊息
            self.dis.x = nearest_obs.center.x
            self.dis.y = nearest_obs.center.y
            self.dis.z = distance  # 設定 z 為距離

            # 發布 Marker
            self.publish_marker(nearest_obs.center.x, nearest_obs.center.y)

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"  # 根據你的 TF 設定，可能需要改成 "odom" 或其他
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nearest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1  # 調整大小
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # 設定為藍色
        marker.lifetime.sec = 1  # 1 秒後自動消失

        self.marker_pub.publish(marker)

    def distance_publisher(self):
        self.distance_pub.publish(self.dis)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
