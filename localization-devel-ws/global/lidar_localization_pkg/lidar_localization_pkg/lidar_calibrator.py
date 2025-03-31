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
        self.adjust_factor = 1.8
        self.pose_sub = self.create_subscription(
            Point,
            'pose',
            self.pose_callback,
            10)

        self.obstacle_sub = self.create_subscription(
            Obstacles, 
            'raw_obstacles', 
            self.obstacle_callback, 
            10)
        
        self.distance_pub = self.create_publisher(
            Point,
            'distance',
            10)
       
        self.marker_pub = self.create_publisher(
            Marker,
            'nearest_obstacle',
            10)

        self.target_pose = None  
        self.dis = Point()
        self.create_timer(1, self.distance_publisher)

    def pose_callback(self, msg):
        self.target_pose = np.array([msg.x, msg.y])
        self.get_logger().info(f'Target pose set to x: {msg.x:.2f}, y: {msg.y:.2f}')

    def obstacle_callback(self, msg):
        if self.target_pose is None:
            self.get_logger().warn("No target pose received yet. Skipping obstacle processing.")
            return

        if msg.circles:  
            distances = [
                np.linalg.norm(np.array([obs.center.x, obs.center.y]) - self.target_pose)
                for obs in msg.circles
            ]
            min_index = np.argmin(distances)  
            nearest_obs = msg.circles[min_index]
            distance = distances[min_index]

            self.dis.x = nearest_obs.center.x
            self.dis.y = nearest_obs.center.y
            self.dis.z = distance  

            self.publish_marker(nearest_obs.center.x, nearest_obs.center.y, nearest_obs.radius)
        
            self.dis.x = nearest_obs.center.x + (1-nearest_obs.center.x) * self.adjust_factor *0.01 * np.cos(theta)
            self.dis.y = nearest_obs.center.y + (1-nearest_obs.center.y) * self.adjust_factor *0.01 * np.sin(theta)

            theta = np.arctan2(self.dis.y - self.target_pose[1], self.dis.x - self.target_pose[0])

             self.get_logger().info(
                f'target obs at x: {self.target_pose[0]:.5f}, y: {self.target_pose[1]:.5f}, '
                f'Nearest obstacle to target at x: {nearest_obs.center.x:.5f}, y: {nearest_obs.center.y:.5f}, '
                f'adjusted x: {self.dis.x:.5f}, y: {self.dis.y:.5f}, '
                f'distance: {distance:.5f}, radius: {nearest_obs.radius:.5f}'
            )

    def publish_marker(self, x, y, radius):
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nearest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = radius * 2  # 確保顯示的圓球與實際障礙物大小一致
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  
        marker.lifetime.sec = 1  

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
