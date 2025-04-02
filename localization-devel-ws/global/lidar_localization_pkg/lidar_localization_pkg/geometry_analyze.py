import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

import numpy as np

# subscribe to topic /candidates, which is a MarkerArray topic
# it gives the position and radius of the three landmarks used to calculate the position
# now i want to find the geometry properties of the three landmarks under different conditions
# such as: when the robot is rotating, or the relationship between the robot position and geometry

class GeometryAnalyze(Node):

    def __init__(self):

        super().__init__('geometry_analyze')

        self.subscription = self.create_subscription(
            MarkerArray,
            '/candidates',
            self.candidates_callback,
            10)
        # self.subscription = self.create_subscription(
        #     PoseWithCovarianceStamped, 
        #     'final_pose',
        #     self.pred_pose_callback,
        #     10
        # )
        self.subscription  # prevent unused variable warning

        self.publisher_angles = self.create_publisher(Point, '/geometry_angles', 10)
        self.publisher_sides = self.create_publisher(Point, '/geometry_sides', 10)

    def get_geometry_angles(self):
        # Calculate angles between landmarks
        if len(self.landmarks) >= 3:
                
            angle_a = np.arctan2(
                self.landmarks[1].y - self.landmarks[0].y,
                self.landmarks[1].x - self.landmarks[0].x
            ) - np.arctan2(
                self.landmarks[2].y - self.landmarks[0].y,
                self.landmarks[2].x - self.landmarks[0].x
            )
            angle_b = np.arctan2(
                self.landmarks[0].y - self.landmarks[1].y,
                self.landmarks[0].x - self.landmarks[1].x
            ) - np.arctan2(
                self.landmarks[2].y - self.landmarks[1].y,
                self.landmarks[2].x - self.landmarks[1].x
            )
            angle_c = np.arctan2(
                self.landmarks[0].y - self.landmarks[2].y,
                self.landmarks[0].x - self.landmarks[2].x
            ) - np.arctan2(
                self.landmarks[1].y - self.landmarks[2].y,
                self.landmarks[1].x - self.landmarks[2].x
            )

            # Create a Point message to publish the angles
            angles_msg = Point()
            angles_msg.x = self.angle_limit_check(angle_a)
            angles_msg.y = self.angle_limit_check(angle_b)
            angles_msg.z = self.angle_limit_check(angle_c)

            # Publish the angles
            self.publisher_angles.publish(angles_msg)
        return
    
    def get_geometry_sides(self):
        # Calculate sides of the triangle formed by the landmarks
        if len(self.landmarks) >= 3:
            side_a = np.sqrt(
                (self.landmarks[1].x - self.landmarks[2].x)**2 +
                (self.landmarks[1].y - self.landmarks[2].y)**2
            )
            side_b = np.sqrt(
                (self.landmarks[0].x - self.landmarks[2].x)**2 +
                (self.landmarks[0].y - self.landmarks[2].y)**2
            )
            side_c = np.sqrt(
                (self.landmarks[0].x - self.landmarks[1].x)**2 +
                (self.landmarks[0].y - self.landmarks[1].y)**2
            )

            # Create a Point message to publish the sides
            sides_msg = Point()
            sides_msg.x = side_a
            sides_msg.y = side_b
            sides_msg.z = side_c

            # Publish the sides
            self.publisher_sides.publish(sides_msg)
        return

    def candidates_callback(self, msg):
        # /candidates.markers[:]{id==0}.pose.position.x
        self.landmarks = []
        self.landmarks_radius = [] # haven't set up in lidar_localization_pkg yet
        for marker in msg.markers:
            if marker.id == 0:
                self.landmarks.append(marker.pose.position)
                self.landmarks_radius.append(marker.scale.x)
            elif marker.id == 1:
                self.landmarks.append(marker.pose.position)
                self.landmarks_radius.append(marker.scale.x)
            elif marker.id == 2:
                self.landmarks.append(marker.pose.position)
                self.landmarks_radius.append(marker.scale.x)
        self.get_geometry_angles()
        self.get_geometry_sides()
    
    def angle_limit_check(self, angle):
        # Check if the angle is within the range of -pi to pi
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle
        
def main(args=None):
    rclpy.init(args=args)

    geometry_analyze = GeometryAnalyze()

    rclpy.spin(geometry_analyze)

    # Destroy the node explicitly
    geometry_analyze.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()