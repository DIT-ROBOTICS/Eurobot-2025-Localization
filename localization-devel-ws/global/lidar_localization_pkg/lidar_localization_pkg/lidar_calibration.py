import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

class LidarCalibrationNode(Node):
    def __init__(self):
        super().__init__('lidar_calibration_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, scan):
        scan_len = len(scan.ranges)
        # caclulate a few middle ranges in the scan data. odd and even cases
        if scan_len % 2 == 0:
            mid_index = scan_len // 2
            dist = (scan.ranges[mid_index] + scan.ranges[mid_index + 1])/2
        else:
            mid_index = scan_len // 2
            dist = (scan.ranges[mid_index] + scan.ranges[mid_index + 1]) / 2
        self.get_logger().info('Middle range: {}'.format(dist))

def main(args=None):
    rclpy.init(args=args)
    lidarcalibration = LidarCalibrationNode()
    rclpy.spin(lidarcalibration)
    lidarcalibration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()