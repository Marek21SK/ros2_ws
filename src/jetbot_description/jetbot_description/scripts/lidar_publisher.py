import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.5, self.publish_scan)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar'
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.01
        scan.time_increment = (1.0 / 40) / 360
        scan.range_min = 0.12
        scan.range_max = 3.5
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [1.0 + 0.1 * i / num_readings for i in range(num_readings)]  
        self.publisher_.publish(scan)
        #self.get_logger().info('Publishing fake LIDAR data')

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
