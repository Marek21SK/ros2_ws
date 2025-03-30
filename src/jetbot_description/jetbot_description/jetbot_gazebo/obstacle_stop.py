#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleStop(Node):
    def __init__(self):
        super().__init__('obstacle_stop')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 0.44  # 44 cm (7cm je dlzka od predku robota k LIDAR-u)
        self.angle_range = 30 # rozsah v stupňoch

    def laser_callback(self, msg):
        total_angles = len(msg.ranges)
        angle_increment = msg.angle_increment
        center_index = total_angles // 2

        range_offset = int(math.radians(self.angle_range) / angle_increment)

        front_ranges = [
            msg.ranges[i] for i in range(center_index - range_offset, center_index + range_offset)
            if not math.isinf(msg.ranges[i])
        ]

        if front_ranges:
            min_distance = min(front_ranges)
            #self.get_logger().info(f'Minimálna vzdialenosť pred robotom: {min_distance:.2f} m')
            if min_distance <= self.safe_distance:
                self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        #self.get_logger().info('Prekážka! Zastavujem robota.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
