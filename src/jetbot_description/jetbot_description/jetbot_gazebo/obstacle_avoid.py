#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 0.44  # 44 cm
        self.angle_range = 30  # rozsah v stupňoch
        self.avoiding_obstacle = False

    def laser_callback(self, msg):
        total_angles = len(msg.ranges)
        angle_increment = msg.angle_increment
        center_index = total_angles // 2

        range_offset = int(math.radians(self.angle_range) / angle_increment)

        front_ranges = [msg.ranges[i] for i in range(center_index - range_offset, center_index + range_offset)
            if not math.isinf(msg.ranges[i])
        ]

        if not front_ranges:
            return

        min_distance = min(front_ranges)

        if min_distance <= self.safe_distance and not self.avoiding_obstacle:
            self.avoiding_obstacle = True
            self.avoid_obstacle()
        elif min_distance > self.safe_distance:
            self.avoiding_obstacle = False

    def avoid_obstacle(self):
        #self.get_logger().info('Obchádzam prekážku...')

        # 1. Otočenie doľava
        twist = Twist()
        twist.angular.z = 0.6
        self.publisher.publish(twist)
        time.sleep(1.5)
        self.publisher.publish(Twist())

        # 2. Dopredu
        twist = Twist()
        twist.linear.x = 0.3
        self.publisher.publish(twist)
        time.sleep(3.5)
        self.publisher.publish(Twist())

        # 3. Otočenie doprava
        twist = Twist()
        twist.angular.z = -0.6
        self.publisher.publish(twist)
        time.sleep(1.5)
        self.publisher.publish(Twist())

        # 4. Pohyb dopredu
        twist.linear.x = 0.3
        self.publisher.publish(twist)
        time.sleep(3.5)
        self.publisher.publish(Twist())

        #self.get_logger().info('Prekážka obídená.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
