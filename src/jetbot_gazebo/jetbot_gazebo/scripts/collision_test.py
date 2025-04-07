#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CollisionTestNode(Node):
    def __init__(self):
        super().__init__('collision_test')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        # self.get_logger().info('Publikujem pohyb vpred...')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
