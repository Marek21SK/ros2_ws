#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.reset_sub = self.create_subscription(Bool, '/reset_path', self.reset_callback, 10)
        self.path = Path()
        self.path.header.frame_id = "odom"

    def reset_callback(self, msg):
        if msg.data:
            self.path.poses = []
            self.get_logger().info("Trajektória resetovaná.")

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()

        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
