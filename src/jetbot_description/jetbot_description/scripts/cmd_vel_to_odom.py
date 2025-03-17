#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class CmdVelToOdom(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_odom')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Premenné pre odometriu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

        # Čas na publikovanie odometrie
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        # Výpočet zmien
        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        delta_theta = self.angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = (self.theta + delta_theta + math.pi) % (2 * math.pi) - math.pi

        # Publikovanie odometrie
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom)

        # Publikovanie TF transformácie
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
